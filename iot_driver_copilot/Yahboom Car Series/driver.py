import os
import threading
import json
import base64
from http.server import BaseHTTPRequestHandler, HTTPServer
import socketserver
import rospy
from std_srvs.srv import Empty
from sensor_msgs.msg import PointCloud2
from octomap_msgs.msg import Octomap
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray

# === Environment Variables ===
DEVICE_IP = os.environ.get("DEVICE_IP", "127.0.0.1")
ROS_MASTER_URI = os.environ.get("ROS_MASTER_URI", "http://%s:11311" % DEVICE_IP)
SERVER_HOST = os.environ.get("SERVER_HOST", "0.0.0.0")
SERVER_PORT = int(os.environ.get("SERVER_PORT", "8080"))

os.environ["ROS_MASTER_URI"] = ROS_MASTER_URI

# === ROS Node Initialization in a background thread ===
def ros_spin_thread():
    rospy.init_node("yahboom_car_driver_http", anonymous=True, disable_signals=True)
    rospy.spin()

ros_thread = threading.Thread(target=ros_spin_thread, daemon=True)
ros_thread.start()

# === Data Storage for Latest Messages ===
ros_data = {
    "octomap_binary": None,
    "octomap_full": None,
    "occupied_cells_vis_array": None,
    "octomap_point_cloud_centers": None,
    "map": None,
    "resultPointCloudFile.pcd": None
}

def store_octomap_binary(msg):
    ros_data['octomap_binary'] = msg

def store_octomap_full(msg):
    ros_data['octomap_full'] = msg

def store_occupied_cells_vis_array(msg):
    ros_data['occupied_cells_vis_array'] = msg

def store_octomap_point_cloud_centers(msg):
    ros_data['octomap_point_cloud_centers'] = msg

def store_map(msg):
    ros_data['map'] = msg

def store_pcd(msg):
    # Placeholder for PCD, could be implemented via a service or file readout
    ros_data['resultPointCloudFile.pcd'] = None

# === Subscriber Initialization ===
def start_ros_subscribers():
    while not rospy.core.is_initialized():
        rospy.sleep(0.1)
    rospy.Subscriber("/octomap_binary", Octomap, store_octomap_binary, queue_size=1)
    rospy.Subscriber("/octomap_full", Octomap, store_octomap_full, queue_size=1)
    rospy.Subscriber("/occupied_cells_vis_array", MarkerArray, store_occupied_cells_vis_array, queue_size=1)
    rospy.Subscriber("/octomap_point_cloud_centers", PointCloud2, store_octomap_point_cloud_centers, queue_size=1)
    rospy.Subscriber("/map", OccupancyGrid, store_map, queue_size=1)
    # No direct PCD topic, would need to be handled differently

sub_thread = threading.Thread(target=start_ros_subscribers, daemon=True)
sub_thread.start()

# === HTTP Handler ===
class YahboomCarHandler(BaseHTTPRequestHandler):
    def _set_headers(self, content_type="application/json"):
        self.send_response(200)
        self.send_header('Content-type', content_type)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()

    def do_GET(self):
        if self.path.startswith("/cmd"):
            data = {
                "octomap_binary": self.serialize_octomap(ros_data["octomap_binary"]),
                "octomap_full": self.serialize_octomap(ros_data["octomap_full"]),
                "occupied_cells_vis_array": self.serialize_marker_array(ros_data["occupied_cells_vis_array"]),
                "octomap_point_cloud_centers": self.serialize_pointcloud2(ros_data["octomap_point_cloud_centers"]),
                "map": self.serialize_occupancy_grid(ros_data["map"]),
                "resultPointCloudFile.pcd": self.get_pcd_file()
            }
            self._set_headers()
            self.wfile.write(json.dumps(data, default=str).encode('utf-8'))
        else:
            self.send_response(404)
            self.end_headers()
            self.wfile.write(b'Not found\n')

    # --- Serialization Functions ---
    def serialize_octomap(self, msg):
        if msg is None:
            return None
        # Octomap contains a binary map (bytes), type info, header, etc.
        return {
            "header": {
                "seq": msg.header.seq,
                "stamp": msg.header.stamp.to_sec(),
                "frame_id": msg.header.frame_id
            },
            "binary": msg.binary,
            "id": msg.id,
            "resolution": msg.resolution,
            "data": base64.b64encode(msg.data).decode('ascii') if msg.data else None
        }

    def serialize_marker_array(self, msg):
        if msg is None:
            return None
        # We'll just output the number of markers and their types for brevity
        return {
            "count": len(msg.markers),
            "markers": [marker.type for marker in msg.markers]
        }

    def serialize_pointcloud2(self, msg):
        if msg is None:
            return None
        # Output header and first 64 bytes of data
        return {
            "header": {
                "seq": msg.header.seq,
                "stamp": msg.header.stamp.to_sec(),
                "frame_id": msg.header.frame_id
            },
            "height": msg.height,
            "width": msg.width,
            "fields": [f.name for f in msg.fields],
            "is_bigendian": msg.is_bigendian,
            "point_step": msg.point_step,
            "row_step": msg.row_step,
            "is_dense": msg.is_dense,
            "data_head": base64.b64encode(msg.data[:64]).decode('ascii') if msg.data else None
        }

    def serialize_occupancy_grid(self, msg):
        if msg is None:
            return None
        return {
            "header": {
                "seq": msg.header.seq,
                "stamp": msg.header.stamp.to_sec(),
                "frame_id": msg.header.frame_id
            },
            "info": {
                "map_load_time": msg.info.map_load_time.to_sec(),
                "resolution": msg.info.resolution,
                "width": msg.info.width,
                "height": msg.info.height,
                "origin": {
                    "position": {
                        "x": msg.info.origin.position.x,
                        "y": msg.info.origin.position.y,
                        "z": msg.info.origin.position.z
                    },
                    "orientation": {
                        "x": msg.info.origin.orientation.x,
                        "y": msg.info.origin.orientation.y,
                        "z": msg.info.origin.orientation.z,
                        "w": msg.info.origin.orientation.w
                    }
                }
            },
            "data_head": msg.data[:64] if msg.data else None
        }

    def get_pcd_file(self):
        # If the robot produces a .pcd file and stores it, read and return as base64.
        pcd_path = "/tmp/resultPointCloudFile.pcd"
        if os.path.exists(pcd_path):
            with open(pcd_path, "rb") as f:
                data = f.read()
            return base64.b64encode(data).decode('ascii')
        return None

# === Threaded HTTP Server ===
class ThreadedHTTPServer(socketserver.ThreadingMixIn, HTTPServer):
    daemon_threads = True

def run():
    server_address = (SERVER_HOST, SERVER_PORT)
    httpd = ThreadedHTTPServer(server_address, YahboomCarHandler)
    print("Starting YahboomCar HTTP driver on {}:{}".format(SERVER_HOST, SERVER_PORT))
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.server_close()

if __name__ == '__main__':
    run()