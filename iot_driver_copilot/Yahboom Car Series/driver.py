import os
import threading
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from flask import Flask, Response, jsonify

# Load configuration from environment variables
DEVICE_IP = os.environ.get("DEVICE_IP", "127.0.0.1")
ROS_MASTER_URI = os.environ.get("ROS_MASTER_URI", f"http://{DEVICE_IP}:11311")
HTTP_SERVER_HOST = os.environ.get("HTTP_SERVER_HOST", "0.0.0.0")
HTTP_SERVER_PORT = int(os.environ.get("HTTP_SERVER_PORT", "8080"))
ROS_TOPIC_POINTCLOUD2 = os.environ.get("ROS_TOPIC_POINTCLOUD2", "/resultPointCloudFile")
ROS_TOPIC_MAP = os.environ.get("ROS_TOPIC_MAP", "/map")
ROS_TOPIC_OCTOMAP = os.environ.get("ROS_TOPIC_OCTOMAP", "/octomap_binary")
ROS_TOPIC_MARKERARRAY = os.environ.get("ROS_TOPIC_MARKERARRAY", "/occupied_cells_vis_array")

os.environ["ROS_MASTER_URI"] = ROS_MASTER_URI

app = Flask(__name__)
latest_pointcloud = {"data": None}
latest_map = {"data": None}
latest_octomap = {"data": None}
latest_markerarray = {"data": None}

def start_ros_node():
    rospy.init_node("yahboom_http_driver", anonymous=True, disable_signals=True)

    def pointcloud_callback(msg):
        latest_pointcloud["data"] = list(pc2.read_points(msg, field_names=None, skip_nans=True))

    def map_callback(msg):
        # OccupancyGrid is a 2D map
        latest_map["data"] = {
            "info": {
                "width": msg.info.width,
                "height": msg.info.height,
                "resolution": msg.info.resolution
            },
            "data": list(msg.data)
        }

    def octomap_callback(msg):
        # Binary octomap
        latest_octomap["data"] = {
            "header": {
                "seq": msg.header.seq,
                "stamp": msg.header.stamp.to_sec(),
                "frame_id": msg.header.frame_id
            },
            "binary": msg.binary,
            "id": msg.id,
            "resolution": msg.resolution,
            "data": list(msg.data)
        }

    def markerarray_callback(msg):
        # MarkerArray contains visualization markers
        latest_markerarray["data"] = [marker.id for marker in msg.markers]

    rospy.Subscriber(ROS_TOPIC_POINTCLOUD2, PointCloud2, pointcloud_callback)
    try:
        from nav_msgs.msg import OccupancyGrid
        rospy.Subscriber(ROS_TOPIC_MAP, OccupancyGrid, map_callback)
    except ImportError:
        pass
    try:
        from octomap_msgs.msg import Octomap
        rospy.Subscriber(ROS_TOPIC_OCTOMAP, Octomap, octomap_callback)
    except ImportError:
        pass
    try:
        from visualization_msgs.msg import MarkerArray
        rospy.Subscriber(ROS_TOPIC_MARKERARRAY, MarkerArray, markerarray_callback)
    except ImportError:
        pass

    rospy.spin()

@app.route("/cmd", methods=["GET"])
def get_status():
    return jsonify({"status": "ok", "description": "0990"})

@app.route("/pointcloud", methods=["GET"])
def get_pointcloud():
    if latest_pointcloud["data"] is not None:
        return Response(
            jsonify({"points": latest_pointcloud["data"]}).data,
            mimetype="application/json"
        )
    else:
        return jsonify({"error": "No point cloud data received yet."}), 503

@app.route("/map", methods=["GET"])
def get_map():
    if latest_map["data"] is not None:
        return Response(
            jsonify(latest_map["data"]).data,
            mimetype="application/json"
        )
    else:
        return jsonify({"error": "No map data received yet."}), 503

@app.route("/octomap", methods=["GET"])
def get_octomap():
    if latest_octomap["data"] is not None:
        return Response(
            jsonify(latest_octomap["data"]).data,
            mimetype="application/json"
        )
    else:
        return jsonify({"error": "No octomap data received yet."}), 503

@app.route("/markers", methods=["GET"])
def get_markers():
    if latest_markerarray["data"] is not None:
        return Response(
            jsonify({"marker_ids": latest_markerarray["data"]}).data,
            mimetype="application/json"
        )
    else:
        return jsonify({"error": "No marker array data received yet."}), 503

def ros_thread():
    try:
        start_ros_node()
    except Exception as e:
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    t = threading.Thread(target=ros_thread, daemon=True)
    t.start()
    app.run(host=HTTP_SERVER_HOST, port=HTTP_SERVER_PORT, threaded=True)