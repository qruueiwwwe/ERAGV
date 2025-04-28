import os
import threading
import rospy
import rosgraph
import rosnode
from flask import Flask, Response, jsonify
from sensor_msgs.msg import PointCloud2
from octomap_msgs.msg import Octomap
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import OccupancyGrid
import std_msgs.msg
import rosbag
import struct
import time
import json
import signal
import sys

# =======================
# Environment Config
# =======================
DEVICE_IP = os.environ.get('DEVICE_IP', 'localhost')
ROS_MASTER_URI = os.environ.get('ROS_MASTER_URI', f'http://{DEVICE_IP}:11311')
SERVER_HOST = os.environ.get('SERVER_HOST', '0.0.0.0')
SERVER_PORT = int(os.environ.get('SERVER_PORT', '8080'))
ROS_NAMESPACE = os.environ.get('ROS_NAMESPACE', '')

# Topics (optionally overrideable)
OCTOMAP_BINARY_TOPIC = os.environ.get('OCTOMAP_BINARY_TOPIC', '/octomap_binary')
OCTOMAP_FULL_TOPIC = os.environ.get('OCTOMAP_FULL_TOPIC', '/octomap_full')
OCCUPIED_CELLS_TOPIC = os.environ.get('OCCUPIED_CELLS_TOPIC', '/occupied_cells_vis_array')
OCTOMAP_POINT_CLOUD_TOPIC = os.environ.get('OCTOMAP_POINT_CLOUD_TOPIC', '/octomap_point_cloud_centers')
MAP_TOPIC = os.environ.get('MAP_TOPIC', '/map')
RESULT_PCD_FILE = os.environ.get('RESULT_PCD_FILE', '/tmp/resultPointCloudFile.pcd')
# =======================

# Set ROS_MASTER_URI env for node
os.environ['ROS_MASTER_URI'] = ROS_MASTER_URI

# ==============
# ROS Bridge
# ==============

class RosDataBridge:
    def __init__(self):
        self.last_octomap_binary = None
        self.last_octomap_full = None
        self.last_occupied_cells = None
        self.last_octomap_point_cloud = None
        self.last_map = None
        self.last_octomap_binary_header = None
        self.last_octomap_full_header = None
        self.last_occupied_cells_header = None
        self.last_octomap_point_cloud_header = None
        self.last_map_header = None
        self.lock = threading.Lock()
        self.subscribers = []

    def start(self):
        if not rospy.core.is_initialized():
            rospy.init_node('yahboom_car_driver', anonymous=True, disable_signals=True)

        self.subscribers.append(rospy.Subscriber(OCTOMAP_BINARY_TOPIC, Octomap, self.octomap_binary_cb))
        self.subscribers.append(rospy.Subscriber(OCTOMAP_FULL_TOPIC, Octomap, self.octomap_full_cb))
        self.subscribers.append(rospy.Subscriber(OCCUPIED_CELLS_TOPIC, MarkerArray, self.occupied_cells_cb))
        self.subscribers.append(rospy.Subscriber(OCTOMAP_POINT_CLOUD_TOPIC, PointCloud2, self.octomap_point_cloud_cb))
        self.subscribers.append(rospy.Subscriber(MAP_TOPIC, OccupancyGrid, self.map_cb))

    def octomap_binary_cb(self, msg):
        with self.lock:
            self.last_octomap_binary = msg.data
            self.last_octomap_binary_header = msg.header

    def octomap_full_cb(self, msg):
        with self.lock:
            self.last_octomap_full = msg.data
            self.last_octomap_full_header = msg.header

    def occupied_cells_cb(self, msg):
        with self.lock:
            self.last_occupied_cells = msg
            # No header for MarkerArray

    def octomap_point_cloud_cb(self, msg):
        with self.lock:
            self.last_octomap_point_cloud = msg
            self.last_octomap_point_cloud_header = msg.header

    def map_cb(self, msg):
        with self.lock:
            self.last_map = msg
            self.last_map_header = msg.header

    # Data retrieval, returns data + header if available
    def get_octomap_binary(self):
        with self.lock:
            return self.last_octomap_binary, self.last_octomap_binary_header

    def get_octomap_full(self):
        with self.lock:
            return self.last_octomap_full, self.last_octomap_full_header

    def get_occupied_cells(self):
        with self.lock:
            return self.last_occupied_cells, None

    def get_octomap_point_cloud(self):
        with self.lock:
            return self.last_octomap_point_cloud, self.last_octomap_point_cloud_header

    def get_map(self):
        with self.lock:
            return self.last_map, self.last_map_header

ros_bridge = RosDataBridge()

# Start ROS in background thread so Flask can serve HTTP
def ros_spin():
    ros_bridge.start()
    rospy.spin()

ros_thread = threading.Thread(target=ros_spin, daemon=True)
ros_thread.start()

# ==============
# HTTP Server
# ==============

app = Flask(__name__)

@app.route('/cmd', methods=['GET'])
def cmd():
    return jsonify({"status": "ok", "desc": "0990"})

@app.route('/octomap_binary', methods=['GET'])
def get_octomap_binary():
    data, header = ros_bridge.get_octomap_binary()
    if data is None:
        return jsonify({"error": "No octomap_binary data available"}), 404
    # Return data as binary octomap file
    return Response(data, mimetype='application/octet-stream')

@app.route('/octomap_full', methods=['GET'])
def get_octomap_full():
    data, header = ros_bridge.get_octomap_full()
    if data is None:
        return jsonify({"error": "No octomap_full data available"}), 404
    return Response(data, mimetype='application/octet-stream')

@app.route('/occupied_cells_vis_array', methods=['GET'])
def get_occupied_cells_vis_array():
    marker_array, _ = ros_bridge.get_occupied_cells()
    if marker_array is None:
        return jsonify({"error": "No occupied_cells_vis_array data available"}), 404
    # For demo, convert to JSON
    markers = []
    for m in marker_array.markers:
        markers.append({
            "ns": m.ns,
            "id": m.id,
            "type": m.type,
            "action": m.action,
            "pose": {
                "position": {
                    "x": m.pose.position.x,
                    "y": m.pose.position.y,
                    "z": m.pose.position.z
                },
                "orientation": {
                    "x": m.pose.orientation.x,
                    "y": m.pose.orientation.y,
                    "z": m.pose.orientation.z,
                    "w": m.pose.orientation.w,
                }
            },
            "scale": {
                "x": m.scale.x,
                "y": m.scale.y,
                "z": m.scale.z,
            },
            "color": {
                "r": m.color.r,
                "g": m.color.g,
                "b": m.color.b,
                "a": m.color.a,
            },
        })
    return jsonify({"markers": markers})

@app.route('/octomap_point_cloud_centers', methods=['GET'])
def get_octomap_point_cloud_centers():
    msg, header = ros_bridge.get_octomap_point_cloud()
    if msg is None:
        return jsonify({"error": "No octomap_point_cloud_centers data available"}), 404
    # Return as binary PointCloud2 stream
    return Response(msg.data, mimetype='application/octet-stream')

@app.route('/map', methods=['GET'])
def get_map():
    msg, header = ros_bridge.get_map()
    if msg is None:
        return jsonify({"error": "No map data available"}), 404
    # Return as OccupancyGrid JSON (partial fields for demo)
    data = {
        "info": {
            "width": msg.info.width,
            "height": msg.info.height,
            "resolution": msg.info.resolution,
            "origin": {
                "position": {
                    "x": msg.info.origin.position.x,
                    "y": msg.info.origin.position.y,
                    "z": msg.info.origin.position.z,
                },
                "orientation": {
                    "x": msg.info.origin.orientation.x,
                    "y": msg.info.origin.orientation.y,
                    "z": msg.info.origin.orientation.z,
                    "w": msg.info.origin.orientation.w,
                }
            }
        },
        "data": list(msg.data[:1000])  # Limit for demo
    }
    return jsonify(data)

@app.route('/resultPointCloudFile.pcd', methods=['GET'])
def get_result_pcd():
    if not os.path.exists(RESULT_PCD_FILE):
        return jsonify({"error": "PCD file not found"}), 404
    def generate():
        with open(RESULT_PCD_FILE, 'rb') as f:
            while True:
                chunk = f.read(4096)
                if not chunk:
                    break
                yield chunk
    return Response(generate(), mimetype='application/octet-stream')

def handle_sigterm(*_):
    func = request.environ.get('werkzeug.server.shutdown')
    if func is not None:
        func()
    sys.exit(0)

signal.signal(signal.SIGTERM, handle_sigterm)

if __name__ == '__main__':
    app.run(host=SERVER_HOST, port=SERVER_PORT, threaded=True)