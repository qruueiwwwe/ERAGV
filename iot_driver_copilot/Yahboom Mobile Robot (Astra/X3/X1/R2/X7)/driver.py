import os
import threading
import time
import json
import io

from flask import Flask, Response, request, jsonify, stream_with_context
import cv2
import numpy as np

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist

from cv_bridge import CvBridge

# Configuration via environment variables
ROS_MASTER_URI = os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
ROS_IP = os.environ.get("ROS_IP", "127.0.0.1")
ROBOT_NAMESPACE = os.environ.get("ROBOT_NAMESPACE", "")
CAMERA_TOPIC = os.environ.get("CAMERA_TOPIC", "/camera/rgb/image_raw")
DEPTH_TOPIC = os.environ.get("DEPTH_TOPIC", "/camera/depth/image_raw")
LIDAR_TOPIC = os.environ.get("LIDAR_TOPIC", "/scan")
HTTP_HOST = os.environ.get("HTTP_HOST", "0.0.0.0")
HTTP_PORT = int(os.environ.get("HTTP_PORT", "8080"))

PARAM_TOPIC = os.environ.get("PARAM_TOPIC", "/params")
STOP_TOPIC = os.environ.get("STOP_TOPIC", "/cmd_stop")
MAP_SERVICE = os.environ.get("MAP_SERVICE", "/map_service")
SPEED_TOPIC = os.environ.get("SPEED_TOPIC", "/cmd_vel")
MODE_TOPIC = os.environ.get("MODE_TOPIC", "/mode")

os.environ['ROS_MASTER_URI'] = ROS_MASTER_URI
os.environ['ROS_IP'] = ROS_IP

# Flask app
app = Flask(__name__)

# ROS Initialization
def ros_init():
    if not rospy.get_node_uri():
        rospy.init_node("yahboom_driver_server", anonymous=True, disable_signals=True)

ros_init()

bridge = CvBridge()

# Shared latest frames for streaming
latest_rgb = {"frame": None, "timestamp": 0}
latest_depth = {"frame": None, "timestamp": 0}

def rgb_callback(msg):
    try:
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        latest_rgb["frame"] = cv_img
        latest_rgb["timestamp"] = time.time()
    except Exception:
        pass

def depth_callback(msg):
    try:
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        # Normalize for display
        norm_img = cv2.normalize(cv_img, None, 0, 255, cv2.NORM_MINMAX)
        norm_img = norm_img.astype(np.uint8)
        depth_colored = cv2.applyColorMap(norm_img, cv2.COLORMAP_JET)
        latest_depth["frame"] = depth_colored
        latest_depth["timestamp"] = time.time()
    except Exception:
        pass

# Subscribe to camera topics
def subscribe_camera_topics():
    rospy.Subscriber(CAMERA_TOPIC, Image, rgb_callback, queue_size=1)
    rospy.Subscriber(DEPTH_TOPIC, Image, depth_callback, queue_size=1)

subscribe_camera_topics()

@app.route('/stream/rgb')
def stream_rgb():
    def gen():
        while True:
            frame = latest_rgb["frame"]
            if frame is not None:
                ret, jpeg = cv2.imencode('.jpg', frame)
                if ret:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
            time.sleep(0.04)  # ~25 FPS
    return Response(stream_with_context(gen()), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/stream/depth')
def stream_depth():
    def gen():
        while True:
            frame = latest_depth["frame"]
            if frame is not None:
                ret, jpeg = cv2.imencode('.jpg', frame)
                if ret:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
            time.sleep(0.1)  # ~10 FPS
    return Response(stream_with_context(gen()), mimetype='multipart/x-mixed-replace; boundary=frame')

# --- API endpoints ---

@app.route('/param', methods=['POST'])
def adjust_param():
    try:
        data = request.get_json(force=True)
    except Exception:
        return jsonify({"status": "error", "message": "Invalid JSON"}), 400

    pub = rospy.Publisher(PARAM_TOPIC, String, queue_size=1)
    # In actual implementation, would use service or custom msg for structured params
    pub.publish(String(json.dumps(data)))
    return jsonify({"status": "ok", "message": "Parameter sent"})

@app.route('/stop', methods=['POST'])
def stop_robot():
    pub = rospy.Publisher(STOP_TOPIC, String, queue_size=1)
    pub.publish(String("stop"))
    return jsonify({"status": "ok", "message": "Stop command sent"})

@app.route('/map', methods=['POST'])
def map_ops():
    try:
        data = request.get_json(force=True)
        action = data.get("action", "")
        options = data.get("options", {})
    except Exception:
        return jsonify({"status": "error", "message": "Invalid JSON"}), 400

    if action == "save":
        try:
            rospy.wait_for_service(MAP_SERVICE, timeout=2)
            srv = rospy.ServiceProxy(MAP_SERVICE, Empty)
            srv()
            return jsonify({"status": "ok", "message": "Map saved"})
        except Exception as e:
            return jsonify({"status": "error", "message": "Service call failed", "detail": str(e)}), 500
    elif action == "nav":
        # In real robots, this may require nav goal publishing
        # Here we simply acknowledge the command
        return jsonify({"status": "ok", "message": "Map navigation action accepted", "options": options})
    else:
        return jsonify({"status": "error", "message": "Unknown action"}), 400

@app.route('/speed', methods=['POST'])
def set_speed():
    try:
        data = request.get_json(force=True)
        linear = float(data.get("linear", 0))
        angular = float(data.get("angular", 0))
    except Exception:
        return jsonify({"status": "error", "message": "Invalid JSON or missing fields"}), 400

    pub = rospy.Publisher(SPEED_TOPIC, Twist, queue_size=1)
    cmd = Twist()
    cmd.linear.x = linear
    cmd.angular.z = angular
    pub.publish(cmd)
    return jsonify({"status": "ok", "message": "Speed command sent"})

@app.route('/mode', methods=['POST'])
def set_mode():
    try:
        data = request.get_json(force=True)
        mode = data.get("mode", "")
    except Exception:
        return jsonify({"status": "error", "message": "Invalid JSON"}), 400

    pub = rospy.Publisher(MODE_TOPIC, String, queue_size=1)
    pub.publish(String(mode))
    return jsonify({"status": "ok", "message": "Mode switch command sent"})

@app.route('/')
def index():
    return """
    <h2>Yahboom Robot HTTP Driver</h2>
    <ul>
      <li>RGB Camera: <a href="/stream/rgb">/stream/rgb</a></li>
      <li>Depth Camera: <a href="/stream/depth">/stream/depth</a></li>
      <li>Speed Command: POST /speed (json: {"linear":..., "angular":...})</li>
      <li>Stop: POST /stop</li>
      <li>Map: POST /map (json: {"action":"save"/"nav", "options":{}})</li>
      <li>Mode: POST /mode (json: {"mode":"manual"/"auto"/...})</li>
      <li>Parameter Tuning: POST /param (json: {"HSV":..., "PID":...})</li>
    </ul>
    """

def ros_spin_thread():
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == "__main__":
    # Run ROS spin in a background thread
    threading.Thread(target=ros_spin_thread, daemon=True).start()
    app.run(host=HTTP_HOST, port=HTTP_PORT, threaded=True)