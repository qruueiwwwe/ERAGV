import os
import threading
import json
import time
import io

from flask import Flask, request, Response, jsonify, stream_with_context
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry, OccupancyGrid
from std_srvs.srv import Empty
from dynamic_reconfigure.msg import Config
from dynamic_reconfigure.srv import Reconfigure, ReconfigureRequest

from cv_bridge import CvBridge
import cv2
import numpy as np

# --- Environment variables ---
ROS_MASTER_URI = os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
ROS_HOSTNAME = os.environ.get("ROS_HOSTNAME", "localhost")
SERVER_HOST = os.environ.get("SERVER_HOST", "0.0.0.0")
SERVER_PORT = int(os.environ.get("SERVER_PORT", "8080"))
RGB_TOPIC = os.environ.get("RGB_TOPIC", "/camera/rgb/image_raw")
DEPTH_TOPIC = os.environ.get("DEPTH_TOPIC", "/camera/depth/image_raw")
CMD_VEL_TOPIC = os.environ.get("CMD_VEL_TOPIC", "/cmd_vel")
STOP_TOPIC = os.environ.get("STOP_TOPIC", "/stop")
MODE_TOPIC = os.environ.get("MODE_TOPIC", "/mode")
PARAM_SERVICE = os.environ.get("PARAM_SERVICE", "/set_parameters")
MAP_SERVICE = os.environ.get("MAP_SERVICE", "/map_manager")
BRIDGE = CvBridge()

app = Flask(__name__)

# --- ROS Node Initialization ---
def init_ros():
    if not rospy.core.is_initialized():
        rospy.init_node("yahboom_http_driver", anonymous=True, disable_signals=True)

init_ros()

# Publishers
cmd_vel_pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=10)
stop_pub = rospy.Publisher(STOP_TOPIC, String, queue_size=10)
mode_pub = rospy.Publisher(MODE_TOPIC, String, queue_size=10)

# --- Video Stream Buffers ---
latest_rgb = {"msg": None, "stamp": 0}
latest_depth = {"msg": None, "stamp": 0}

def rgb_callback(msg):
    latest_rgb["msg"] = msg
    latest_rgb["stamp"] = time.time()

def depth_callback(msg):
    latest_depth["msg"] = msg
    latest_depth["stamp"] = time.time()

rgb_sub = rospy.Subscriber(RGB_TOPIC, Image, rgb_callback, queue_size=1)
depth_sub = rospy.Subscriber(DEPTH_TOPIC, Image, depth_callback, queue_size=1)

# --- MJPEG Streaming Generators ---
def mjpeg_stream_generator(image_source, img_type="rgb"):
    while True:
        img_msg = image_source["msg"]
        if img_msg is not None:
            try:
                cv_img = BRIDGE.imgmsg_to_cv2(img_msg, desired_encoding="bgr8" if img_type == "rgb" else "passthrough")
                if img_type == "depth":
                    # Normalize and convert depth image to 8-bit for display
                    cv_img = np.nan_to_num(cv_img)
                    cv_img = (cv_img / np.max(cv_img) * 255).astype(np.uint8)
                ret, jpeg = cv2.imencode('.jpg', cv_img)
                if ret:
                    frame = jpeg.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            except Exception:
                pass
        time.sleep(0.05)

@app.route('/stream/rgb')
def stream_rgb():
    return Response(stream_with_context(mjpeg_stream_generator(latest_rgb, "rgb")),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/stream/depth')
def stream_depth():
    return Response(stream_with_context(mjpeg_stream_generator(latest_depth, "depth")),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

# --- API Endpoints ---

@app.route("/param", methods=["POST"])
def set_param():
    data = request.get_json(force=True)
    try:
        rospy.wait_for_service(PARAM_SERVICE, timeout=2)
        param_srv = rospy.ServiceProxy(PARAM_SERVICE, Reconfigure)
        req = ReconfigureRequest()
        for k, v in data.items():
            req.config.doubles.append(Config.DoubleParameter(name=k, value=float(v)))
        resp = param_srv(req)
        return jsonify({"status": "ok"}), 200
    except Exception as e:
        return jsonify({"status": "error", "error": str(e)}), 500

@app.route("/stop", methods=["POST"])
def stop():
    try:
        # Option 1: publish to a stop topic
        stop_pub.publish(String(data="stop"))
        # Option 2: send zero velocity
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        cmd_vel_pub.publish(twist)
        return jsonify({"status": "ok"}), 200
    except Exception as e:
        return jsonify({"status": "error", "error": str(e)}), 500

@app.route("/map", methods=["POST"])
def map_manage():
    data = request.get_json(force=True)
    action = data.get("action", "")
    options = data.get("options", {})
    try:
        rospy.wait_for_service(MAP_SERVICE, timeout=2)
        map_srv = rospy.ServiceProxy(MAP_SERVICE, Empty)
        if action in ["save", "nav"]:
            map_srv()
            return jsonify({"status": "ok", "action": action}), 200
        else:
            return jsonify({"status": "error", "error": "Invalid action"}), 400
    except Exception as e:
        return jsonify({"status": "error", "error": str(e)}), 500

@app.route("/speed", methods=["POST"])
def set_speed():
    data = request.get_json(force=True)
    linear = float(data.get("linear", 0))
    angular = float(data.get("angular", 0))
    twist = Twist()
    twist.linear.x = linear
    twist.angular.z = angular
    try:
        cmd_vel_pub.publish(twist)
        return jsonify({"status": "ok"}), 200
    except Exception as e:
        return jsonify({"status": "error", "error": str(e)}), 500

@app.route("/mode", methods=["POST"])
def set_mode():
    data = request.get_json(force=True)
    mode = data.get("mode", "")
    try:
        mode_pub.publish(String(data=mode))
        return jsonify({"status": "ok", "mode": mode}), 200
    except Exception as e:
        return jsonify({"status": "error", "error": str(e)}), 500

# --- Root/Info Page ---
@app.route("/")
def root():
    return jsonify({
        "device": "Yahboom Mobile Robot",
        "manufacturer": "Yahboom, Orbbec",
        "api": [
            {"method": "POST", "path": "/param", "desc": "Adjust system parameters"},
            {"method": "POST", "path": "/stop", "desc": "Emergency stop"},
            {"method": "POST", "path": "/map", "desc": "Map management (save/nav)"},
            {"method": "POST", "path": "/speed", "desc": "Set robot speed"},
            {"method": "POST", "path": "/mode", "desc": "Set robot mode"},
            {"method": "GET", "path": "/stream/rgb", "desc": "RGB camera MJPEG stream"},
            {"method": "GET", "path": "/stream/depth", "desc": "Depth camera MJPEG stream"}
        ]
    })

if __name__ == "__main__":
    app.run(host=SERVER_HOST, port=SERVER_PORT, threaded=True)