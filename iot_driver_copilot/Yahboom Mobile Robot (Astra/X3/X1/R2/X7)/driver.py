import os
import io
import threading
import time
import json

from flask import Flask, request, Response, jsonify, stream_with_context
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist

import numpy as np
import cv2
from cv_bridge import CvBridge

# Environment variable configuration
ROS_MASTER_URI = os.environ.get("ROS_MASTER_URI")
DEVICE_IP = os.environ.get("DEVICE_IP", "localhost")
SERVER_HOST = os.environ.get("SERVER_HOST", "0.0.0.0")
SERVER_PORT = int(os.environ.get("SERVER_PORT", "8080"))
RGB_TOPIC = os.environ.get("RGB_TOPIC", "/camera/color/image_raw")
DEPTH_TOPIC = os.environ.get("DEPTH_TOPIC", "/camera/depth/image_raw")
CMD_VEL_TOPIC = os.environ.get("CMD_VEL_TOPIC", "/cmd_vel")
STOP_SERVICE = os.environ.get("STOP_SERVICE", "/emergency_stop")
HSV_PARAM_TOPIC = os.environ.get("HSV_PARAM_TOPIC", "/params/hsv")
PID_PARAM_TOPIC = os.environ.get("PID_PARAM_TOPIC", "/params/pid")
MODE_TOPIC = os.environ.get("MODE_TOPIC", "/robot_mode")
MAP_ACTION_TOPIC = os.environ.get("MAP_ACTION_TOPIC", "/map_action")

app = Flask(__name__)
bridge = CvBridge()

# Shared image buffers
latest_rgb = {"img": None, "stamp": 0}
latest_depth = {"img": None, "stamp": 0}

def ros_init():
    rospy.init_node("yahboom_http_driver", anonymous=True, disable_signals=True)

def rgb_callback(msg):
    try:
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        _, jpeg = cv2.imencode('.jpg', cv_img)
        latest_rgb["img"] = jpeg.tobytes()
        latest_rgb["stamp"] = time.time()
    except Exception:
        pass

def depth_callback(msg):
    try:
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        norm_depth = cv2.normalize(cv_img, None, 0, 255, cv2.NORM_MINMAX)
        norm_depth = np.uint8(norm_depth)
        color_depth = cv2.applyColorMap(norm_depth, cv2.COLORMAP_JET)
        _, jpeg = cv2.imencode('.jpg', color_depth)
        latest_depth["img"] = jpeg.tobytes()
        latest_depth["stamp"] = time.time()
    except Exception:
        pass

def ros_spin_thread():
    rospy.Subscriber(RGB_TOPIC, Image, rgb_callback, queue_size=1)
    rospy.Subscriber(DEPTH_TOPIC, Image, depth_callback, queue_size=1)
    rospy.spin()

@app.route("/rgb", methods=["GET"])
def stream_rgb():
    def gen():
        while True:
            if latest_rgb["img"] is not None:
                frame = latest_rgb["img"]
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            time.sleep(0.05)
    return Response(stream_with_context(gen()), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/depth", methods=["GET"])
def stream_depth():
    def gen():
        while True:
            if latest_depth["img"] is not None:
                frame = latest_depth["img"]
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            time.sleep(0.05)
    return Response(stream_with_context(gen()), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/param", methods=["POST"])
def adjust_param():
    data = request.get_json(force=True)
    result = {}
    if "hsv" in data:
        pub = rospy.Publisher(HSV_PARAM_TOPIC, String, queue_size=1)
        pub.publish(String(json.dumps(data["hsv"])))
        result["hsv"] = "sent"
    if "pid" in data:
        pub = rospy.Publisher(PID_PARAM_TOPIC, String, queue_size=1)
        pub.publish(String(json.dumps(data["pid"])))
        result["pid"] = "sent"
    return jsonify({"status": "ok", "details": result})

@app.route("/stop", methods=["POST"])
def stop_cmd():
    try:
        rospy.wait_for_service(STOP_SERVICE, timeout=2)
        stop_srv = rospy.ServiceProxy(STOP_SERVICE, Empty)
        stop_srv()
        return jsonify({"status": "ok", "message": "Emergency stop sent"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route("/map", methods=["POST"])
def map_action():
    data = request.get_json(force=True)
    pub = rospy.Publisher(MAP_ACTION_TOPIC, String, queue_size=1)
    pub.publish(String(json.dumps(data)))
    return jsonify({"status": "ok", "message": "Map action sent"})

@app.route("/speed", methods=["POST"])
def send_speed():
    data = request.get_json(force=True)
    twist = Twist()
    twist.linear.x = data.get('linear', 0.0)
    twist.angular.z = data.get('angular', 0.0)
    pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=1)
    pub.publish(twist)
    return jsonify({"status": "ok", "message": "Speed command sent"})

@app.route("/mode", methods=["POST"])
def set_mode():
    data = request.get_json(force=True)
    pub = rospy.Publisher(MODE_TOPIC, String, queue_size=1)
    pub.publish(String(str(data.get("mode", ""))))
    return jsonify({"status": "ok", "message": "Mode switch sent"})

def start_ros_thread():
    thread = threading.Thread(target=ros_spin_thread)
    thread.daemon = True
    thread.start()

if __name__ == "__main__":
    ros_init()
    start_ros_thread()
    app.run(host=SERVER_HOST, port=SERVER_PORT, threaded=True)