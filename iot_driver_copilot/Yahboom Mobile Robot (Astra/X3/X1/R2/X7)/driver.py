import os
import io
import json
import threading
import time
from flask import Flask, request, Response, jsonify, stream_with_context
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Empty
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty as EmptySrv
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# Environment Variables
ROS_MASTER_URI = os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
ROS_IP = os.environ.get("ROS_IP", "localhost")
HTTP_HOST = os.environ.get("HTTP_HOST", "0.0.0.0")
HTTP_PORT = int(os.environ.get("HTTP_PORT", "8080"))
RGB_TOPIC = os.environ.get("RGB_TOPIC", "/camera/rgb/image_raw")
DEPTH_TOPIC = os.environ.get("DEPTH_TOPIC", "/camera/depth/image_raw")
CMD_VEL_TOPIC = os.environ.get("CMD_VEL_TOPIC", "/cmd_vel")
STOP_TOPIC = os.environ.get("STOP_TOPIC", "/stop")
MODE_TOPIC = os.environ.get("MODE_TOPIC", "/mode")
PARAM_SERVICE = os.environ.get("PARAM_SERVICE", "/set_parameters")
MAP_SERVICE = os.environ.get("MAP_SERVICE", "/map_manager")
MAP_ACTION_TOPIC = os.environ.get("MAP_ACTION_TOPIC", "/map_action")

# ROS Initialization Flag
ros_started = threading.Event()

# ROS Node Initialization
def init_ros():
    os.environ["ROS_MASTER_URI"] = ROS_MASTER_URI
    os.environ["ROS_IP"] = ROS_IP
    if not rospy.core.is_initialized():
        rospy.init_node("yahboom_driver_http_server", anonymous=True, disable_signals=True)
    ros_started.set()

ros_thread = threading.Thread(target=init_ros)
ros_thread.daemon = True
ros_thread.start()
ros_started.wait(timeout=5)

app = Flask(__name__)
bridge = CvBridge()

latest_rgb_image = {"msg": None, "stamp": 0}
latest_depth_image = {"msg": None, "stamp": 0}


def rgb_callback(msg):
    latest_rgb_image["msg"] = msg
    latest_rgb_image["stamp"] = time.time()

def depth_callback(msg):
    latest_depth_image["msg"] = msg
    latest_depth_image["stamp"] = time.time()


# Subscribe to image topics in background thread
def subscribe_images():
    rospy.Subscriber(RGB_TOPIC, Image, rgb_callback, queue_size=1)
    rospy.Subscriber(DEPTH_TOPIC, Image, depth_callback, queue_size=1)
image_thread = threading.Thread(target=subscribe_images)
image_thread.daemon = True
image_thread.start()


def gen_mjpeg(image_type):
    while True:
        if image_type == "rgb":
            data = latest_rgb_image
        else:
            data = latest_depth_image
        msg = data["msg"]
        if msg is not None:
            try:
                cv_img = bridge.imgmsg_to_cv2(msg, "bgr8" if image_type == "rgb" else "passthrough")
                if image_type == "depth":
                    norm = cv2.normalize(cv_img, None, 0, 255, cv2.NORM_MINMAX)
                    cv_img = cv2.applyColorMap(norm.astype(np.uint8), cv2.COLORMAP_JET)
                ret, jpeg = cv2.imencode('.jpg', cv_img)
                if ret:
                    frame = jpeg.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            except CvBridgeError:
                pass
        time.sleep(0.05)

@app.route("/rgb.mjpeg")
def rgb_mjpeg():
    return Response(stream_with_context(gen_mjpeg("rgb")),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/depth.mjpeg")
def depth_mjpeg():
    return Response(stream_with_context(gen_mjpeg("depth")),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route("/speed", methods=["POST"])
def set_speed():
    data = request.get_json(force=True)
    linear = float(data.get("linear", 0))
    angular = float(data.get("angular", 0))
    pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=1)
    twist = Twist()
    twist.linear.x = linear
    twist.angular.z = angular
    pub.publish(twist)
    return jsonify({"result": "ok", "linear": linear, "angular": angular})


@app.route("/stop", methods=["POST"])
def stop_robot():
    # Try stop service first, else publish zero velocity
    try:
        stop_pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=1)
        twist = Twist()
        stop_pub.publish(twist)
        # If there is a stop topic, publish an Empty message
        if STOP_TOPIC:
            stop_signal = rospy.Publisher(STOP_TOPIC, Empty, queue_size=1)
            stop_signal.publish(Empty())
    except Exception:
        pass
    return jsonify({"result": "stopped"})


@app.route("/param", methods=["POST"])
def param():
    data = request.get_json(force=True)
    # Assume a ROS service exists for setting parameters
    try:
        rospy.wait_for_service(PARAM_SERVICE, timeout=2)
        set_param = rospy.ServiceProxy(PARAM_SERVICE, EmptySrv)
        set_param()
    except Exception:
        # If service not available, fallback to publishing on /param topic if exists
        param_pub = rospy.Publisher("/param", String, queue_size=1)
        param_pub.publish(String(json.dumps(data)))
    return jsonify({"result": "parameter update requested", "payload": data})


@app.route("/map", methods=["POST"])
def map_action():
    data = request.get_json(force=True)
    action = data.get("action", "")
    options = data.get("options", {})
    # Try call a map manager service or publish an action string
    try:
        rospy.wait_for_service(MAP_SERVICE, timeout=2)
        map_srv = rospy.ServiceProxy(MAP_SERVICE, EmptySrv)
        map_srv()
    except Exception:
        if MAP_ACTION_TOPIC:
            map_pub = rospy.Publisher(MAP_ACTION_TOPIC, String, queue_size=1)
            payload = {"action": action, "options": options}
            map_pub.publish(String(json.dumps(payload)))
    return jsonify({"result": "map action requested", "action": action, "options": options})


@app.route("/mode", methods=["POST"])
def mode_switch():
    data = request.get_json(force=True)
    mode = data.get("mode", "manual")
    mode_pub = rospy.Publisher(MODE_TOPIC, String, queue_size=1)
    mode_pub.publish(String(mode))
    return jsonify({"result": "mode switch requested", "mode": mode})


@app.route("/")
def index():
    return """<html>
    <head><title>Yahboom Mobile Robot HTTP Driver</title></head>
    <body>
        <h1>Yahboom Mobile Robot HTTP Driver</h1>
        <ul>
            <li><a href="/rgb.mjpeg" target="_blank">RGB Camera Stream</a></li>
            <li><a href="/depth.mjpeg" target="_blank">Depth Camera Stream</a></li>
        </ul>
        <p>API endpoints:</p>
        <ul>
            <li>POST /speed {"linear": 1.0, "angular": 0.5}</li>
            <li>POST /stop {}</li>
            <li>POST /param {"HSV": {...}, "PID": {...}}</li>
            <li>POST /map {"action": "save", "options": {...}}</li>
            <li>POST /mode {"mode": "manual"}</li>
        </ul>
    </body>
    </html>
    """


if __name__ == "__main__":
    app.run(host=HTTP_HOST, port=HTTP_PORT, threaded=True)
