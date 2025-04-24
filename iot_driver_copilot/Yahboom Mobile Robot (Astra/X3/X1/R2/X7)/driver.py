import os
import io
import json
import threading
import time
from flask import Flask, request, Response, jsonify, stream_with_context
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from nav_msgs.srv import GetMap
from std_srvs.srv import EmptyRequest
from cv_bridge import CvBridge
import cv2
import numpy as np

# Environment variables
ROS_MASTER_URI = os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
ROS_HOSTNAME = os.environ.get("ROS_HOSTNAME", "localhost")
ROS_NAMESPACE = os.environ.get("ROS_NAMESPACE", "")
SERVER_HOST = os.environ.get("SERVER_HOST", "0.0.0.0")
SERVER_PORT = int(os.environ.get("SERVER_PORT", "8080"))
RGB_TOPIC = os.environ.get("RGB_TOPIC", "/camera/rgb/image_raw")
DEPTH_TOPIC = os.environ.get("DEPTH_TOPIC", "/camera/depth/image_raw")
SPEED_TOPIC = os.environ.get("SPEED_TOPIC", "/cmd_vel")
STOP_TOPIC = os.environ.get("STOP_TOPIC", "/cmd_vel")
PARAM_SERVICE = os.environ.get("PARAM_SERVICE", "/set_parameters")
MODE_TOPIC = os.environ.get("MODE_TOPIC", "/robot_mode")
MAP_SERVICE = os.environ.get("MAP_SERVICE", "/dynamic_map")
MAP_SAVE_SERVICE = os.environ.get("MAP_SAVE_SERVICE", "/map_saver/save_map")
HSV_PARAM_NAME = os.environ.get("HSV_PARAM_NAME", "/hsv_params")
PID_PARAM_NAME = os.environ.get("PID_PARAM_NAME", "/pid_params")
BRIDGE = CvBridge()

app = Flask(__name__)

# ROS Initialization
ros_initialized = False
def ros_init():
    global ros_initialized
    if not ros_initialized:
        rospy.init_node('yahboom_driver_http', anonymous=True, disable_signals=True)
        ros_initialized = True

# ---------- Image Streaming Section ----------

class StreamBuffer:
    def __init__(self):
        self.frame = None
        self.lock = threading.Lock()
    def update(self, frame):
        with self.lock:
            self.frame = frame
    def get(self):
        with self.lock:
            return self.frame

rgb_buffer = StreamBuffer()
depth_buffer = StreamBuffer()

def rgb_callback(msg):
    try:
        cv_img = BRIDGE.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        _, jpeg = cv2.imencode('.jpg', cv_img)
        rgb_buffer.update(jpeg.tobytes())
    except Exception:
        pass

def depth_callback(msg):
    try:
        cv_img = BRIDGE.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        norm = cv2.normalize(cv_img, None, 0, 255, cv2.NORM_MINMAX)
        norm = np.uint8(norm)
        color = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
        _, jpeg = cv2.imencode('.jpg', color)
        depth_buffer.update(jpeg.tobytes())
    except Exception:
        pass

def start_image_subscribers():
    ros_init()
    threading.Thread(target=lambda: rospy.Subscriber(RGB_TOPIC, Image, rgb_callback), daemon=True).start()
    threading.Thread(target=lambda: rospy.Subscriber(DEPTH_TOPIC, Image, depth_callback), daemon=True).start()

@app.route('/stream/rgb')
def stream_rgb():
    def generate():
        while True:
            frame = rgb_buffer.get()
            if frame is not None:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            time.sleep(0.07)
    return Response(stream_with_context(generate()), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/stream/depth')
def stream_depth():
    def generate():
        while True:
            frame = depth_buffer.get()
            if frame is not None:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            time.sleep(0.07)
    return Response(stream_with_context(generate()), mimetype='multipart/x-mixed-replace; boundary=frame')

# ---------- Command Endpoints ----------

@app.route('/param', methods=['POST'])
def set_param():
    ros_init()
    data = request.get_json(force=True)
    resp = {}
    # HSV
    if 'hsv' in data:
        try:
            rospy.set_param(HSV_PARAM_NAME, data['hsv'])
            resp['hsv'] = 'ok'
        except Exception as e:
            resp['hsv'] = str(e)
    # PID
    if 'pid' in data:
        try:
            rospy.set_param(PID_PARAM_NAME, data['pid'])
            resp['pid'] = 'ok'
        except Exception as e:
            resp['pid'] = str(e)
    return jsonify(resp)

@app.route('/stop', methods=['POST'])
def stop():
    ros_init()
    pub = rospy.Publisher(STOP_TOPIC, Twist, queue_size=1)
    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    for _ in range(3):
        pub.publish(twist)
        time.sleep(0.05)
    return jsonify({'status': 'stopped'})

@app.route('/map', methods=['POST'])
def map_manage():
    ros_init()
    data = request.get_json(force=True)
    action = data.get('action', '')
    resp = {}
    if action == 'save':
        try:
            rospy.wait_for_service(MAP_SAVE_SERVICE, timeout=3)
            save_map = rospy.ServiceProxy(MAP_SAVE_SERVICE, Empty)
            save_map(EmptyRequest())
            resp['result'] = 'map_saved'
        except Exception as e:
            resp['result'] = str(e)
    elif action == 'nav':
        # Navigation initiation logic placeholder (not implemented)
        resp['result'] = 'nav_not_implemented'
    else:
        resp['result'] = 'unknown_action'
    return jsonify(resp)

@app.route('/speed', methods=['POST'])
def set_speed():
    ros_init()
    data = request.get_json(force=True)
    linear = float(data.get('linear', 0.0))
    angular = float(data.get('angular', 0.0))
    pub = rospy.Publisher(SPEED_TOPIC, Twist, queue_size=1)
    twist = Twist()
    twist.linear.x = linear
    twist.angular.z = angular
    for _ in range(2):
        pub.publish(twist)
        time.sleep(0.05)
    return jsonify({'status': 'speed_set', 'linear': linear, 'angular': angular})

@app.route('/mode', methods=['POST'])
def set_mode():
    ros_init()
    data = request.get_json(force=True)
    mode = data.get('mode')
    if not mode:
        return jsonify({'error': 'Missing mode'}), 400
    pub = rospy.Publisher(MODE_TOPIC, String, queue_size=1)
    msg = String(data=str(mode))
    for _ in range(2):
        pub.publish(msg)
        time.sleep(0.05)
    return jsonify({'status': 'mode_set', 'mode': mode})

# ---------- Main ----------

if __name__ == '__main__':
    start_image_subscribers()
    app.run(host=SERVER_HOST, port=SERVER_PORT, threaded=True)