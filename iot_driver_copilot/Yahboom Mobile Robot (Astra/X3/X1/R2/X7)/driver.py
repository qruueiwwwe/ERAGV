import os
import io
import json
import threading
import time
from flask import Flask, Response, request, jsonify
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from std_msgs.msg import String
from nav_msgs.srv import GetMap
from cv_bridge import CvBridge
import cv2
import numpy as np

# Configuration from environment variables
ROS_MASTER_URI = os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')
ROS_HOSTNAME = os.environ.get('ROS_HOSTNAME', 'localhost')
HTTP_HOST = os.environ.get('HTTP_HOST', '0.0.0.0')
HTTP_PORT = int(os.environ.get('HTTP_PORT', '8080'))
RGB_TOPIC = os.environ.get('RGB_TOPIC', '/camera/rgb/image_raw')
DEPTH_TOPIC = os.environ.get('DEPTH_TOPIC', '/camera/depth/image_raw')
CMD_VEL_TOPIC = os.environ.get('CMD_VEL_TOPIC', '/cmd_vel')
STOP_SERVICE = os.environ.get('STOP_SERVICE', '/stop')
PARAM_SERVICE = os.environ.get('PARAM_SERVICE', '/set_params')
MODE_TOPIC = os.environ.get('MODE_TOPIC', '/mode_switch')
MAP_SERVICE = os.environ.get('MAP_SERVICE', '/map_manager')
MAP_TOPIC = os.environ.get('MAP_TOPIC', '/map')

app = Flask(__name__)
bridge = CvBridge()
rospy_inited = False

# Shared image buffers for streaming
rgb_image_lock = threading.Lock()
rgb_image_data = None

depth_image_lock = threading.Lock()
depth_image_data = None

def ros_init():
    global rospy_inited
    if not rospy_inited:
        os.environ['ROS_MASTER_URI'] = ROS_MASTER_URI
        os.environ['ROS_HOSTNAME'] = ROS_HOSTNAME
        rospy.init_node('yahboom_http_driver', anonymous=True, disable_signals=True)
        rospy_inited = True

def rgb_callback(msg):
    global rgb_image_data
    with rgb_image_lock:
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        ret, jpeg = cv2.imencode('.jpg', cv_img)
        if ret:
            rgb_image_data = jpeg.tobytes()

def depth_callback(msg):
    global depth_image_data
    with depth_image_lock:
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        norm_img = cv2.normalize(cv_img, None, 0, 255, cv2.NORM_MINMAX)
        norm_img = norm_img.astype(np.uint8)
        colorized = cv2.applyColorMap(norm_img, cv2.COLORMAP_JET)
        ret, jpeg = cv2.imencode('.jpg', colorized)
        if ret:
            depth_image_data = jpeg.tobytes()

def start_ros_subscribers():
    ros_init()
    rospy.Subscriber(RGB_TOPIC, Image, rgb_callback, queue_size=1, buff_size=2**24)
    rospy.Subscriber(DEPTH_TOPIC, Image, depth_callback, queue_size=1, buff_size=2**24)

streaming_started = False
def ensure_streaming():
    global streaming_started
    if not streaming_started:
        threading.Thread(target=start_ros_subscribers, daemon=True).start()
        streaming_started = True

def gen_mjpeg(get_image, lock):
    ensure_streaming()
    while True:
        with lock:
            frame = get_image()
        if frame is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        else:
            time.sleep(0.05)

@app.route('/rgb')
def stream_rgb():
    def get_image():
        return rgb_image_data
    return Response(gen_mjpeg(get_image, rgb_image_lock),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/depth')
def stream_depth():
    def get_image():
        return depth_image_data
    return Response(gen_mjpeg(get_image, depth_image_lock),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/param', methods=['POST'])
def set_param():
    ros_init()
    data = request.get_json(force=True)
    try:
        rospy.wait_for_service(PARAM_SERVICE, timeout=3)
        set_params = rospy.ServiceProxy(PARAM_SERVICE, Empty)
        set_params()
        return jsonify({"status": "success"}), 200
    except Exception as ex:
        return jsonify({"status": "error", "detail": str(ex)}), 500

@app.route('/stop', methods=['POST'])
def stop_robot():
    ros_init()
    try:
        rospy.wait_for_service(STOP_SERVICE, timeout=2)
        stop_srv = rospy.ServiceProxy(STOP_SERVICE, Empty)
        stop_srv()
        return jsonify({"status": "stopped"}), 200
    except Exception as ex:
        return jsonify({"status": "error", "detail": str(ex)}), 500

@app.route('/speed', methods=['POST'])
def set_speed():
    ros_init()
    data = request.get_json(force=True)
    linear = float(data.get('linear', 0.0))
    angular = float(data.get('angular', 0.0))
    pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=1)
    msg = Twist()
    msg.linear.x = linear
    msg.angular.z = angular
    # Publish in a thread to avoid blocking
    threading.Thread(target=lambda: [pub.publish(msg), time.sleep(0.1)], daemon=True).start()
    return jsonify({"status": "sent", "linear": linear, "angular": angular}), 200

@app.route('/mode', methods=['POST'])
def set_mode():
    ros_init()
    data = request.get_json(force=True)
    mode = str(data.get('mode', 'manual'))
    pub = rospy.Publisher(MODE_TOPIC, String, queue_size=1)
    msg = String(data=mode)
    threading.Thread(target=lambda: [pub.publish(msg), time.sleep(0.1)], daemon=True).start()
    return jsonify({"status": "mode_set", "mode": mode}), 200

@app.route('/map', methods=['POST'])
def map_action():
    ros_init()
    data = request.get_json(force=True)
    action = data.get('action', '')
    options = data.get('options', {})
    try:
        rospy.wait_for_service(MAP_SERVICE, timeout=3)
        map_srv = rospy.ServiceProxy(MAP_SERVICE, Empty)
        map_srv()
        return jsonify({"status": "map_action_sent", "action": action, "options": options}), 200
    except Exception as ex:
        return jsonify({"status": "error", "detail": str(ex)}), 500

@app.route('/map_image')
def get_map_image():
    ros_init()
    try:
        msg = rospy.wait_for_message(MAP_TOPIC, Image, timeout=3)
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        ret, jpeg = cv2.imencode('.jpg', cv_img)
        if ret:
            return Response(jpeg.tobytes(), mimetype='image/jpeg')
        else:
            return Response("Image conversion failed", status=500)
    except Exception as ex:
        return Response(str(ex), status=500)

@app.route('/')
def index():
    return '''
    <html>
    <head><title>Yahboom Mobile Robot HTTP Driver</title></head>
    <body>
    <h2>Yahboom Mobile Robot HTTP Driver</h2>
    <ul>
      <li><a href="/rgb">/rgb</a> (Live RGB Camera MJPEG Stream)</li>
      <li><a href="/depth">/depth</a> (Live Depth Camera MJPEG Stream)</li>
      <li><a href="/map_image">/map_image</a> (Current Map Image)</li>
      <li>POST /speed (JSON {"linear": float, "angular": float})</li>
      <li>POST /param (JSON for HSV/PID tuning)</li>
      <li>POST /stop (No body)</li>
      <li>POST /mode (JSON {"mode": string})</li>
      <li>POST /map (JSON {"action": string, "options": object})</li>
    </ul>
    </body>
    </html>
    '''

if __name__ == '__main__':
    app.run(host=HTTP_HOST, port=HTTP_PORT, threaded=True)
