import os
import threading
import time
import json
import io

from flask import Flask, Response, request, jsonify, abort

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from dynamic_reconfigure.srv import Reconfigure, ReconfigureRequest, DoubleParameter
from nav_msgs.srv import GetMap
from nav_msgs.msg import Odometry

import cv2
from cv_bridge import CvBridge

# Configuration via environment variables
ROS_MASTER_URI = os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')
ROS_HOSTNAME = os.environ.get('ROS_HOSTNAME', 'localhost')
DEVICE_NAMESPACE = os.environ.get('DEVICE_NAMESPACE', '')
SERVER_HOST = os.environ.get('SERVER_HOST', '0.0.0.0')
SERVER_PORT = int(os.environ.get('SERVER_PORT', '8080'))

RGB_IMAGE_TOPIC = os.environ.get('RGB_IMAGE_TOPIC', '/camera/rgb/image_raw')
DEPTH_IMAGE_TOPIC = os.environ.get('DEPTH_IMAGE_TOPIC', '/camera/depth/image_raw')
CMD_VEL_TOPIC = os.environ.get('CMD_VEL_TOPIC', '/cmd_vel')
STOP_SERVICE = os.environ.get('STOP_SERVICE', '/stop')
HSV_PARAM_SERVICE = os.environ.get('HSV_PARAM_SERVICE', '/set_hsv')
PID_PARAM_SERVICE = os.environ.get('PID_PARAM_SERVICE', '/set_pid')
MAP_SERVICE = os.environ.get('MAP_SERVICE', '/map_manager')
MODE_TOPIC = os.environ.get('MODE_TOPIC', '/mode_switch')

# ROS initialization
ros_env_loaded = False
if not rospy.core.is_initialized():
    os.environ['ROS_MASTER_URI'] = ROS_MASTER_URI
    os.environ['ROS_HOSTNAME'] = ROS_HOSTNAME
    ros_env_loaded = True

# Flask server
app = Flask(__name__)

bridge = CvBridge()

latest_rgb_frame = None
latest_depth_frame = None
rgb_lock = threading.Lock()
depth_lock = threading.Lock()

def rgb_image_callback(msg):
    global latest_rgb_frame
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with rgb_lock:
            latest_rgb_frame = cv_image
    except Exception as e:
        pass

def depth_image_callback(msg):
    global latest_depth_frame
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        normed = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX)
        depth_colored = cv2.applyColorMap(normed.astype('uint8'), cv2.COLORMAP_JET)
        with depth_lock:
            latest_depth_frame = depth_colored
    except Exception as e:
        pass

def ros_spin_thread():
    rospy.spin()

def start_ros_subscribers():
    rospy.init_node('yahboom_http_driver', anonymous=True, disable_signals=True)
    rospy.Subscriber(RGB_IMAGE_TOPIC, Image, rgb_image_callback)
    rospy.Subscriber(DEPTH_IMAGE_TOPIC, Image, depth_image_callback)
    # No spin here, handled in thread
    t = threading.Thread(target=ros_spin_thread, daemon=True)
    t.start()

@app.route('/rgb', methods=['GET'])
def get_rgb_stream():
    def gen():
        while True:
            with rgb_lock:
                frame = latest_rgb_frame.copy() if latest_rgb_frame is not None else None
            if frame is not None:
                ret, jpeg = cv2.imencode('.jpg', frame)
                if not ret:
                    continue
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
            else:
                time.sleep(0.1)
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/depth', methods=['GET'])
def get_depth_stream():
    def gen():
        while True:
            with depth_lock:
                frame = latest_depth_frame.copy() if latest_depth_frame is not None else None
            if frame is not None:
                ret, jpeg = cv2.imencode('.jpg', frame)
                if not ret:
                    continue
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
            else:
                time.sleep(0.1)
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/param', methods=['POST'])
def set_param():
    data = request.get_json(force=True)
    service = None
    if 'HSV' in data or 'hsv' in data:
        service = HSV_PARAM_SERVICE
        param_type = 'HSV'
        param_data = data['HSV'] if 'HSV' in data else data['hsv']
    elif 'PID' in data or 'pid' in data:
        service = PID_PARAM_SERVICE
        param_type = 'PID'
        param_data = data['PID'] if 'PID' in data else data['pid']
    else:
        return jsonify({"error": "No supported parameter in request"}), 400

    try:
        rospy.wait_for_service(service, timeout=2)
        set_param_srv = rospy.ServiceProxy(service, Reconfigure)
        req = ReconfigureRequest()
        for k, v in param_data.items():
            req.config.doubles.append(DoubleParameter(name=k, value=float(v)))
        set_param_srv(req)
        return jsonify({"status": "ok", "type": param_type})
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/stop', methods=['POST'])
def stop_cmd():
    try:
        rospy.wait_for_service(STOP_SERVICE, timeout=2)
        stop_srv = rospy.ServiceProxy(STOP_SERVICE, Empty)
        stop_srv()
        return jsonify({"status": "stopped"})
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/map', methods=['POST'])
def manage_map():
    data = request.get_json(force=True)
    action = data.get('action', '').lower()
    options = data.get('options', {})

    if action not in ['save', 'nav']:
        return jsonify({"error": "Unsupported action"}), 400
    try:
        rospy.wait_for_service(MAP_SERVICE, timeout=2)
        map_srv = rospy.ServiceProxy(MAP_SERVICE, String)
        req = json.dumps({'action': action, 'options': options})
        resp = map_srv(req)
        return jsonify({"status": "ok", "response": resp.data})
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/speed', methods=['POST'])
def set_speed():
    data = request.get_json(force=True)
    linear = float(data.get('linear', 0.0))
    angular = float(data.get('angular', 0.0))
    try:
        pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=1)
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        for _ in range(3):  # Publish a few times to ensure delivery
            pub.publish(twist)
            time.sleep(0.05)
        return jsonify({"status": "ok"})
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/mode', methods=['POST'])
def set_mode():
    data = request.get_json(force=True)
    mode = data.get('mode', '')
    if not mode:
        return jsonify({"error": "Mode not specified"}), 400
    try:
        pub = rospy.Publisher(MODE_TOPIC, String, queue_size=1)
        msg = String()
        msg.data = mode
        for _ in range(3):
            pub.publish(msg)
            time.sleep(0.05)
        return jsonify({"status": "ok"})
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/')
def index():
    return '''
    <h1>Yahboom Mobile Robot HTTP Driver</h1>
    <ul>
      <li><a href="/rgb">/rgb</a> - RGB camera MJPEG stream</li>
      <li><a href="/depth">/depth</a> - Depth camera MJPEG stream</li>
    </ul>
    <p>POST to /param, /speed, /stop, /mode, /map for control.</p>
    '''

if __name__ == '__main__':
    start_ros_subscribers()
    app.run(host=SERVER_HOST, port=SERVER_PORT, threaded=True)