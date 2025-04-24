import os
import threading
import io
import json
import time

from flask import Flask, Response, request, jsonify, stream_with_context

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid

from cv_bridge import CvBridge
import cv2
import numpy as np

# Configuration from environment variables
ROS_MASTER_URI       = os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')
ROS_IP              = os.environ.get('ROS_IP', '127.0.0.1')
HTTP_SERVER_HOST    = os.environ.get('HTTP_SERVER_HOST', '0.0.0.0')
HTTP_SERVER_PORT    = int(os.environ.get('HTTP_SERVER_PORT', '8080'))
RGB_TOPIC           = os.environ.get('RGB_TOPIC', '/camera/rgb/image_raw')
DEPTH_TOPIC         = os.environ.get('DEPTH_TOPIC', '/camera/depth/image_raw')
CMD_VEL_TOPIC       = os.environ.get('CMD_VEL_TOPIC', '/cmd_vel')
PARAM_TOPIC         = os.environ.get('PARAM_TOPIC', '/param_tuning')
STOP_SERVICE_NAME   = os.environ.get('STOP_SERVICE_NAME', '/emergency_stop')
MAP_ACTION_TOPIC    = os.environ.get('MAP_ACTION_TOPIC', '/map_action')
MODE_TOPIC          = os.environ.get('MODE_TOPIC', '/mode_cmd')

# Initialize Flask app
app = Flask(__name__)

# Ensure ROS environment
os.environ["ROS_MASTER_URI"] = ROS_MASTER_URI
os.environ["ROS_IP"] = ROS_IP

# Initialize ROS (in a thread-safe way)
roscore_started = threading.Event()
bridge = CvBridge()

def ros_init():
    if not rospy.core.is_initialized():
        rospy.init_node('yahboom_http_driver', anonymous=True, disable_signals=True)
        roscore_started.set()

ros_init_thread = threading.Thread(target=ros_init)
ros_init_thread.daemon = True
ros_init_thread.start()
roscore_started.wait()

# ----------- Video Streaming Logic ----------- #
class ImageStreamer:
    def __init__(self, topic, encoding='bgr8', to_depth=False):
        self.topic = topic
        self.encoding = encoding
        self.to_depth = to_depth
        self.frame = None
        self.lock = threading.Lock()
        self.sub = rospy.Subscriber(self.topic, Image, self.callback, queue_size=1)

    def callback(self, msg):
        try:
            cv_img = bridge.imgmsg_to_cv2(msg, self.encoding)
            if self.to_depth:
                if len(cv_img.shape) == 2:
                    # Normalize for visualization
                    norm_img = cv2.normalize(cv_img, None, 0, 255, cv2.NORM_MINMAX)
                    norm_img = norm_img.astype(np.uint8)
                    cv_img = cv2.applyColorMap(norm_img, cv2.COLORMAP_JET)
                else:
                    cv_img = cv2.cvtColor(cv_img, cv2.COLOR_GRAY2BGR)
            _, jpeg = cv2.imencode('.jpg', cv_img)
            with self.lock:
                self.frame = jpeg.tobytes()
        except Exception as e:
            # Swallow errors for robustness
            pass

    def get_frame(self):
        with self.lock:
            return self.frame

rgb_streamer = ImageStreamer(RGB_TOPIC, encoding='bgr8')
depth_streamer = ImageStreamer(DEPTH_TOPIC, encoding='passthrough', to_depth=True)

def mjpeg_stream_generator(streamer):
    while True:
        frame = streamer.get_frame()
        if frame:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        else:
            time.sleep(0.05)

@app.route('/video/rgb')
def video_rgb():
    return Response(mjpeg_stream_generator(rgb_streamer),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/video/depth')
def video_depth():
    return Response(mjpeg_stream_generator(depth_streamer),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

# ----------- API Endpoints ----------- #

# /param: Adjust system parameters (HSV, PID, etc)
@app.route('/param', methods=['POST'])
def set_param():
    data = request.get_json(force=True)
    pub = rospy.Publisher(PARAM_TOPIC, String, queue_size=1)
    msg = String()
    msg.data = json.dumps(data)
    pub.publish(msg)
    return jsonify({'status': 'ok', 'sent': data})

# /stop: Emergency stop
@app.route('/stop', methods=['POST'])
def stop():
    try:
        rospy.wait_for_service(STOP_SERVICE_NAME, timeout=2.0)
        stop_srv = rospy.ServiceProxy(STOP_SERVICE_NAME, Empty)
        stop_srv()
        return jsonify({'status': 'ok', 'message': 'Emergency stop sent'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500

# /map: Manage map functions (save/nav)
@app.route('/map', methods=['POST'])
def map_action():
    data = request.get_json(force=True)
    pub = rospy.Publisher(MAP_ACTION_TOPIC, String, queue_size=1)
    msg = String()
    msg.data = json.dumps(data)
    pub.publish(msg)
    return jsonify({'status': 'ok', 'map_action': data})

# /speed: Send speed command
@app.route('/speed', methods=['POST'])
def set_speed():
    data = request.get_json(force=True)
    linear = data.get('linear', 0.0)
    angular = data.get('angular', 0.0)
    pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=1)
    twist = Twist()
    twist.linear.x = float(linear)
    twist.angular.z = float(angular)
    pub.publish(twist)
    return jsonify({'status': 'ok', 'sent': {'linear': linear, 'angular': angular}})

# /mode: Switch mode
@app.route('/mode', methods=['POST'])
def set_mode():
    data = request.get_json(force=True)
    mode = data.get('mode', None)
    if mode is None:
        return jsonify({'status': 'error', 'message': "Missing 'mode' in request"}), 400
    pub = rospy.Publisher(MODE_TOPIC, String, queue_size=1)
    msg = String()
    msg.data = str(mode)
    pub.publish(msg)
    return jsonify({'status': 'ok', 'sent_mode': mode})

# ----------- Main ----------- #
if __name__ == '__main__':
    app.run(host=HTTP_SERVER_HOST, port=HTTP_SERVER_PORT, threaded=True)