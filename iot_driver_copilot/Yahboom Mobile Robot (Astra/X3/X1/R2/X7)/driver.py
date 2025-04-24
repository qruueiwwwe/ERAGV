import os
import threading
import json
import io
import time

from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse, parse_qs
import socketserver

import cv2
from cv_bridge import CvBridge
import numpy as np

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from std_msgs.msg import Float32MultiArray

# --- ENVIRONMENT CONFIGURATION ---
ROS_MASTER_URI = os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')
ROS_HOSTNAME = os.environ.get('ROS_HOSTNAME', 'localhost')
ROS_NAMESPACE = os.environ.get('ROS_NAMESPACE', '')
ROBOT_IP = os.environ.get('ROBOT_IP', 'localhost')

SERVER_HOST = os.environ.get('SERVER_HOST', '0.0.0.0')
SERVER_PORT = int(os.environ.get('SERVER_PORT', 8000))

RGB_TOPIC = os.environ.get('RGB_TOPIC', '/camera/rgb/image_raw')
DEPTH_TOPIC = os.environ.get('DEPTH_TOPIC', '/camera/depth/image_raw')
SPEED_TOPIC = os.environ.get('SPEED_TOPIC', '/cmd_vel')
STOP_SERVICE = os.environ.get('STOP_SERVICE', '/emergency_stop')
PARAM_TOPIC = os.environ.get('PARAM_TOPIC', '/tuning_param')
MAP_ACTION_TOPIC = os.environ.get('MAP_ACTION_TOPIC', '/map_action')
MODE_TOPIC = os.environ.get('MODE_TOPIC', '/set_mode')

# --- ROS INITIALIZATION ---
os.environ['ROS_MASTER_URI'] = ROS_MASTER_URI
os.environ['ROS_HOSTNAME'] = ROS_HOSTNAME

rospy.init_node('yahboom_http_driver', anonymous=True, disable_signals=True)

cv_bridge = CvBridge()

# Shared image frames
latest_rgb_frame = {"img": None, "stamp": 0}
latest_depth_frame = {"img": None, "stamp": 0}
rgb_lock = threading.Lock()
depth_lock = threading.Lock()


def rgb_callback(msg):
    global latest_rgb_frame
    img = cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    with rgb_lock:
        latest_rgb_frame['img'] = img
        latest_rgb_frame['stamp'] = time.time()


def depth_callback(msg):
    global latest_depth_frame
    img = cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    norm_img = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX)
    norm_img = np.uint8(norm_img)
    depth_colored = cv2.applyColorMap(norm_img, cv2.COLORMAP_JET)
    with depth_lock:
        latest_depth_frame['img'] = depth_colored
        latest_depth_frame['stamp'] = time.time()


# ROS Subscribers
rospy.Subscriber(RGB_TOPIC, Image, rgb_callback, queue_size=1)
rospy.Subscriber(DEPTH_TOPIC, Image, depth_callback, queue_size=1)

# Publishers
speed_pub = rospy.Publisher(SPEED_TOPIC, Twist, queue_size=1)
param_pub = rospy.Publisher(PARAM_TOPIC, Float32MultiArray, queue_size=1)
map_action_pub = rospy.Publisher(MAP_ACTION_TOPIC, String, queue_size=1)
mode_pub = rospy.Publisher(MODE_TOPIC, String, queue_size=1)

# Service Proxy for Stop
try:
    rospy.wait_for_service(STOP_SERVICE, timeout=5)
    stop_srv = rospy.ServiceProxy(STOP_SERVICE, Empty)
except Exception:
    stop_srv = None


# --- HTTP SERVER HANDLER ---
class YahboomHandler(BaseHTTPRequestHandler):
    def _set_headers(self, content_type='application/json', code=200):
        self.send_response(code)
        self.send_header('Content-type', content_type)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()

    def do_OPTIONS(self):
        self.send_response(200, "ok")
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'POST, GET, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

    def do_GET(self):
        parsed_path = urlparse(self.path)
        if parsed_path.path == '/rgb':
            self._serve_mjpeg('rgb')
        elif parsed_path.path == '/depth':
            self._serve_mjpeg('depth')
        else:
            self._set_headers('application/json', 404)
            self.wfile.write(json.dumps({'error': 'Not found'}).encode())

    def _serve_mjpeg(self, img_type):
        self.send_response(200)
        self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        try:
            while True:
                if img_type == 'rgb':
                    with rgb_lock:
                        img = latest_rgb_frame['img'].copy() if latest_rgb_frame['img'] is not None else None
                else:
                    with depth_lock:
                        img = latest_depth_frame['img'].copy() if latest_depth_frame['img'] is not None else None

                if img is not None:
                    _, jpg = cv2.imencode('.jpg', img)
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                    self.wfile.write(jpg.tobytes())
                    self.wfile.write(b"\r\n")
                else:
                    blank = np.zeros((480, 640, 3), dtype=np.uint8)
                    _, jpg = cv2.imencode('.jpg', blank)
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                    self.wfile.write(jpg.tobytes())
                    self.wfile.write(b"\r\n")
                time.sleep(0.1)
        except BrokenPipeError:
            pass
        except Exception:
            pass

    def do_POST(self):
        parsed_path = urlparse(self.path)
        content_length = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(content_length) if content_length > 0 else b'{}'
        try:
            data = json.loads(body.decode('utf-8'))
        except Exception:
            data = {}

        if parsed_path.path == '/param':
            self._handle_param(data)
        elif parsed_path.path == '/stop':
            self._handle_stop()
        elif parsed_path.path == '/map':
            self._handle_map(data)
        elif parsed_path.path == '/speed':
            self._handle_speed(data)
        elif parsed_path.path == '/mode':
            self._handle_mode(data)
        else:
            self._set_headers('application/json', 404)
            self.wfile.write(json.dumps({'error': 'Not found'}).encode())

    def _handle_param(self, data):
        arr = Float32MultiArray()
        arr.data = []
        for v in data.values():
            try:
                arr.data.append(float(v))
            except Exception:
                pass
        param_pub.publish(arr)
        self._set_headers('application/json', 200)
        self.wfile.write(json.dumps({'status': 'ok'}).encode())

    def _handle_stop(self):
        if stop_srv:
            try:
                stop_srv()
                self._set_headers('application/json', 200)
                self.wfile.write(json.dumps({'status': 'stopped'}).encode())
            except Exception as e:
                self._set_headers('application/json', 500)
                self.wfile.write(json.dumps({'error': str(e)}).encode())
        else:
            self._set_headers('application/json', 500)
            self.wfile.write(json.dumps({'error': 'Stop service unavailable'}).encode())

    def _handle_map(self, data):
        try:
            action = data.get('action', 'save')
            options = data.get('options', {})
            msg = {'action': action, 'options': options}
            map_action_pub.publish(String(json.dumps(msg)))
            self._set_headers('application/json', 200)
            self.wfile.write(json.dumps({'status': 'map_action_sent'}).encode())
        except Exception as e:
            self._set_headers('application/json', 500)
            self.wfile.write(json.dumps({'error': str(e)}).encode())

    def _handle_speed(self, data):
        try:
            linear = data.get('linear', 0.0)
            angular = data.get('angular', 0.0)
            tw = Twist()
            tw.linear.x = float(linear)
            tw.angular.z = float(angular)
            speed_pub.publish(tw)
            self._set_headers('application/json', 200)
            self.wfile.write(json.dumps({'status': 'speed_commanded'}).encode())
        except Exception as e:
            self._set_headers('application/json', 500)
            self.wfile.write(json.dumps({'error': str(e)}).encode())

    def _handle_mode(self, data):
        try:
            mode = data.get('mode', '')
            if mode:
                mode_pub.publish(String(mode))
                self._set_headers('application/json', 200)
                self.wfile.write(json.dumps({'status': 'mode_switched'}).encode())
            else:
                self._set_headers('application/json', 400)
                self.wfile.write(json.dumps({'error': 'Missing mode'}).encode())
        except Exception as e:
            self._set_headers('application/json', 500)
            self.wfile.write(json.dumps({'error': str(e)}).encode())


class ThreadedHTTPServer(socketserver.ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def run_server():
    server = ThreadedHTTPServer((SERVER_HOST, SERVER_PORT), YahboomHandler)
    print(f"Yahboom HTTP driver running on {SERVER_HOST}:{SERVER_PORT}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down HTTP driver...")
        server.server_close()


if __name__ == '__main__':
    run_server()