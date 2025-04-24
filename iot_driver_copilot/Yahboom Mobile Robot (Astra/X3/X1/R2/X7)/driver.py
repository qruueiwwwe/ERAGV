import os
import io
import threading
import json
import time
import cv2
import numpy as np
from flask import Flask, Response, request, jsonify
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from cv_bridge import CvBridge

# --- ENVIRONMENT VARIABLES ---
ROS_MASTER_URI = os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')
ROS_IP = os.environ.get('ROS_IP', '127.0.0.1')
ROS_NAMESPACE = os.environ.get('ROS_NAMESPACE', '')
HTTP_HOST = os.environ.get('HTTP_HOST', '0.0.0.0')
HTTP_PORT = int(os.environ.get('HTTP_PORT', '8080'))
CAMERA_RGB_TOPIC = os.environ.get('CAMERA_RGB_TOPIC', '/camera/color/image_raw')
CAMERA_DEPTH_TOPIC = os.environ.get('CAMERA_DEPTH_TOPIC', '/camera/depth/image_raw')
SPEED_TOPIC = os.environ.get('SPEED_TOPIC', '/cmd_vel')
MODE_TOPIC = os.environ.get('MODE_TOPIC', '/mode')
STOP_TOPIC = os.environ.get('STOP_TOPIC', '/stop')
PARAM_TOPIC = os.environ.get('PARAM_TOPIC', '/param')
MAP_ACTION_TOPIC = os.environ.get('MAP_ACTION_TOPIC', '/map_action')
MAP_SERVICE_SAVE = os.environ.get('MAP_SERVICE_SAVE', '/save_map')
MAP_SERVICE_NAV = os.environ.get('MAP_SERVICE_NAV', '/start_nav')
# ---

app = Flask(__name__)
bridge = CvBridge()

# --- ROS Node Startup ---
def start_ros_node():
    try:
        rospy.get_master().getPid()
    except:
        rospy.init_node("yahboom_http_driver", anonymous=True, disable_signals=True)

ros_thread = threading.Thread(target=start_ros_node)
ros_thread.daemon = True
ros_thread.start()
time.sleep(1)

# --- IMAGE STREAMING ---
latest_rgb_image = {"data": None, "time": 0}
latest_depth_image = {"data": None, "time": 0}

def rgb_callback(msg):
    latest_rgb_image["data"] = msg
    latest_rgb_image["time"] = time.time()

def depth_callback(msg):
    latest_depth_image["data"] = msg
    latest_depth_image["time"] = time.time()

rospy.Subscriber(CAMERA_RGB_TOPIC, Image, rgb_callback)
rospy.Subscriber(CAMERA_DEPTH_TOPIC, Image, depth_callback)

def gen_mjpeg(image_type='rgb'):
    while True:
        if image_type == 'rgb':
            img_msg = latest_rgb_image["data"]
        elif image_type == 'depth':
            img_msg = latest_depth_image["data"]
        else:
            img_msg = None
        if img_msg is not None:
            try:
                cv_img = bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8' if image_type=='rgb' else 'passthrough')
                if image_type == 'depth':
                    # Normalize depth for visualization
                    norm_img = cv2.normalize(cv_img, None, 0, 255, cv2.NORM_MINMAX)
                    norm_img = np.uint8(norm_img)
                    norm_img = cv2.applyColorMap(norm_img, cv2.COLORMAP_JET)
                    cv_img = norm_img
                ret, jpeg = cv2.imencode('.jpg', cv_img)
                if not ret:
                    continue
                frame = jpeg.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            except Exception:
                continue
        time.sleep(0.05)

@app.route("/stream/rgb")
def stream_rgb():
    return Response(gen_mjpeg('rgb'), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/stream/depth")
def stream_depth():
    return Response(gen_mjpeg('depth'), mimetype='multipart/x-mixed-replace; boundary=frame')

# --- CONTROL ENDPOINTS ---

@app.route("/speed", methods=["POST"])
def set_speed():
    try:
        data = request.get_json(force=True)
        linear = float(data.get('linear', 0.0))
        angular = float(data.get('angular', 0.0))
        pub = rospy.Publisher(SPEED_TOPIC, Twist, queue_size=1)
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        pub.publish(twist)
        return jsonify({"status": "ok", "msg": "Speed command sent"})
    except Exception as e:
        return jsonify({"status": "error", "msg": str(e)}), 400

@app.route("/mode", methods=["POST"])
def set_mode():
    try:
        data = request.get_json(force=True)
        mode = data.get('mode')
        if not mode:
            return jsonify({"status": "error", "msg": "No mode provided"}), 400
        pub = rospy.Publisher(MODE_TOPIC, String, queue_size=1)
        msg = String()
        msg.data = str(mode)
        pub.publish(msg)
        return jsonify({"status": "ok", "msg": f"Mode set to {mode}"})
    except Exception as e:
        return jsonify({"status": "error", "msg": str(e)}), 400

@app.route("/stop", methods=["POST"])
def stop_robot():
    try:
        pub = rospy.Publisher(STOP_TOPIC, String, queue_size=1)
        msg = String()
        msg.data = "stop"
        pub.publish(msg)
        return jsonify({"status": "ok", "msg": "Stop command sent"})
    except Exception as e:
        return jsonify({"status": "error", "msg": str(e)}), 400

@app.route("/param", methods=["POST"])
def set_param():
    try:
        data = request.get_json(force=True)
        pub = rospy.Publisher(PARAM_TOPIC, String, queue_size=1)
        msg = String()
        msg.data = json.dumps(data)
        pub.publish(msg)
        return jsonify({"status": "ok", "msg": "Parameter update sent"})
    except Exception as e:
        return jsonify({"status": "error", "msg": str(e)}), 400

@app.route("/map", methods=["POST"])
def map_action():
    try:
        data = request.get_json(force=True)
        action = data.get('action')
        if not action:
            return jsonify({"status": "error", "msg": "No action provided"}), 400
        if action == 'save':
            rospy.wait_for_service(MAP_SERVICE_SAVE, timeout=3)
            map_save = rospy.ServiceProxy(MAP_SERVICE_SAVE, Empty)
            map_save()
            return jsonify({"status": "ok", "msg": "Map save triggered"})
        elif action == 'nav':
            rospy.wait_for_service(MAP_SERVICE_NAV, timeout=3)
            map_nav = rospy.ServiceProxy(MAP_SERVICE_NAV, Empty)
            map_nav()
            return jsonify({"status": "ok", "msg": "Navigation started"})
        else:
            pub = rospy.Publisher(MAP_ACTION_TOPIC, String, queue_size=1)
            msg = String()
            msg.data = json.dumps(data)
            pub.publish(msg)
            return jsonify({"status": "ok", "msg": f"Custom map action {action} sent"})
    except Exception as e:
        return jsonify({"status": "error", "msg": str(e)}), 400

# --- ROOT PAGE FOR BROWSER ---
@app.route("/")
def index():
    return """
    <html>
      <head>
        <title>Yahboom Robot HTTP Driver</title>
      </head>
      <body>
        <h1>Yahboom Mobile Robot HTTP Driver</h1>
        <h2>Live Streams</h2>
        <div>
          <b>RGB Camera:</b><br>
          <img src="/stream/rgb" width="480"/><br>
          <b>Depth Camera:</b><br>
          <img src="/stream/depth" width="480"/>
        </div>
        <h2>API Endpoints</h2>
        <ul>
          <li>POST /param - Adjust system parameters (JSON: HSV/PID)</li>
          <li>POST /stop - Issue emergency stop</li>
          <li>POST /map - Map actions (JSON: {"action": "save"|"nav"|custom})</li>
          <li>POST /speed - Set speed (JSON: {"linear": 1.0, "angular": 0.5})</li>
          <li>POST /mode - Set mode (JSON: {"mode": "manual"/"autonomous"/...})</li>
        </ul>
      </body>
    </html>
    """

if __name__ == "__main__":
    app.run(host=HTTP_HOST, port=HTTP_PORT, threaded=True)