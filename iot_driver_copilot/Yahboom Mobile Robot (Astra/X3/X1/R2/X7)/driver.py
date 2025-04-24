import os
import json
import threading
import io
from flask import Flask, request, Response, jsonify, stream_with_context, abort
from sensor_msgs.msg import Image
import rospy
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import String
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist

# === Environment Variables ===
ROS_MASTER_URI = os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
ROS_HOSTNAME = os.environ.get("ROS_HOSTNAME", "localhost")
DEVICE_NAME = os.environ.get("DEVICE_NAME", "yahboom_robot")
SERVER_HOST = os.environ.get("SERVER_HOST", "0.0.0.0")
SERVER_PORT = int(os.environ.get("SERVER_PORT", "8000"))
RGB_TOPIC = os.environ.get("RGB_TOPIC", "/camera/rgb/image_raw")
DEPTH_TOPIC = os.environ.get("DEPTH_TOPIC", "/camera/depth/image_raw")
CMD_VEL_TOPIC = os.environ.get("CMD_VEL_TOPIC", "/cmd_vel")
STOP_SERVICE = os.environ.get("STOP_SERVICE", "/stop")
PARAM_SERVICE = os.environ.get("PARAM_SERVICE", "/set_param")
MODE_TOPIC = os.environ.get("MODE_TOPIC", "/mode_switch")
MAP_SERVICE = os.environ.get("MAP_SERVICE", "/map_service")
ROS_NAMESPACE = os.environ.get("ROS_NAMESPACE", "")

# === ROS Initialization ===
os.environ["ROS_MASTER_URI"] = ROS_MASTER_URI
os.environ["ROS_HOSTNAME"] = ROS_HOSTNAME
rospy.init_node("yahboom_driver_http", anonymous=True, disable_signals=True)

bridge = CvBridge()
app = Flask(__name__)

# === Video Stream Buffer ===
frame_lock = threading.Lock()
rgb_frame = None
depth_frame = None

def rgb_callback(msg):
    global rgb_frame
    try:
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        with frame_lock:
            ret, jpeg = cv2.imencode('.jpg', cv_img)
            if ret:
                rgb_frame = jpeg.tobytes()
    except Exception:
        pass

def depth_callback(msg):
    global depth_frame
    try:
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        # Normalize depth for visualization
        norm_img = cv2.normalize(cv_img, None, 0, 255, cv2.NORM_MINMAX)
        norm_img = norm_img.astype(np.uint8)
        color_img = cv2.applyColorMap(norm_img, cv2.COLORMAP_JET)
        with frame_lock:
            ret, jpeg = cv2.imencode('.jpg', color_img)
            if ret:
                depth_frame = jpeg.tobytes()
    except Exception:
        pass

# Start asynchronous ROS subscribers for image topics
def ros_spin():
    rospy.Subscriber(RGB_TOPIC, Image, rgb_callback, queue_size=1)
    rospy.Subscriber(DEPTH_TOPIC, Image, depth_callback, queue_size=1)
    rospy.spin()

threading.Thread(target=ros_spin, daemon=True).start()

# === MJPEG Streaming for RGB Image ===
@app.route('/stream/rgb')
def stream_rgb():
    def generate():
        while True:
            with frame_lock:
                frame = rgb_frame
            if frame:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            rospy.sleep(0.05)
    return Response(stream_with_context(generate()), mimetype='multipart/x-mixed-replace; boundary=frame')

# === MJPEG Streaming for Depth Image ===
@app.route('/stream/depth')
def stream_depth():
    def generate():
        while True:
            with frame_lock:
                frame = depth_frame
            if frame:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            rospy.sleep(0.05)
    return Response(stream_with_context(generate()), mimetype='multipart/x-mixed-replace; boundary=frame')

# === Command Endpoints ===

@app.route('/param', methods=['POST'])
def set_param():
    data = request.get_json(force=True)
    try:
        # Example: publish new param as a JSON string on a ROS topic
        pub = rospy.Publisher(PARAM_SERVICE, String, queue_size=1)
        pub.publish(json.dumps(data))
        return jsonify({"status": "ok", "message": "Parameters updated."})
    except Exception as e:
        abort(500, str(e))

@app.route('/stop', methods=['POST'])
def stop():
    try:
        # Send zero velocity or call stop service
        if rospy.has_param(STOP_SERVICE):
            rospy.wait_for_service(STOP_SERVICE, timeout=2)
            stop_srv = rospy.ServiceProxy(STOP_SERVICE, Empty)
            stop_srv()
        else:
            pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=1)
            twist = Twist()
            pub.publish(twist)
        return jsonify({"status": "ok", "message": "Stop signal sent."})
    except Exception as e:
        abort(500, str(e))

@app.route('/map', methods=['POST'])
def map_action():
    data = request.get_json(force=True)
    action = data.get("action")
    options = data.get("options", {})
    try:
        # Publish to map_service or topic
        pub = rospy.Publisher(MAP_SERVICE, String, queue_size=1)
        msg = {"action": action, "options": options}
        pub.publish(json.dumps(msg))
        return jsonify({"status": "ok", "message": f"Map action '{action}' performed."})
    except Exception as e:
        abort(500, str(e))

@app.route('/speed', methods=['POST'])
def set_speed():
    data = request.get_json(force=True)
    linear = data.get("linear", 0.0)
    angular = data.get("angular", 0.0)
    try:
        pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=1)
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        pub.publish(twist)
        return jsonify({"status": "ok", "message": "Speed command sent."})
    except Exception as e:
        abort(500, str(e))

@app.route('/mode', methods=['POST'])
def set_mode():
    data = request.get_json(force=True)
    mode = data.get("mode", "")
    try:
        pub = rospy.Publisher(MODE_TOPIC, String, queue_size=1)
        pub.publish(mode)
        return jsonify({"status": "ok", "message": f"Mode set to '{mode}'."})
    except Exception as e:
        abort(500, str(e))

@app.route('/')
def index():
    return '''
    <html>
    <head><title>Yahboom Robot HTTP Driver</title></head>
    <body>
        <h1>Yahboom Robot HTTP Driver</h1>
        <ul>
            <li><a href="/stream/rgb">Live RGB Camera Stream (MJPEG)</a></li>
            <li><a href="/stream/depth">Live Depth Camera Stream (MJPEG)</a></li>
        </ul>
        <p>Use POST to /param, /stop, /map, /speed, /mode for robot control.</p>
    </body>
    </html>
    '''

if __name__ == '__main__':
    app.run(host=SERVER_HOST, port=SERVER_PORT, threaded=True)