import os
import threading
import json
import cv2
import numpy as np
import rospy
from flask import Flask, Response, request, jsonify
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from cv_bridge import CvBridge

# Environment configuration
DEVICE_ROS_MASTER_URI = os.environ.get("DEVICE_ROS_MASTER_URI", "http://localhost:11311")
DEVICE_ROS_IP = os.environ.get("DEVICE_ROS_IP", "127.0.0.1")
SERVER_HOST = os.environ.get("SERVER_HOST", "0.0.0.0")
SERVER_PORT = int(os.environ.get("SERVER_PORT", "8080"))
RGB_IMAGE_TOPIC = os.environ.get("RGB_IMAGE_TOPIC", "/camera/rgb/image_raw")
DEPTH_IMAGE_TOPIC = os.environ.get("DEPTH_IMAGE_TOPIC", "/camera/depth/image_raw")
CMD_VEL_TOPIC = os.environ.get("CMD_VEL_TOPIC", "/cmd_vel")
STOP_SERVICE = os.environ.get("STOP_SERVICE", "/emergency_stop")
PARAM_SERVICE = os.environ.get("PARAM_SERVICE", "/set_param")
MODE_TOPIC = os.environ.get("MODE_TOPIC", "/robot_mode")
MAP_ACTION_TOPIC = os.environ.get("MAP_ACTION_TOPIC", "/map_action")

os.environ["ROS_MASTER_URI"] = DEVICE_ROS_MASTER_URI
os.environ["ROS_IP"] = DEVICE_ROS_IP

app = Flask(__name__)
bridge = CvBridge()

# ROS Node initialization
def ros_spin():
    if not rospy.core.is_initialized():
        rospy.init_node('yahboom_http_driver', anonymous=True, disable_signals=True)
threading.Thread(target=ros_spin, daemon=True).start()

# Global frame buffers
rgb_frame = {"img": None, "lock": threading.Lock()}
depth_frame = {"img": None, "lock": threading.Lock()}

def rgb_callback(msg):
    with rgb_frame["lock"]:
        rgb_frame["img"] = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

def depth_callback(msg):
    with depth_frame["lock"]:
        img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        # Normalize for visualization (optional)
        norm_img = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX)
        depth_frame["img"] = np.uint8(norm_img)

def ensure_ros_subscribers():
    if not hasattr(ensure_ros_subscribers, "inited"):
        rospy.Subscriber(RGB_IMAGE_TOPIC, Image, rgb_callback, queue_size=1)
        rospy.Subscriber(DEPTH_IMAGE_TOPIC, Image, depth_callback, queue_size=1)
        ensure_ros_subscribers.inited = True
ensure_ros_subscribers()

def gen_rgb_stream():
    while True:
        with rgb_frame["lock"]:
            img = rgb_frame["img"].copy() if rgb_frame["img"] is not None else None
        if img is not None:
            ret, jpeg = cv2.imencode('.jpg', img)
            if ret:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
        else:
            # Black frame as placeholder
            blank = np.zeros((240, 320, 3), np.uint8)
            ret, jpeg = cv2.imencode('.jpg', blank)
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
        rospy.sleep(0.05)

def gen_depth_stream():
    while True:
        with depth_frame["lock"]:
            img = depth_frame["img"].copy() if depth_frame["img"] is not None else None
        if img is not None:
            colored = cv2.applyColorMap(img, cv2.COLORMAP_JET)
            ret, jpeg = cv2.imencode('.jpg', colored)
            if ret:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
        else:
            blank = np.zeros((240, 320, 3), np.uint8)
            ret, jpeg = cv2.imencode('.jpg', blank)
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
        rospy.sleep(0.05)

@app.route('/rgb')
def rgb_stream():
    return Response(gen_rgb_stream(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/depth')
def depth_stream():
    return Response(gen_depth_stream(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/speed', methods=['POST'])
def set_speed():
    try:
        data = request.get_json(force=True)
        twist = Twist()
        twist.linear.x = float(data.get('linear', 0.0))
        twist.angular.z = float(data.get('angular', 0.0))
        pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=1)
        pub.publish(twist)
        return jsonify({"status": "ok"}), 200
    except Exception as e:
        return jsonify({"status": "fail", "error": str(e)}), 400

@app.route('/stop', methods=['POST'])
def stop_robot():
    try:
        rospy.wait_for_service(STOP_SERVICE, timeout=2)
        stop_srv = rospy.ServiceProxy(STOP_SERVICE, Empty)
        stop_srv()
        return jsonify({"status": "ok"}), 200
    except Exception as e:
        # Fallback: publish zero velocity if no stop service
        try:
            pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=1)
            twist = Twist()
            pub.publish(twist)
        except Exception:
            pass
        return jsonify({"status": "fail", "error": str(e)}), 400

@app.route('/param', methods=['POST'])
def set_param():
    try:
        data = request.get_json(force=True)
        rospy.wait_for_service(PARAM_SERVICE, timeout=2)
        set_param_srv = rospy.ServiceProxy(PARAM_SERVICE, String)
        resp = set_param_srv(String(json.dumps(data)))
        return jsonify({"status": "ok", "response": resp.data}), 200
    except Exception as e:
        return jsonify({"status": "fail", "error": str(e)}), 400

@app.route('/mode', methods=['POST'])
def set_mode():
    try:
        data = request.get_json(force=True)
        mode = data.get('mode', '')
        pub = rospy.Publisher(MODE_TOPIC, String, queue_size=1)
        pub.publish(String(mode))
        return jsonify({"status": "ok"}), 200
    except Exception as e:
        return jsonify({"status": "fail", "error": str(e)}), 400

@app.route('/map', methods=['POST'])
def map_action():
    try:
        data = request.get_json(force=True)
        pub = rospy.Publisher(MAP_ACTION_TOPIC, String, queue_size=1)
        pub.publish(String(json.dumps(data)))
        return jsonify({"status": "ok"}), 200
    except Exception as e:
        return jsonify({"status": "fail", "error": str(e)}), 400

@app.route('/')
def root_info():
    return jsonify({
        "endpoints": [
            {"method": "GET", "path": "/rgb", "description": "RGB camera stream (HTTP MJPEG)"},
            {"method": "GET", "path": "/depth", "description": "Depth image stream (HTTP MJPEG)"},
            {"method": "POST", "path": "/speed", "description": "Set robot speed"},
            {"method": "POST", "path": "/stop", "description": "Emergency stop"},
            {"method": "POST", "path": "/param", "description": "Set HSV/PID or other parameters"},
            {"method": "POST", "path": "/mode", "description": "Switch robot mode"},
            {"method": "POST", "path": "/map", "description": "Perform map actions"}
        ]
    })

if __name__ == '__main__':
    app.run(host=SERVER_HOST, port=SERVER_PORT, threaded=True)