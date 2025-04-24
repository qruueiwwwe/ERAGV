import os
import json
import threading
import time
from flask import Flask, request, Response, jsonify, stream_with_context
import cv2
import numpy as np

try:
    import rospy
    from sensor_msgs.msg import Image
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    from std_srvs.srv import Empty
    from cv_bridge import CvBridge
except ImportError:
    rospy = None  # For non-ROS environments/testing

# Read configuration from environment variables
ROS_MASTER_URI = os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
ROS_IP = os.environ.get("ROS_IP", "127.0.0.1")
HTTP_HOST = os.environ.get("HTTP_HOST", "0.0.0.0")
HTTP_PORT = int(os.environ.get("HTTP_PORT", "8000"))
RGB_TOPIC = os.environ.get("RGB_TOPIC", "/camera/color/image_raw")
DEPTH_TOPIC = os.environ.get("DEPTH_TOPIC", "/camera/depth/image_raw")
CMD_VEL_TOPIC = os.environ.get("CMD_VEL_TOPIC", "/cmd_vel")
MODE_TOPIC = os.environ.get("MODE_TOPIC", "/mode_cmd")
PARAM_TOPIC = os.environ.get("PARAM_TOPIC", "/set_param")
STOP_SERVICE = os.environ.get("STOP_SERVICE", "/emergency_stop")
MAP_SERVICE = os.environ.get("MAP_SERVICE", "/map_manager")
BRIDGE_TYPE = os.environ.get("CV_BRIDGE_TYPE", "bgr8")  # or "rgb8" if needed

app = Flask(__name__)
bridge = CvBridge() if rospy else None

# Shared frame buffers
rgb_frame = {'data': None, 'lock': threading.Lock()}
depth_frame = {'data': None, 'lock': threading.Lock()}

def ros_init():
    if rospy and not rospy.core.is_initialized():
        import sys
        rospy.init_node("yahboom_driver_node", anonymous=True, disable_signals=True)

def rgb_image_callback(msg):
    try:
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding=BRIDGE_TYPE)
        ret, jpeg = cv2.imencode('.jpg', cv_img)
        if ret:
            with rgb_frame['lock']:
                rgb_frame['data'] = jpeg.tobytes()
    except Exception:
        pass

def depth_image_callback(msg):
    try:
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        norm_img = cv2.normalize(cv_img, None, 0, 255, cv2.NORM_MINMAX)
        norm_img = np.uint8(norm_img)
        color_img = cv2.applyColorMap(norm_img, cv2.COLORMAP_JET)
        ret, jpeg = cv2.imencode('.jpg', color_img)
        if ret:
            with depth_frame['lock']:
                depth_frame['data'] = jpeg.tobytes()
    except Exception:
        pass

def ros_spin_thread():
    while not rospy.is_shutdown():
        rospy.spin()
        time.sleep(0.01)

def start_ros():
    ros_init()
    # Subscribers for video
    rospy.Subscriber(RGB_TOPIC, Image, rgb_image_callback, queue_size=1)
    rospy.Subscriber(DEPTH_TOPIC, Image, depth_image_callback, queue_size=1)
    threading.Thread(target=ros_spin_thread, daemon=True).start()

@app.route('/video/rgb')
def video_rgb():
    def gen():
        while True:
            with rgb_frame['lock']:
                frame = rgb_frame['data']
            if frame is not None:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            time.sleep(0.04)  # ~25 FPS
    return Response(stream_with_context(gen()), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/video/depth')
def video_depth():
    def gen():
        while True:
            with depth_frame['lock']:
                frame = depth_frame['data']
            if frame is not None:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            time.sleep(0.08)  # ~12 FPS
    return Response(stream_with_context(gen()), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/param', methods=['POST'])
def set_param():
    data = request.get_json(force=True)
    if not data:
        return jsonify({'error': 'No JSON data provided'}), 400
    if rospy:
        pub = rospy.Publisher(PARAM_TOPIC, String, queue_size=1)
        msg = String()
        msg.data = json.dumps(data)
        pub.publish(msg)
        return jsonify({'status': 'Parameter update sent', 'data': data})
    else:
        return jsonify({'error': 'ROS is not available'}), 503

@app.route('/stop', methods=['POST'])
def stop():
    if rospy:
        try:
            rospy.wait_for_service(STOP_SERVICE, timeout=2)
            stop_srv = rospy.ServiceProxy(STOP_SERVICE, Empty)
            stop_srv()
            return jsonify({'status': 'Emergency stop command sent'})
        except Exception:
            return jsonify({'error': 'Failed to call stop service'}), 500
    else:
        return jsonify({'error': 'ROS is not available'}), 503

@app.route('/map', methods=['POST'])
def map_manager():
    data = request.get_json(force=True)
    if not data or 'action' not in data:
        return jsonify({'error': 'Missing action for map operation'}), 400
    if rospy:
        try:
            rospy.wait_for_service(MAP_SERVICE, timeout=2)
            map_srv = rospy.ServiceProxy(MAP_SERVICE, String)
            resp = map_srv(String(json.dumps(data)))
            return jsonify({'status': 'Map operation sent', 'response': resp.data})
        except Exception:
            return jsonify({'error': 'Failed to call map service'}), 500
    else:
        return jsonify({'error': 'ROS is not available'}), 503

@app.route('/speed', methods=['POST'])
def set_speed():
    data = request.get_json(force=True)
    if not data or 'linear' not in data or 'angular' not in data:
        return jsonify({'error': 'Must provide both linear and angular speeds'}), 400
    if rospy:
        pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=1)
        twist = Twist()
        twist.linear.x = float(data.get('linear', 0))
        twist.angular.z = float(data.get('angular', 0))
        pub.publish(twist)
        return jsonify({'status': 'Speed command sent', 'linear': twist.linear.x, 'angular': twist.angular.z})
    else:
        return jsonify({'error': 'ROS is not available'}), 503

@app.route('/mode', methods=['POST'])
def set_mode():
    data = request.get_json(force=True)
    if not data or 'mode' not in data:
        return jsonify({'error': 'No mode specified'}), 400
    if rospy:
        pub = rospy.Publisher(MODE_TOPIC, String, queue_size=1)
        msg = String()
        msg.data = data['mode']
        pub.publish(msg)
        return jsonify({'status': 'Mode command sent', 'mode': data['mode']})
    else:
        return jsonify({'error': 'ROS is not available'}), 503

@app.route('/')
def index():
    return '''
    <h1>Yahboom Mobile Robot HTTP Driver</h1>
    <ul>
        <li>RGB Video: <a href="/video/rgb">/video/rgb</a></li>
        <li>Depth Video: <a href="/video/depth">/video/depth</a></li>
        <li>POST /param (JSON)</li>
        <li>POST /stop</li>
        <li>POST /map (JSON)</li>
        <li>POST /speed (JSON)</li>
        <li>POST /mode (JSON)</li>
    </ul>
    '''

if __name__ == '__main__':
    if rospy:
        start_ros()
    app.run(host=HTTP_HOST, port=HTTP_PORT, threaded=True)
