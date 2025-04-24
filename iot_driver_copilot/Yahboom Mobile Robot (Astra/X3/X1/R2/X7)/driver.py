import os
import json
import asyncio
import threading
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from fastapi import FastAPI, Request, Response, BackgroundTasks
from fastapi.responses import StreamingResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Empty
from nav_msgs.msg import OccupancyGrid
from starlette.status import HTTP_400_BAD_REQUEST
from starlette.responses import PlainTextResponse
from cv_bridge import CvBridge
import uvicorn

# Environment Variables
DEVICE_ROS_MASTER_URI = os.environ.get('DEVICE_ROS_MASTER_URI', 'http://localhost:11311')
DEVICE_ROS_DOMAIN_ID = os.environ.get('DEVICE_ROS_DOMAIN_ID', '0')
DEVICE_RGB_TOPIC = os.environ.get('DEVICE_RGB_TOPIC', '/camera/color/image_raw')
DEVICE_DEPTH_TOPIC = os.environ.get('DEVICE_DEPTH_TOPIC', '/camera/depth/image_raw')
DEVICE_SPEED_TOPIC = os.environ.get('DEVICE_SPEED_TOPIC', '/cmd_vel')
DEVICE_MODE_TOPIC = os.environ.get('DEVICE_MODE_TOPIC', '/mode_switch')
DEVICE_PARAM_TOPIC = os.environ.get('DEVICE_PARAM_TOPIC', '/param_tuning')
DEVICE_STOP_TOPIC = os.environ.get('DEVICE_STOP_TOPIC', '/emergency_stop')
DEVICE_MAP_SERVICE = os.environ.get('DEVICE_MAP_SERVICE', '/map_service')
SERVER_HOST = os.environ.get('SERVER_HOST', '0.0.0.0')
SERVER_PORT = int(os.environ.get('SERVER_PORT', 8000))
STREAM_FPS = int(os.environ.get('STREAM_FPS', 10))

os.environ['ROS_DOMAIN_ID'] = DEVICE_ROS_DOMAIN_ID

app = FastAPI(title="Yahboom Mobile Robot HTTP Driver")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

bridge = CvBridge()

class ROS2Node(Node):
    def __init__(self):
        super().__init__('yahboom_http_driver')
        self.latest_rgb = None
        self.latest_depth = None
        self.rgb_sub = self.create_subscription(
            Image, DEVICE_RGB_TOPIC, self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, DEVICE_DEPTH_TOPIC, self.depth_callback, 10)
        self.speed_pub = self.create_publisher(
            Twist, DEVICE_SPEED_TOPIC, 10)
        self.mode_pub = self.create_publisher(
            String, DEVICE_MODE_TOPIC, 10)
        self.param_pub = self.create_publisher(
            String, DEVICE_PARAM_TOPIC, 10)
        self.stop_pub = self.create_publisher(
            String, DEVICE_STOP_TOPIC, 10)
        self.map_cli = self.create_client(Empty, DEVICE_MAP_SERVICE)

    def rgb_callback(self, msg):
        try:
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.latest_rgb = cv_img
        except Exception:
            self.latest_rgb = None

    def depth_callback(self, msg):
        try:
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            norm_img = cv2.normalize(cv_img, None, 0, 255, cv2.NORM_MINMAX)
            norm_img = np.uint8(norm_img)
            depth_color = cv2.applyColorMap(norm_img, cv2.COLORMAP_JET)
            self.latest_depth = depth_color
        except Exception:
            self.latest_depth = None

    def publish_speed(self, linear, angular):
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self.speed_pub.publish(twist)

    def publish_mode(self, mode):
        msg = String()
        msg.data = str(mode)
        self.mode_pub.publish(msg)

    def publish_param(self, params):
        msg = String()
        msg.data = json.dumps(params)
        self.param_pub.publish(msg)

    def publish_stop(self):
        msg = String()
        msg.data = "stop"
        self.stop_pub.publish(msg)

    async def call_map_service(self, action, options):
        if not self.map_cli.wait_for_service(timeout_sec=2.0):
            return False, 'Map service unavailable'
        req = Empty.Request()
        future = self.map_cli.call_async(req)
        await asyncio.wrap_future(future)
        if future.done():
            return True, 'OK'
        else:
            return False, 'Service call failed'

def ros_spin(node):
    rclpy.spin(node)

# ROS2 thread start
def start_ros2():
    global ros2_node
    rclpy.init()
    ros2_node = ROS2Node()
    t = threading.Thread(target=ros_spin, args=(ros2_node,), daemon=True)
    t.start()
    return ros2_node

ros2_node = start_ros2()

# MJPEG Stream generator for RGB
def mjpeg_rgb_stream():
    while True:
        frame = ros2_node.latest_rgb
        if frame is not None:
            ret, jpeg = cv2.imencode('.jpg', frame)
            if ret:
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' +
                       jpeg.tobytes() + b'\r\n')
        else:
            # Empty frame placeholder
            img = np.zeros((240, 320, 3), dtype=np.uint8)
            ret, jpeg = cv2.imencode('.jpg', img)
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' +
                   jpeg.tobytes() + b'\r\n')
        asyncio.sleep(1.0/STREAM_FPS)

# MJPEG Stream generator for Depth
def mjpeg_depth_stream():
    while True:
        frame = ros2_node.latest_depth
        if frame is not None:
            ret, jpeg = cv2.imencode('.jpg', frame)
            if ret:
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' +
                       jpeg.tobytes() + b'\r\n')
        else:
            # Empty frame placeholder
            img = np.zeros((240, 320, 3), dtype=np.uint8)
            ret, jpeg = cv2.imencode('.jpg', img)
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' +
                   jpeg.tobytes() + b'\r\n')
        asyncio.sleep(1.0/STREAM_FPS)

@app.get("/stream/rgb")
async def stream_rgb():
    return StreamingResponse(mjpeg_rgb_stream(), media_type='multipart/x-mixed-replace; boundary=frame')

@app.get("/stream/depth")
async def stream_depth():
    return StreamingResponse(mjpeg_depth_stream(), media_type='multipart/x-mixed-replace; boundary=frame')

@app.post("/speed")
async def set_speed(request: Request):
    try:
        data = await request.json()
        linear = data['linear']
        angular = data['angular']
        ros2_node.publish_speed(linear, angular)
        return JSONResponse({'result': 'ok'})
    except Exception as e:
        return JSONResponse({'error': str(e)}, status_code=HTTP_400_BAD_REQUEST)

@app.post("/mode")
async def set_mode(request: Request):
    try:
        data = await request.json()
        mode = data['mode']
        ros2_node.publish_mode(mode)
        return JSONResponse({'result': 'ok'})
    except Exception as e:
        return JSONResponse({'error': str(e)}, status_code=HTTP_400_BAD_REQUEST)

@app.post("/param")
async def set_param(request: Request):
    try:
        data = await request.json()
        ros2_node.publish_param(data)
        return JSONResponse({'result': 'ok'})
    except Exception as e:
        return JSONResponse({'error': str(e)}, status_code=HTTP_400_BAD_REQUEST)

@app.post("/stop")
async def stop_robot():
    try:
        ros2_node.publish_stop()
        return JSONResponse({'result': 'ok'})
    except Exception as e:
        return JSONResponse({'error': str(e)}, status_code=HTTP_400_BAD_REQUEST)

@app.post("/map")
async def map_action(request: Request):
    try:
        data = await request.json()
        action = data.get('action')
        options = data.get('options', {})
        # Only support 'save' and 'nav' for demo
        result, msg = await ros2_node.call_map_service(action, options)
        if result:
            return JSONResponse({'result': 'ok'})
        else:
            return JSONResponse({'error': msg}, status_code=HTTP_400_BAD_REQUEST)
    except Exception as e:
        return JSONResponse({'error': str(e)}, status_code=HTTP_400_BAD_REQUEST)

if __name__ == "__main__":
    uvicorn.run("main:app", host=SERVER_HOST, port=SERVER_PORT, reload=False)