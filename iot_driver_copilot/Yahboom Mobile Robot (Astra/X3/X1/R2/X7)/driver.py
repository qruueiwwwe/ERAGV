import os
import io
import json
import threading
from fastapi import FastAPI, Request, Response, BackgroundTasks
from fastapi.responses import StreamingResponse, JSONResponse
from pydantic import BaseModel
from typing import Optional
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import Bool

import cv2
import numpy as np

from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import CompressedImage

import asyncio

# --- Environment Variable Configuration ---
ROS_DOMAIN_ID = int(os.environ.get("ROS_DOMAIN_ID", "0"))
ROS_IP = os.environ.get("ROS_IP", "127.0.0.1")
ROS_MASTER_URI = os.environ.get("ROS_MASTER_URI", "http://127.0.0.1:11311")
CAMERA_RGB_TOPIC = os.environ.get("CAMERA_RGB_TOPIC", "/camera/color/image_raw")
CAMERA_DEPTH_TOPIC = os.environ.get("CAMERA_DEPTH_TOPIC", "/camera/depth/image_raw")
SPEED_TOPIC = os.environ.get("SPEED_TOPIC", "/cmd_vel")
STOP_SERVICE = os.environ.get("STOP_SERVICE", "/emergency_stop")
PARAM_TOPIC = os.environ.get("PARAM_TOPIC", "/set_param")
MODE_TOPIC = os.environ.get("MODE_TOPIC", "/set_mode")
MAP_ACTION_TOPIC = os.environ.get("MAP_ACTION_TOPIC", "/map_action")
HTTP_SERVER_HOST = os.environ.get("HTTP_SERVER_HOST", "0.0.0.0")
HTTP_SERVER_PORT = int(os.environ.get("HTTP_SERVER_PORT", "8000"))

# --- ROS Node Definition ---
class RobotBridgeNode(Node):
    def __init__(self):
        super().__init__('yahboom_bridge_driver')
        self.cv_bridge = None
        try:
            from cv_bridge import CvBridge
            self.cv_bridge = CvBridge()
        except ImportError:
            self.get_logger().warn("cv_bridge not installed. Image streaming won't work.")

        # Publisher cache
        self.speed_pub = self.create_publisher(Twist, SPEED_TOPIC, 10)
        self.param_pub = self.create_publisher(String, PARAM_TOPIC, 10)
        self.mode_pub = self.create_publisher(String, MODE_TOPIC, 10)
        self.map_action_pub = self.create_publisher(String, MAP_ACTION_TOPIC, 10)

        # Service client cache
        self.stop_cli = self.create_client(Empty, STOP_SERVICE)

        # For Image streaming
        self.rgb_sub = None
        self.depth_sub = None
        self.rgb_frame = None
        self.depth_frame = None
        self.rgb_lock = threading.Lock()
        self.depth_lock = threading.Lock()

    def start_rgb_stream(self, topic=CAMERA_RGB_TOPIC):
        if self.rgb_sub is None:
            self.rgb_sub = self.create_subscription(
                Image, topic, self.rgb_callback, 10)

    def start_depth_stream(self, topic=CAMERA_DEPTH_TOPIC):
        if self.depth_sub is None:
            self.depth_sub = self.create_subscription(
                Image, topic, self.depth_callback, 10)

    def rgb_callback(self, msg):
        if self.cv_bridge:
            frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            ret, jpeg = cv2.imencode('.jpg', frame)
            if ret:
                with self.rgb_lock:
                    self.rgb_frame = jpeg.tobytes()

    def depth_callback(self, msg):
        if self.cv_bridge:
            depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            norm_depth = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
            norm_depth = np.uint8(norm_depth)
            depth_colored = cv2.applyColorMap(norm_depth, cv2.COLORMAP_JET)
            ret, jpeg = cv2.imencode('.jpg', depth_colored)
            if ret:
                with self.depth_lock:
                    self.depth_frame = jpeg.tobytes()

    def get_latest_rgb(self):
        with self.rgb_lock:
            return self.rgb_frame

    def get_latest_depth(self):
        with self.depth_lock:
            return self.depth_frame

    async def send_speed(self, linear: float, angular: float):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.speed_pub.publish(twist)

    async def send_param(self, param_json: dict):
        msg = String()
        msg.data = json.dumps(param_json)
        self.param_pub.publish(msg)

    async def send_mode(self, mode: str):
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)

    async def map_action(self, action: dict):
        msg = String()
        msg.data = json.dumps(action)
        self.map_action_pub.publish(msg)

    async def stop_robot(self):
        if not self.stop_cli.wait_for_service(timeout_sec=2.0):
            return False
        req = Empty.Request()
        future = self.stop_cli.call_async(req)
        await asyncio.wrap_future(future)
        return True

# --- FastAPI App Definition ---
app = FastAPI()
node: Optional[RobotBridgeNode] = None

def ros_spin():
    rclpy.spin(node)

@app.on_event("startup")
def startup_event():
    global node, ros_thread
    rclpy.init(args=None)
    node = RobotBridgeNode()
    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()

@app.on_event("shutdown")
def shutdown_event():
    global node
    if node:
        node.destroy_node()
        rclpy.shutdown()

# --- POST /speed ---
class SpeedCommand(BaseModel):
    linear: float
    angular: float

@app.post("/speed")
async def set_speed(cmd: SpeedCommand):
    await node.send_speed(cmd.linear, cmd.angular)
    return {"status": "ok"}

# --- POST /stop ---
@app.post("/stop")
async def stop_robot():
    result = await node.stop_robot()
    if result:
        return {"status": "stopped"}
    else:
        return JSONResponse({"status": "error", "reason": "Stop service unavailable"}, status_code=500)

# --- POST /param ---
@app.post("/param")
async def set_param(request: Request):
    data = await request.json()
    await node.send_param(data)
    return {"status": "ok"}

# --- POST /mode ---
class ModeCommand(BaseModel):
    mode: str

@app.post("/mode")
async def set_mode(cmd: ModeCommand):
    await node.send_mode(cmd.mode)
    return {"status": "ok"}

# --- POST /map ---
@app.post("/map")
async def map_action(request: Request):
    data = await request.json()
    await node.map_action(data)
    return {"status": "ok"}

# --- Video Streaming Endpoints ---
def mjpeg_stream_rgb():
    node.start_rgb_stream()
    while True:
        frame = node.get_latest_rgb()
        if frame is not None:
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
        else:
            # If no frame available, yield a small wait
            import time
            time.sleep(0.05)

def mjpeg_stream_depth():
    node.start_depth_stream()
    while True:
        frame = node.get_latest_depth()
        if frame is not None:
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
        else:
            import time
            time.sleep(0.05)

@app.get("/video/rgb")
def video_rgb():
    return StreamingResponse(mjpeg_stream_rgb(), media_type="multipart/x-mixed-replace; boundary=frame")

@app.get("/video/depth")
def video_depth():
    return StreamingResponse(mjpeg_stream_depth(), media_type="multipart/x-mixed-replace; boundary=frame")

# --- Main Entrypoint ---
if __name__ == "__main__":
    import uvicorn
    uvicorn.run("main:app", host=HTTP_SERVER_HOST, port=HTTP_SERVER_PORT, reload=False)