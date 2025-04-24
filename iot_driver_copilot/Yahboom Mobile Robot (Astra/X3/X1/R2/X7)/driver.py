import os
import json
import asyncio
import threading
from functools import partial
from typing import Optional

from fastapi import FastAPI, Request, Response, BackgroundTasks, status
from fastapi.responses import StreamingResponse, JSONResponse, PlainTextResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import uvicorn

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Empty
from std_msgs.msg import Float32MultiArray

# --- Environment Configuration ---
DEVICE_IP = os.environ.get('DEVICE_IP', 'localhost')
ROS_MASTER_URI = os.environ.get('ROS_MASTER_URI', f'http://{DEVICE_IP}:11311')
ROS_DOMAIN_ID = os.environ.get('ROS_DOMAIN_ID', '0')
SERVER_HOST = os.environ.get('SERVER_HOST', '0.0.0.0')
SERVER_PORT = int(os.environ.get('SERVER_PORT', '8000'))
IMAGE_TOPIC = os.environ.get('IMAGE_TOPIC', '/camera/color/image_raw')
DEPTH_TOPIC = os.environ.get('DEPTH_TOPIC', '/camera/depth/image_raw')
CMD_VEL_TOPIC = os.environ.get('CMD_VEL_TOPIC', '/cmd_vel')
STOP_TOPIC = os.environ.get('STOP_TOPIC', '/stop_signal')
PARAM_TOPIC = os.environ.get('PARAM_TOPIC', '/param_tune')
MODE_TOPIC = os.environ.get('MODE_TOPIC', '/mode_switch')
MAP_SERVICE = os.environ.get('MAP_SERVICE', '/map_service')
MAP_TOPIC = os.environ.get('MAP_TOPIC', '/map_cmd')

# --- FastAPI App ---
app = FastAPI(
    title="Yahboom Mobile Robot HTTP ROS Driver",
    description="HTTP API for Yahboom Mobile Robot (Astra/X3/X1/R2/X7) with ROS backend.",
    version="1.0.0"
)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"]
)

# --- ROS2 Node Management ---

class ROSNode(Node):
    def __init__(self):
        super().__init__("yahboom_http_bridge")
        self.image_msg: Optional[Image] = None
        self.depth_msg: Optional[Image] = None
        self._lock = threading.Lock()
        self.image_sub = self.create_subscription(
            Image, IMAGE_TOPIC, self.image_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, DEPTH_TOPIC, self.depth_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.stop_pub = self.create_publisher(String, STOP_TOPIC, 10)
        self.param_pub = self.create_publisher(Float32MultiArray, PARAM_TOPIC, 10)
        self.mode_pub = self.create_publisher(String, MODE_TOPIC, 10)
        # For map management: use topic or service, depending on implementation
        self.map_pub = self.create_publisher(String, MAP_TOPIC, 10)
        self.map_cli = self.create_client(Empty, MAP_SERVICE)

    def image_callback(self, msg):
        with self._lock:
            self.image_msg = msg

    def depth_callback(self, msg):
        with self._lock:
            self.depth_msg = msg

    def get_image(self, kind='rgb'):
        with self._lock:
            if kind == 'rgb':
                return self.image_msg
            elif kind == 'depth':
                return self.depth_msg
            else:
                return None

# --- ROS2 Initialization and Threading ---

rclpy.init(args=None)
ros_node = ROSNode()
ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
ros_thread.start()

# --- Models ---

class SpeedCommand(BaseModel):
    linear: float
    angular: float

class ParamTuning(BaseModel):
    params: dict

class ModeSwitch(BaseModel):
    mode: str

class MapAction(BaseModel):
    action: str
    options: Optional[dict] = None

# --- API Endpoints ---

@app.post("/param")
async def param_tuning(request: Request):
    data = await request.json()
    params = data.get("params", {})
    if not isinstance(params, dict):
        return JSONResponse({"error": "params must be a dict."}, status_code=400)
    arr = Float32MultiArray()
    arr.data = []
    for v in params.values():
        try:
            arr.data.append(float(v))
        except:
            arr.data.append(0.0)
    ros_node.param_pub.publish(arr)
    return JSONResponse({"status": "ok", "sent_params": params})

@app.post("/stop")
async def stop_robot():
    ros_node.stop_pub.publish(String(data="emergency_stop"))
    return JSONResponse({"status": "stopped"})

@app.post("/map")
async def map_action(action: MapAction, background_tasks: BackgroundTasks):
    if action.action.lower() == "save":
        def call_service():
            while not ros_node.map_cli.wait_for_service(timeout_sec=1.0):
                pass
            req = Empty.Request()
            future = ros_node.map_cli.call_async(req)
            rclpy.spin_until_future_complete(ros_node, future)
        background_tasks.add_task(call_service)
        return JSONResponse({"status": "map_save_requested"})
    elif action.action.lower() == "nav":
        nav_cmd = String()
        nav_cmd.data = json.dumps(action.options) if action.options else "{}"
        ros_node.map_pub.publish(nav_cmd)
        return JSONResponse({"status": "map_nav_command_sent"})
    else:
        return JSONResponse({"error": "Invalid action. Use 'save' or 'nav'."}, status_code=400)

@app.post("/speed")
async def set_speed(cmd: SpeedCommand):
    twist = Twist()
    twist.linear.x = float(cmd.linear)
    twist.angular.z = float(cmd.angular)
    ros_node.cmd_vel_pub.publish(twist)
    return JSONResponse({"status": "speed_set", "linear": cmd.linear, "angular": cmd.angular})

@app.post("/mode")
async def set_mode(mode: ModeSwitch):
    ros_node.mode_pub.publish(String(data=mode.mode))
    return JSONResponse({"status": "mode_set", "mode": mode.mode})

# --- MJPEG Streaming (RGB/DEPTH) ---

def image_to_jpeg_bytes(msg: Image) -> Optional[bytes]:
    try:
        # For sensor_msgs/Image in RGB8 or MONO8, the data is raw. For browser streaming, we need JPEG.
        # We'll use OpenCV for conversion, but to avoid 3rd party command execution, do it in-Python.
        import numpy as np
        import cv2
        if msg.encoding.lower() in ["rgb8", "bgr8"]:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            if msg.encoding.lower() == "rgb8":
                arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            _, jpeg = cv2.imencode('.jpg', arr)
            return jpeg.tobytes()
        elif msg.encoding.lower() in ["mono8"]:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
            _, jpeg = cv2.imencode('.jpg', arr)
            return jpeg.tobytes()
        elif msg.encoding.lower() in ["16uc1", "mono16"]:
            arr = np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, msg.width))
            # Normalize and convert to 8-bit for JPEG
            arr8 = np.clip(arr / (arr.max() / 255.0 + 1e-5), 0, 255).astype(np.uint8)
            _, jpeg = cv2.imencode('.jpg', arr8)
            return jpeg.tobytes()
        else:
            return None
    except Exception:
        return None

async def mjpeg_stream(kind='rgb'):
    boundary = "frame"
    while True:
        msg = ros_node.get_image(kind=kind)
        if msg is not None:
            jpeg = image_to_jpeg_bytes(msg)
            if jpeg is not None:
                yield (
                    b"--%s\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Content-Length: %d\r\n\r\n" % (boundary.encode(), len(jpeg))
                )
                yield jpeg
                yield b"\r\n"
        await asyncio.sleep(0.07)  # ~14fps

@app.get("/stream/rgb")
async def rgb_stream():
    return StreamingResponse(
        mjpeg_stream('rgb'),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )

@app.get("/stream/depth")
async def depth_stream():
    return StreamingResponse(
        mjpeg_stream('depth'),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )

@app.get("/")
def root():
    return PlainTextResponse(
        "Yahboom Mobile Robot HTTP ROS Driver\n\n"
        "API Endpoints:\n"
        "POST /param - System parameter tuning\n"
        "POST /stop - Emergency stop\n"
        "POST /map - Map save/nav\n"
        "POST /speed - Set speed\n"
        "POST /mode - Mode switch\n"
        "GET /stream/rgb - MJPEG RGB stream\n"
        "GET /stream/depth - MJPEG Depth stream"
    )

# --- Main Server Entry Point ---

if __name__ == "__main__":
    uvicorn.run(app, host=SERVER_HOST, port=SERVER_PORT)