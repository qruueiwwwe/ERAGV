import os
import json
import asyncio
import threading
import rclpy
from rclpy.node import Node
from fastapi import FastAPI, Request, Response, BackgroundTasks
from fastapi.responses import StreamingResponse, JSONResponse, PlainTextResponse
from fastapi.middleware.cors import CORSMiddleware
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from nav_msgs.srv import SetMap
from starlette.status import HTTP_400_BAD_REQUEST
from typing import Optional
import cv2
import numpy as np

# ----------------- ENVIRONMENT VARIABLE CONFIGURATION --------------------

ROS_MASTER_URI = os.environ.get("ROS_MASTER_URI", "localhost")
ROS_DOMAIN_ID = int(os.environ.get("ROS_DOMAIN_ID", "0"))
DEVICE_NAMESPACE = os.environ.get("DEVICE_NAMESPACE", "")
HTTP_HOST = os.environ.get("HTTP_HOST", "0.0.0.0")
HTTP_PORT = int(os.environ.get("HTTP_PORT", "8080"))
RGB_TOPIC = os.environ.get("RGB_TOPIC", "/camera/color/image_raw")
DEPTH_TOPIC = os.environ.get("DEPTH_TOPIC", "/camera/depth/image_raw")
SPEED_TOPIC = os.environ.get("SPEED_TOPIC", "/cmd_vel")
STOP_SERVICE = os.environ.get("STOP_SERVICE", "/emergency_stop")
PARAM_SERVICE = os.environ.get("PARAM_SERVICE", "/set_param")
MODE_TOPIC = os.environ.get("MODE_TOPIC", "/robot_mode")
MAP_SERVICE = os.environ.get("MAP_SERVICE", "/map_manager")
# Add further topic/service envs as needed for extensibility

# ----------------- ROS2 NODE MANAGEMENT --------------------

class YahboomROSNode(Node):
    def __init__(self):
        super().__init__('yahboom_http_bridge')
        self.rgb_image_msg = None
        self.depth_image_msg = None
        self.rgb_lock = threading.Lock()
        self.depth_lock = threading.Lock()
        self.rgb_event = threading.Event()
        self.depth_event = threading.Event()

        self.rgb_sub = self.create_subscription(
            Image, RGB_TOPIC, self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, DEPTH_TOPIC, self.depth_callback, 10)
        self.speed_pub = self.create_publisher(
            Twist, SPEED_TOPIC, 10)
        self.mode_pub = self.create_publisher(
            String, MODE_TOPIC, 10)

    def rgb_callback(self, msg):
        with self.rgb_lock:
            self.rgb_image_msg = msg
            self.rgb_event.set()

    def depth_callback(self, msg):
        with self.depth_lock:
            self.depth_image_msg = msg
            self.depth_event.set()

    def get_rgb_image(self, timeout=2):
        got = self.rgb_event.wait(timeout)
        if not got:
            return None
        with self.rgb_lock:
            img = self.rgb_image_msg
            self.rgb_event.clear()
            return img

    def get_depth_image(self, timeout=2):
        got = self.depth_event.wait(timeout)
        if not got:
            return None
        with self.depth_lock:
            img = self.depth_image_msg
            self.depth_event.clear()
            return img

    def publish_speed(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.speed_pub.publish(msg)

    def publish_mode(self, mode: str):
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)

# ----------------- FastAPI HTTP SERVER --------------------

app = FastAPI(title="Yahboom Mobile Robot HTTP Driver")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

ros_context = {
    'node': None,
    'loop': None,
    'executor_thread': None
}

def ros_spin(node):
    rclpy.spin(node)

def ros_init():
    rclpy.init(args=None)
    node = YahboomROSNode()
    thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    thread.start()
    ros_context['node'] = node
    ros_context['executor_thread'] = thread

@app.on_event("startup")
def startup_event():
    ros_init()

@app.on_event("shutdown")
def shutdown_event():
    node = ros_context.get('node')
    if node is not None:
        node.destroy_node()
    rclpy.shutdown()

def rosimg_to_jpeg(msg: Image) -> Optional[bytes]:
    if msg.encoding not in ("rgb8", "bgr8"):
        # Only support rgb8 or bgr8 for browser streaming
        return None
    dtype = np.uint8
    img = np.ndarray(shape=(msg.height, msg.width, 3), dtype=dtype, buffer=msg.data)
    if msg.encoding == "rgb8":
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    _, jpeg = cv2.imencode('.jpg', img)
    return jpeg.tobytes()

# ----------------- MJPEG STREAMING ENDPOINTS --------------------

def mjpeg_streamer(image_getter_func, boundary="frame", interval=0.07):
    def gen():
        while True:
            imgmsg = image_getter_func()
            if imgmsg is None:
                continue
            jpeg_data = rosimg_to_jpeg(imgmsg)
            if jpeg_data is None:
                continue
            yield (b"--" + boundary.encode() + b"\r\n"
                   b"Content-Type: image/jpeg\r\n"
                   b"Content-Length: " + str(len(jpeg_data)).encode() + b"\r\n\r\n" +
                   jpeg_data + b"\r\n")
            # ~14 FPS
            asyncio.sleep(interval)
    return gen

@app.get("/stream/rgb")
def stream_rgb():
    node = ros_context['node']
    def img_gen():
        while True:
            imgmsg = node.get_rgb_image(timeout=5)
            if imgmsg is None:
                continue
            jpeg_data = rosimg_to_jpeg(imgmsg)
            if jpeg_data is None:
                continue
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n"
                   b"Content-Length: " + str(len(jpeg_data)).encode() + b"\r\n\r\n" +
                   jpeg_data + b"\r\n")
    return StreamingResponse(img_gen(), media_type="multipart/x-mixed-replace; boundary=frame")

@app.get("/stream/depth")
def stream_depth():
    node = ros_context['node']
    def img_gen():
        while True:
            imgmsg = node.get_depth_image(timeout=5)
            if imgmsg is None:
                continue
            # For depth, visualize as gray
            dtype = np.uint16 if imgmsg.encoding == "16UC1" else np.uint8
            img = np.ndarray(shape=(imgmsg.height, imgmsg.width, 1), dtype=dtype, buffer=imgmsg.data)
            normed = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX)
            normed = normed.astype(np.uint8)
            color = cv2.applyColorMap(normed, cv2.COLORMAP_JET)
            _, jpeg = cv2.imencode('.jpg', color)
            jpeg_data = jpeg.tobytes()
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n"
                   b"Content-Length: " + str(len(jpeg_data)).encode() + b"\r\n\r\n" +
                   jpeg_data + b"\r\n")
    return StreamingResponse(img_gen(), media_type="multipart/x-mixed-replace; boundary=frame")

# ----------------- API ENDPOINTS --------------------

@app.post("/param")
async def set_param(request: Request):
    """
    Adjust system parameters like HSV or PID values.
    """
    node = ros_context['node']
    try:
        data = await request.json()
    except Exception:
        return JSONResponse({"error": "Invalid JSON"}, status_code=HTTP_400_BAD_REQUEST)
    # Call a ROS service or publish to a topic to set parameters
    # Here, we assume a generic service call
    cli = node.create_client(Empty, PARAM_SERVICE)
    if not cli.wait_for_service(timeout_sec=2.0):
        return JSONResponse({"error": "Param service unavailable"}, status_code=503)
    req = Empty.Request()
    fut = cli.call_async(req)
    while rclpy.ok() and not fut.done():
        await asyncio.sleep(0.1)
    if fut.result() is not None:
        return {"success": True}
    return JSONResponse({"error": "Failed to set param"}, status_code=500)

@app.post("/stop")
async def stop_robot():
    """
    Emergency stop.
    """
    node = ros_context['node']
    cli = node.create_client(Empty, STOP_SERVICE)
    if not cli.wait_for_service(timeout_sec=2.0):
        return JSONResponse({"error": "Stop service unavailable"}, status_code=503)
    req = Empty.Request()
    fut = cli.call_async(req)
    while rclpy.ok() and not fut.done():
        await asyncio.sleep(0.1)
    if fut.result() is not None:
        return {"success": True}
    return JSONResponse({"error": "Failed to stop"}, status_code=500)

@app.post("/speed")
async def set_speed(request: Request):
    """
    Send a speed command to the robot.
    """
    node = ros_context['node']
    try:
        data = await request.json()
        lin = float(data.get("linear", 0.0))
        ang = float(data.get("angular", 0.0))
    except Exception:
        return JSONResponse({"error": "Invalid input"}, status_code=HTTP_400_BAD_REQUEST)
    node.publish_speed(lin, ang)
    return {"success": True}

@app.post("/mode")
async def set_mode(request: Request):
    """
    Switch robot's operating mode.
    """
    node = ros_context['node']
    try:
        data = await request.json()
        mode = str(data.get("mode", "manual"))
    except Exception:
        return JSONResponse({"error": "Invalid input"}, status_code=HTTP_400_BAD_REQUEST)
    node.publish_mode(mode)
    return {"success": True}

@app.post("/map")
async def map_manager(request: Request):
    """
    Map-related actions (save, nav, etc)
    """
    node = ros_context['node']
    try:
        data = await request.json()
        action = data.get("action", "")
        options = data.get("options", {})
    except Exception:
        return JSONResponse({"error": "Invalid input"}, status_code=HTTP_400_BAD_REQUEST)
    # This is highly device/ROS-specific; here, we just simulate a ROS service call
    cli = node.create_client(Empty, MAP_SERVICE)
    if not cli.wait_for_service(timeout_sec=2.0):
        return JSONResponse({"error": "Map service unavailable"}, status_code=503)
    req = Empty.Request()
    fut = cli.call_async(req)
    while rclpy.ok() and not fut.done():
        await asyncio.sleep(0.1)
    if fut.result() is not None:
        return {"success": True}
    return JSONResponse({"error": "Failed to process map action"}, status_code=500)

# ----------------- MAIN ENTRYPOINT --------------------

if __name__ == "__main__":
    import uvicorn
    uvicorn.run("main:app", host=HTTP_HOST, port=HTTP_PORT, reload=False)
