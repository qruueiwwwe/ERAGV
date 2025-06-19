import os
import threading
import json
from flask import Flask, jsonify, request
import rospy
from turtlesim.msg import Pose

# Configuration from environment variables
ROS_MASTER_URI = os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
ROS_HOSTNAME = os.environ.get("ROS_HOSTNAME", "localhost")
HTTP_SERVER_HOST = os.environ.get("HTTP_SERVER_HOST", "0.0.0.0")
HTTP_SERVER_PORT = int(os.environ.get("HTTP_SERVER_PORT", 8080))

# Set ROS environment (must be set before rospy.init_node)
os.environ["ROS_MASTER_URI"] = ROS_MASTER_URI
os.environ["ROS_HOSTNAME"] = ROS_HOSTNAME

app = Flask(__name__)

class TurtlePoseSubscriber:
    def __init__(self):
        self.lock = threading.Lock()
        self.latest_pose = None
        self.subscriber = None
        self.is_shutdown = False
        self.start_ros_node()
        self.subscribe_pose()

    def start_ros_node(self):
        # Avoid multiple initializations
        if not rospy.core.is_initialized():
            rospy.init_node('turtle_pose_subscriber_http', anonymous=True, disable_signals=True)

    def pose_callback(self, msg):
        with self.lock:
            self.latest_pose = {
                "x": msg.x,
                "y": msg.y,
                "theta": msg.theta,
                "linear_velocity": msg.linear_velocity,
                "angular_velocity": msg.angular_velocity
            }

    def subscribe_pose(self):
        with self.lock:
            if self.subscriber is not None:
                self.subscriber.unregister()
            self.subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback, queue_size=1)

    def get_latest_pose(self):
        with self.lock:
            return self.latest_pose.copy() if self.latest_pose else None

    def shutdown(self):
        with self.lock:
            self.is_shutdown = True
            if self.subscriber is not None:
                self.subscriber.unregister()
                self.subscriber = None

pose_sub = TurtlePoseSubscriber()

@app.route('/commands/sub', methods=['POST'])
def resubscribe():
    pose_sub.subscribe_pose()
    return jsonify({
        "status": "success",
        "message": "Resubscribed to /turtle1/pose"
    }), 200

@app.route('/pose', methods=['GET'])
def get_pose():
    pose = pose_sub.get_latest_pose()
    if pose is None:
        return jsonify({"error": "No pose data received yet"}), 404
    return jsonify(pose), 200

def ros_spin_thread():
    # Run rospy.spin() in a separate thread to process callbacks
    while not rospy.core.is_shutdown() and not pose_sub.is_shutdown:
        rospy.rostime.wallsleep(0.1)

if __name__ == '__main__':
    # Start ROS spin thread for callbacks
    t = threading.Thread(target=ros_spin_thread)
    t.daemon = True
    t.start()
    app.run(host=HTTP_SERVER_HOST, port=HTTP_SERVER_PORT)