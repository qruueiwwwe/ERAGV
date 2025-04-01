#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import serial
import yaml
import os
import time

class ElephantAGVNode:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('elephant_agv_node', anonymous=True)
        
        # 加载配置
        self.config = self.load_config()
        
        # 初始化串口
        self.serial_port = None
        self.init_serial()
        
        # 创建发布者和订阅者
        self.cmd_vel_sub = rospy.Subscriber(
            'cmd_vel',
            Twist,
            self.cmd_vel_callback,
            queue_size=10)
        
        self.status_pub = rospy.Publisher(
            'agv_status',
            String,
            queue_size=10)
            
        # 创建定时器用于状态发布
        rospy.Timer(rospy.Duration(1.0), self.publish_status)
        
        rospy.loginfo('Elephant AGV driver node has been initialized')

    def load_config(self):
        config_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(__file__))),
            'config',
            'elephant_agv_config.yaml'
        )
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)

    def init_serial(self):
        try:
            self.serial_port = serial.Serial(
                port=self.config['serial_port'],
                baudrate=self.config['baud_rate'],
                timeout=self.config['timeout']
            )
            rospy.loginfo(f'Successfully connected to {self.config["serial_port"]}')
        except Exception as e:
            rospy.logerr(f'Failed to connect to serial port: {str(e)}')
            raise

    def cmd_vel_callback(self, msg):
        # 将ROS的Twist消息转换为AGV控制命令
        linear_x = max(min(msg.linear.x, self.config['max_linear_velocity']), 
                      -self.config['max_linear_velocity'])
        angular_z = max(min(msg.angular.z, self.config['max_angular_velocity']), 
                       -self.config['max_angular_velocity'])
        
        # 构建控制命令
        cmd = self.build_control_command(linear_x, angular_z)
        
        # 发送命令
        try:
            self.serial_port.write(cmd.encode())
        except Exception as e:
            rospy.logerr(f'Failed to send command: {str(e)}')

    def build_control_command(self, linear_x, angular_z):
        # 这里需要根据实际的AGV协议来实现命令构建
        # 示例格式：$VEL,linear_x,angular_z#
        return f'$VEL,{linear_x:.2f},{angular_z:.2f}#\n'

    def publish_status(self, event):
        try:
            # 读取AGV状态
            if self.serial_port.in_waiting:
                status = self.serial_port.readline().decode().strip()
                msg = String()
                msg.data = status
                self.status_pub.publish(msg)
        except Exception as e:
            rospy.logerr(f'Failed to read status: {str(e)}')

    def __del__(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()

def main():
    try:
        node = ElephantAGVNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main() 