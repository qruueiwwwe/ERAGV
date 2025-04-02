#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import serial
import yaml
import os
import time
import math

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
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.status_pub = rospy.Publisher('agv_status', String, queue_size=10)
        
        # 运动状态变量
        self.current_velocity = Twist()
        self.current_steer_angle = 0.0
        
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

    # ========== 麦克纳姆轮全向运动控制方法 ==========
    def move_forward(self, speed=0.5):
        """前进控制"""
        cmd = Twist()
        cmd.linear.x = min(speed, self.config['max_linear_velocity'])
        self.current_velocity = cmd
        self.send_command(cmd)

    def move_backward(self, speed=0.5):
        """后退控制"""
        cmd = Twist()
        cmd.linear.x = max(-speed, -self.config['max_linear_velocity'])
        self.current_velocity = cmd
        self.send_command(cmd)

    def move_left(self, speed=0.5):
        """左移控制"""
        cmd = Twist()
        cmd.linear.y = min(speed, self.config['max_linear_velocity'])
        self.current_velocity = cmd
        self.send_command(cmd)

    def move_right(self, speed=0.5):
        """右移控制"""
        cmd = Twist()
        cmd.linear.y = max(-speed, -self.config['max_linear_velocity'])
        self.current_velocity = cmd
        self.send_command(cmd)

    def rotate(self, angular_speed):
        """原地旋转控制"""
        cmd = Twist()
        cmd.angular.z = max(min(angular_speed, self.config['max_angular_velocity']), 
                          -self.config['max_angular_velocity'])
        self.current_velocity = cmd
        self.send_command(cmd)

    def move_diagonal(self, x_speed=0.5, y_speed=0.5):
        """对角线运动控制"""
        cmd = Twist()
        cmd.linear.x = max(min(x_speed, self.config['max_linear_velocity']), 
                          -self.config['max_linear_velocity'])
        cmd.linear.y = max(min(y_speed, self.config['max_linear_velocity']), 
                          -self.config['max_linear_velocity'])
        self.current_velocity = cmd
        self.send_command(cmd)

    def move_arc(self, linear_speed=0.5, angular_speed=0.5):
        """圆弧运动控制"""
        cmd = Twist()
        cmd.linear.x = max(min(linear_speed, self.config['max_linear_velocity']), 
                          -self.config['max_linear_velocity'])
        cmd.angular.z = max(min(angular_speed, self.config['max_angular_velocity']), 
                          -self.config['max_angular_velocity'])
        self.current_velocity = cmd
        self.send_command(cmd)
        
    def stop(self):
        """紧急停止"""
        cmd = Twist()
        self.current_velocity = cmd
        self.send_command(cmd)

    def send_command(self, cmd):
        """发送控制命令到AGV"""
        try:
            # 构建控制命令
            cmd_str = self.build_control_command(cmd.linear.x, cmd.linear.y, cmd.angular.z)
            # 发送命令
            self.serial_port.write(cmd_str.encode())
        except Exception as e:
            rospy.logerr(f'Failed to send command: {str(e)}')

    def build_control_command(self, linear_x, linear_y, angular_z):
        """构建AGV控制命令"""
        # 这里需要根据实际的AGV协议来实现命令构建
        # 示例格式：$VEL,linear_x,linear_y,angular_z#
        return f'$VEL,{linear_x:.2f},{linear_y:.2f},{angular_z:.2f}#\n'

    def publish_status(self, event):
        """发布AGV状态"""
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
        """清理资源"""
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