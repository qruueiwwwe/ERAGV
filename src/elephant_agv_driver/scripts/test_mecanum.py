#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import time

def test_mecanum_movement():
    # 初始化ROS节点
    rospy.init_node('mecanum_test', anonymous=True)
    
    # 创建速度发布者
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    
    # 等待连接建立
    rospy.sleep(1.0)
    
    try:
        # 测试前进
        rospy.loginfo("测试前进...")
        cmd = Twist()
        cmd.linear.x = 0.5
        cmd_vel_pub.publish(cmd)
        rospy.sleep(2.0)
        
        # 测试后退
        rospy.loginfo("测试后退...")
        cmd = Twist()
        cmd.linear.x = -0.5
        cmd_vel_pub.publish(cmd)
        rospy.sleep(2.0)
        
        # 测试左移
        rospy.loginfo("测试左移...")
        cmd = Twist()
        cmd.linear.y = 0.5
        cmd_vel_pub.publish(cmd)
        rospy.sleep(2.0)
        
        # 测试右移
        rospy.loginfo("测试右移...")
        cmd = Twist()
        cmd.linear.y = -0.5
        cmd_vel_pub.publish(cmd)
        rospy.sleep(2.0)
        
        # 测试原地旋转
        rospy.loginfo("测试原地旋转...")
        cmd = Twist()
        cmd.angular.z = 0.5
        cmd_vel_pub.publish(cmd)
        rospy.sleep(2.0)
        
        # 测试对角线运动
        rospy.loginfo("测试对角线运动...")
        cmd = Twist()
        cmd.linear.x = 0.3
        cmd.linear.y = 0.3
        cmd_vel_pub.publish(cmd)
        rospy.sleep(2.0)
        
        # 测试圆弧运动
        rospy.loginfo("测试圆弧运动...")
        cmd = Twist()
        cmd.linear.x = 0.3
        cmd.angular.z = 0.2
        cmd_vel_pub.publish(cmd)
        rospy.sleep(2.0)
        
        # 停止
        rospy.loginfo("测试完成，停止...")
        cmd = Twist()
        cmd_vel_pub.publish(cmd)
        
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    test_mecanum_movement() 