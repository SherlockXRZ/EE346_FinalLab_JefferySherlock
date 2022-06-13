#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rosbag
import rospy
import numpy as np
import subprocess
import math
import time
from std_msgs.msg import UInt8, Float64
import cv2.aruco as aruco
from geometry_msgs.msg import Twist
from actionlib.action_client import GoalManager
import rospy 
import actionlib

def bag_publisher():
    #client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    #client.wait_for_server()
    # 读取roabag
    bag_file_path = '/home/robot/rosData/turn_right_fast.bag'
    bag_data = rosbag.Bag(bag_file_path,'r')
    bag_info = bag_data.get_type_and_topic_info()
    cmd_vel = bag_data.read_messages('/cmd_vel')
    print(bag_info)

    # 初始化发布信息
    bag_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    #rate = rospy.Rate(10) # 10Hz

    # 调用命令行读取rosbag
    bag_process = subprocess.call(['rosbag', 'play', '/home/robot/rosData/turn_right_normal.bag'])

    # 发布速度消息
    #for topic, msg, t in cmd_vel:
    #    print('time: ', t)
    #    print(msg)
    #    bag_pub.publish(msg) # 发布
    #    rate.sleep() # 休眠 

    return 'Rosbag publishing is over!'


if __name__ == '__main__':
    rospy.init_node('BagPublisher',anonymous=True)  
    result = bag_publisher()
    rospy.loginfo(result)

