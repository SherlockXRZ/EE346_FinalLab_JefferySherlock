#!/usr/bin/python
# -*- coding: UTF-8 -*-
# 上面两行不可省略，第一行是：告诉操作系统执行这个脚本的时候，调用 /usr/bin 下的 python 解释器。第二行是：定义编码格式 "UTF-8-" 支持中文

from actionlib.action_client import GoalManager
import rospy 
import actionlib
from initialpose.msg import MoveBaseAction, MoveBaseGoal

def send_initial_pose_python():
    client = actionlib.SimpleActionClient('initialpose',MoveBaseAction)
    client.wait_for_server()
    #定义四个发送目标点的对象
    goal0 = MoveBaseGoal()
   
    # 初始化四个目标点在 map 坐标系下的坐标,数据来源于《采集的目标点.docx》
    goal0.pose.pose.position.x =         0.579705297947
    goal0.pose.pose.position.y =         0.0596310272813
    goal0.pose.pose.orientation.z =   0.967677018403
    goal0.pose.pose.orientation.w =  0.25219275971

    
    goal0.pose.header.frame_id = "map"
    goal0.pose.header.stamp = rospy.Time.now()
    client.send_goal( goal0)
    str_log = "Send Initial pose"
    rospy.loginfo(str_log)
    

    wait = client.wait_for_result(rospy.Duration.from_sec(2.0))  # 发送完毕目标点之后，根据action 的机制，等待反馈执行的状态，等待时长是：30 s.
    if not wait:
        str_log="initializing"
        rospy.loginfo(str_log)
    else:
        str_log="initialized"
        rospy.loginfo(str_log)

    return "Mission Finished."

if __name__ == '__main__':
        rospy.init_node('send_initial_pose_python',anonymous=True)    # python 语言方式下的　初始化 ROS 节点，
        result = send_initial_pose_python()
        rospy.loginfo(result)