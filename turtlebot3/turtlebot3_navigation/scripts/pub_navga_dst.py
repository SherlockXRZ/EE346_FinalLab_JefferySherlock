#!/usr/bin/python
# -*- coding: UTF-8 -*-
# 上面两行不可省略，第一行是：告诉操作系统执行这个脚本的时候，调用 /usr/bin 下的 python 解释器。第二行是：定义编码格式 "UTF-8-" 支持中文

from actionlib.action_client import GoalManager
import rospy 
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def send_goals_python():
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    #定义四个发送目标点的对象
    goal00 = MoveBaseGoal()
    goal0 = MoveBaseGoal()
    goal1 = MoveBaseGoal() 
    goal2 = MoveBaseGoal() 
    goal3 = MoveBaseGoal()
    goal4 = MoveBaseGoal()
    goal5 = MoveBaseGoal()
    # 初始化四个目标点在 map 坐标系下的坐标,数据来源于《采集的目标点.docx》
    goal00.target_pose.pose.position.x =     0.77
    goal00.target_pose.pose.position.y =     -0.95
    goal00.target_pose.pose.orientation.z =  0.707106796641
    goal00.target_pose.pose.orientation.w =  -0.707106796641

    # p2
    goal0.target_pose.pose.position.x =     -0.0287735648453
    goal0.target_pose.pose.position.y =     -4.08753183365
    goal0.target_pose.pose.orientation.z =  0.707106796641
    goal0.target_pose.pose.orientation.w =  -0.707106796641
    
    # p3
    goal1.target_pose.pose.position.x = 3.95840157999
    goal1.target_pose.pose.position.y = -4.03311231613
    goal1.target_pose.pose.orientation.z = 0.707106796641
    goal1.target_pose.pose.orientation.w = 0.707106765732
    
    # p4
    goal2.target_pose.pose.position.x = 3.8586430645
    goal2.target_pose.pose.position.y = -0.0125219726562
    goal2.target_pose.pose.orientation.z =  0.707106796641
    goal2.target_pose.pose.orientation.w = - 0.707106796641
    
    goal3.target_pose.pose.position.x = 3.24684820175
    goal3.target_pose.pose.position.y =  -2.10940843582
    goal3.target_pose.pose.orientation.z = 1
    goal3.target_pose.pose.orientation.w = 0

    goal4.target_pose.pose.position.x = 0.753088831902
    goal4.target_pose.pose.position.y =  -1.91926503181
    goal4.target_pose.pose.orientation.z = 0.707106796641
    goal4.target_pose.pose.orientation.w = 0.707106796641
     
    # p1
    goal5.target_pose.pose.position.x = 0
    goal5.target_pose.pose.position.y =  0
    goal5.target_pose.pose.orientation.z = 0
    goal5.target_pose.pose.orientation.w = 1


    # goal_lists=[goal00, goal0, goal1, goal2, goal3, goal4, goal5]       # 采用 python 中的列表方式，替代实现C/C++ 中的数组概念 
    goal_lists=[goal0, goal1, goal2, goal5]
    total_goal = len(goal_lists)
    print(total_goal)
    goal_number = total_goal     # total is 6 goals
    while(goal_number):      
        if(total_goal - goal_number ==0):
            goal_lists[total_goal-goal_number].target_pose.header.frame_id = "map"
            goal_lists[total_goal-goal_number].target_pose.header.stamp = rospy.Time.now()
            client.send_goal( goal_lists[total_goal-goal_number])
            str_log = "Send NO. %s Goal !!!" %str(total_goal-goal_number)
            rospy.loginfo(str_log)
        elif(total_goal - goal_number ==1):
            goal_lists[total_goal-goal_number].target_pose.header.frame_id = "map"
            goal_lists[total_goal-goal_number].target_pose.header.stamp = rospy.Time.now()
            client.send_goal( goal_lists[total_goal-goal_number])
            str_log = "Send NO. %s Goal !!!" %str(total_goal-goal_number)
            rospy.loginfo(str_log)
        elif(total_goal - goal_number ==2):
            goal_lists[total_goal-goal_number].target_pose.header.frame_id = "map"
            goal_lists[total_goal-goal_number].target_pose.header.stamp = rospy.Time.now()
            client.send_goal( goal_lists[total_goal-goal_number])
            str_log = "Send NO. %s Goal !!!" %str(total_goal-goal_number)
            rospy.loginfo(str_log)
        elif(total_goal - goal_number ==3):
            goal_lists[total_goal-goal_number].target_pose.header.frame_id = "map"
            goal_lists[total_goal-goal_number].target_pose.header.stamp = rospy.Time.now()
            client.send_goal( goal_lists[total_goal-goal_number])
            str_log = "Send NO. %s Goal !!!" %str(total_goal-goal_number)
        elif(total_goal - goal_number ==4):
            goal_lists[total_goal-goal_number].target_pose.header.frame_id = "map"
            goal_lists[total_goal-goal_number].target_pose.header.stamp = rospy.Time.now()
            client.send_goal( goal_lists[total_goal-goal_number])
            str_log = "Send NO. %s Goal !!!" %str(total_goal-goal_number)
        elif(total_goal - goal_number ==5):
            goal_lists[total_goal-goal_number].target_pose.header.frame_id = "map"
            goal_lists[total_goal-goal_number].target_pose.header.stamp = rospy.Time.now()
            client.send_goal( goal_lists[total_goal-goal_number])
            str_log = "Send NO. %s Goal !!!" %str(total_goal-goal_number)
        elif(total_goal - goal_number ==6):
            goal_lists[total_goal-goal_number].target_pose.header.frame_id = "map"
            goal_lists[total_goal-goal_number].target_pose.header.stamp = rospy.Time.now()
            client.send_goal( goal_lists[total_goal-goal_number])
            str_log = "Send NO. %s Goal !!!" %str(total_goal-goal_number)
            rospy.loginfo(str_log)

        wait = client.wait_for_result(rospy.Duration.from_sec(60.0))  # 发送完毕目标点之后，根据action 的机制，等待反馈执行的状态，等待时长是：60 s.
        if not wait:
            str_log="The NO. %s Goal Planning Failed for some reasons" %str(6-goal_number)
            rospy.loginfo(str_log)
            goal_number = goal_number - 1  # 等待超时之后 准备发送下一个目标点，action server 会自动执行最新的请求.
            continue
        else:
            str_log="The NO. %s Goal achieved success !!!" %str(6-goal_number)
            rospy.loginfo(str_log)
            goal_number = goal_number - 1
    return "Mission Finished."

if __name__ == '__main__':
        rospy.init_node('send_goals_python',anonymous=True)    # python 语言方式下的　初始化 ROS 节点，
        result = send_goals_python()
        rospy.loginfo(result)
