
/**
 *该例程将订阅/turtle1/pose话题，消息类型为turtlesim::Pose
 */

#include <ros/ros.h>
#include "turtlesim/Pose.h"

void poseCallback(const turtlesim::Pose::ConstPtr& msg){
	//打印接受到的消息
	ROS_INFO("Turtle pose: x:%0.6f, y:%0.6f", msg->x, msg->y);
	
}

int main(int argc, char **argv){
	//初始化ROS节点
	ros::init(argc, argv, "pose_subscriber");

	//创建节点句柄
	ros::NodeHandle n;

	//创建一个subscriber,订阅名为/turtle1/pose的topic,注册回调函数poseCallback
	ros::Subscriber pose_sub = n.subscribe("/turtle1/pose", 10, poseCallback);

	//循环等待
	ros::spin();

	return 0;
}

