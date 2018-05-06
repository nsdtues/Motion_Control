//system_node.cpp  
//创建于 2018年5月6日
//更新于 2018年5月6日

#include "motion_control/motion_control.h"
#include "boost/thread.hpp"
#include <ros/ros.h>
#include "motion_control/msg_motion_cmd.h"
#include "motion_control/sys_cmd_msg_to_motor.msg"
#include "predefinition.h"

sem_t sem_sub;
sem_t sem_pub;

ros::Publisher pub_msg_motion_cmd;



void sys_cmd_msg_to_motor_callback(const motion_control::sys_cmd_msg_to_motor& cmd_input)
{
	
	
	
	
	
	
	
	
}


void motion_cmd_pub_loop(void)
{
	
	
	
	
	
	
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "system_node");
    ros::NodeHandle nh;
	
	pub_msg_motion_cmd = nh.advertise<motion_control::msg_motion_cmd>("msg_motion_cmd",50,true);
	ros::Subscriber sub_sys_cmd_msg_to_motor = nh.subscribe("sys_cmd_msg_to_motor", 50, sys_cmd_msg_to_motor_callback);
	boost::thread motion_cmd_pub(&motion_cmd_pub_loop);
	
	
	ros::spin();
    return 0;	
}