//serial_motor.cpp  
//创建于 2018年5月3日
//更新于 2018年5月3日
#include "motion_control/motion_control.h"
#include "boost/thread.hpp"
#include <ros/ros.h>
#include "motion_control/msg_serial_force.h"

int MotorPort;

void force_callback(const motion_control::msg_serial_force& force_input)
{
	std::string serial_force_state;
	int force;
	
	serial_force_state = force_input.serial_force_state;
	force = force_input.force;
	
	ROS_INFO("I heard: [%d]\n", force);  
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_motor");
    ros::NodeHandle nh;
	
	
	ros::Subscriber sub_force = nh.subscribe("msg_serial_force", 50, force_callback);

	ros::spin();
    return 0;
	
	
}