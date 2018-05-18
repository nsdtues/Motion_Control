//motion_module_run_info_node.cpp
#include "motion_control/motion_control.h"
#include <ros/ros.h>
#include "motion_control/sensor_run_info_msg.h"
#include "motion_control/motion_module_run_info_msg.h"
#include "predefinition.h"

ros::Publisher pub_motion_module_run_info;

motion_control::motion_module_run_info_msg motion_module_run_info_msg;

void sub_force_run_info_msg_callback(const motion_control::sensor_run_info_msg& force_run_info_input)
{
	motion_module_run_info_msg.force_state = force_run_info_input.state;
	motion_module_run_info_msg.force_error_log = force_run_info_input.error_log;
}


void sub_pot_run_info_msg_callback(const motion_control::sensor_run_info_msg& pot_run_info_input)
{
	motion_module_run_info_msg.pot_state = pot_run_info_input.state;
	motion_module_run_info_msg.pot_error_log = pot_run_info_input.error_log;
}


void sub_motor_run_info_msg_callback(const motion_control::sensor_run_info_msg& motor_run_info_input)
{
	motion_module_run_info_msg.motor_state = motor_run_info_input.state;
	motion_module_run_info_msg.motor_error_log = motor_run_info_input.error_log;
}

void timer_callback(const ros::TimerEvent&)
{
	ROS_INFO("force sensor state: [%s]", motion_module_run_info_msg.force_state.c_str());
	ROS_INFO("force sensor error log: [%u]", motion_module_run_info_msg.force_error_log);
	ROS_INFO("pot sensor state: [%s]", motion_module_run_info_msg.pot_state.c_str());
	ROS_INFO("pot sensor error log: [%u]", motion_module_run_info_msg.pot_error_log);
	ROS_INFO("motor sensor state: [%s]", motion_module_run_info_msg.motor_state.c_str());
	ROS_INFO("motor sensor error log: [%u]\n", motion_module_run_info_msg.motor_error_log);
	pub_motion_module_run_info.publish(motion_module_run_info_msg);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_module_run_info_node");
    ros::NodeHandle nh;
	
	const double_t pub_freq = 10.0;
	
	pub_motion_module_run_info = nh.advertise<motion_control::motion_module_run_info_msg>("motion_module_run_info_msg",1,true);
	
	ros::Subscriber sub_force_run_info_msg = nh.subscribe("force_run_info_msg", 1, sub_force_run_info_msg_callback);
	ros::Subscriber sub_pot_run_info_msg = nh.subscribe("pot_run_info_msg", 1, sub_pot_run_info_msg_callback);
	ros::Subscriber sub_motor_run_info_msg = nh.subscribe("motor_run_info_msg", 1, sub_motor_run_info_msg_callback);
	ros::Timer timer1 = nh.createTimer(ros::Duration(1.0/pub_freq), timer_callback);
	
	
	ros::spin();
    return 0;	
}