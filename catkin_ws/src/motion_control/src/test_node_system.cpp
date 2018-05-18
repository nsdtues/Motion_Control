#include "motion_control/motion_control.h"
#include "boost/thread.hpp"
#include "boost/interprocess/sync/interprocess_semaphore.hpp"
#include <ros/ros.h>
#include "motion_control/sensor_position_msg.h"
#include "motion_control/node_motor_msg_to_sys.h"
#include "motion_control/sys_cmd_msg_to_motor.h"
#include "predefinition.h"

ros::Publisher pub_sensor_position_msg;
ros::Publisher pub_sys_cmd_msg_to_motor;

void node_motor_msg_to_sys_callback(const motion_control::node_motor_msg_to_sys& motor_msg_to_sys_input)
{

}



void timer_callback(const ros::TimerEvent&)
{
    motion_control::sensor_position_msg sensor_position_msg;
	motion_control::sys_cmd_msg_to_motor sys_cmd_msg_to_motor;
    //setting pub_serial_data by raspmsgsend
	sensor_position_msg.position = 1000;
    pub_sensor_position_msg.publish(sensor_position_msg);
	sys_cmd_msg_to_motor.syscmd = "cmdmotorinitial";
	pub_sys_cmd_msg_to_motor.publish(sys_cmd_msg_to_motor);

	ROS_INFO("pub sensor data");
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_node_system");
    ros::NodeHandle nh;
	
	const double_t pub_freq = 100.0;
	
	pub_sensor_position_msg = nh.advertise<motion_control::sensor_position_msg>("sensor_position_msg",1,true);
	pub_sys_cmd_msg_to_motor = nh.advertise<motion_control::sys_cmd_msg_to_motor>("sys_cmd_msg_to_motor",1,true);
	ros::Subscriber sub_node_motor_msg_to_sys = nh.subscribe("node_motor_msg_to_sys", 1, node_motor_msg_to_sys_callback);
	ros::Timer timer1 = nh.createTimer(ros::Duration(1.0/pub_freq), timer_callback);
	
	
	ros::spin();
    return 0;	
}