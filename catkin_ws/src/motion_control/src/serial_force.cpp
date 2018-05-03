//serial_force.cpp  
//创建于 2018年5月3日
//更新于 2018年5月3日
#include "motion_control/motion_control.h"
#include "boost/thread.hpp"
#include <ros/ros.h>
#include "motion_control/msg_serial_force.h"

int FrocePort;
ros::Publisher pub_msg_force;

void serial_read_loop(void)
{
    // FrocePort = tty_init(FORCE_PORT_NUM);
	// driver_init(FrocePort,FORCE_PORT_NUM);
	
	motion_control::msg_serial_force msg_serial_force;
	
	msg_serial_force.serial_force_state = "hello world";
	msg_serial_force.force = 2000;
	
	pub_msg_force.publish(msg_serial_force);
	
	return;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "serial_force");
	ros::NodeHandle nh;
	
	pub_msg_force = nh.advertise<motion_control::msg_serial_force>("msg_serial_force",50,true);
	
	boost::thread serial_read(&serial_read_loop);
	
	ros::spin();
    return 0;
}