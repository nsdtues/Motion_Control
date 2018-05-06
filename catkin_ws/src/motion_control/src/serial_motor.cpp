//serial_motor.cpp  
//创建于 2018年5月3日
//更新于 2018年5月3日
#include "motion_control/motion_control.h"
#include "boost/thread.hpp"
#include <ros/ros.h>
#include "motion_control/msg_serial_force.h"

int MotorPort;
uint32_t force;
void force_callback(const motion_control::msg_serial_force& force_input)
{
	std::string serial_force_state;

	
	serial_force_state = force_input.serial_force_state;
	force = force_input.force;
	
	ROS_INFO("I heard: [%d]\n", force);  
}

void motor_ctrl_loop(void)
{
	struct timeval tv;
    struct timespec ts;
	
	uint32_t time_now,time_mark;
	
	gettimeofday(&tv,NULL);
	time_mark = (uint32_t)(tv.tv_sec*1000+tv.tv_usec/1000);		//获取系统时间，单位为ms
	
	
	for(;;){
		gettimeofday(&tv,NULL);
		time_now = (uint32_t)(tv.tv_sec*1000+tv.tv_usec/1000); 
		time_now = (time_now - time_mark)&0x000fffff;
		ROS_INFO("time=%u force=%d\n",time_now , force);  
		usleep(20000);
	}
	
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_motor");
    ros::NodeHandle nh;
	
	
	ros::Subscriber sub_force = nh.subscribe("msg_serial_force", 50, force_callback);
	boost::thread motor_ctrl(&motor_ctrl_loop);
	
	
	ros::spin();
    return 0;	
}