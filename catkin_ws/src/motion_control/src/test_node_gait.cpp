#include "motion_control/motion_control.h"
#include "boost/thread.hpp"
#include "boost/interprocess/sync/interprocess_semaphore.hpp"
#include <ros/ros.h>
#include "motion_control/msg_gait.h"
#include "predefinition.h"


ros::Publisher pub_gait_msg;
int gait_cnt = 0;


void timer_callback(const ros::TimerEvent&)
{
	motion_control::msg_gait msg_gait;
	
	gait_cnt++;
	if(gait_cnt < 27)
		msg_gait.gait = "GaitL:B, GaitR:A, Gait:Gatiwalking";
	else if(gait_cnt < 60)
		msg_gait.gait = "GaitL:C, GaitR:A, Gait:Gatiwalking";
	else if(gait_cnt < 100)
		msg_gait.gait = "GaitL:A, GaitR:A, Gait:Gatiwalking";
	else
		gait_cnt = 0;
	pub_gait_msg.publish(msg_gait);

	ROS_INFO("pub gait msg");
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_node_gait");
    ros::NodeHandle nh;
	
	const double_t pub_freq = 100.0;
	
	pub_gait_msg = nh.advertise<motion_control::msg_gait>("msg_gait",1,true);
	ros::Timer timer1 = nh.createTimer(ros::Duration(1.0/pub_freq), timer_callback);
	
	
	ros::spin();
    return 0;	
}