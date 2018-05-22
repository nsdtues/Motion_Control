#include "ros/ros.h"

/**
 * This tutorial demonstrates the use of timer callbacks.
 */

void callback1(const ros::TimerEvent&)
{
    ROS_INFO("start...............................................");
    const u_int32_t num = 90000;
    u_int32_t i;
    for(i = 0;i < num;i++){
        ROS_INFO("%ud\n",i*(i+1)/3*5/2);
    }
    ROS_INFO("end.");
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "timer_test");
  ros::NodeHandle n;

  ros::Timer timer1 = n.createTimer(ros::Duration(0.0001), callback1);

  ros::spin();

  return 0;
}
