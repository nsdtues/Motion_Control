#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <boost/thread.hpp>
#include <unistd.h>

u_int8_t flag = 0;

void receive_loop_msg(void)
{
    while(1){
         ROS_ERROR("flag = %d.\n",flag);
    }

}

void motion_cmd_callback(const std_msgs::Int16& motion_cmd_input)
{
        int16_t cmd_type = motion_cmd_input.data;
        switch (cmd_type) {
        case 0:
            flag = 1;
            ROS_INFO("flag = %d.\n",flag);
            break;
        case 1:
            flag = 2;
            ROS_INFO("flag = %d.\n",flag);
            break;
        case 2:
            flag = 3;
            ROS_INFO("flag = %d.\n",flag);
            break;
        case 3:
            flag = 4;
            ROS_INFO("flag = %d.\n",flag);
            break;
        case 4:
            flag = 5;
            break;
        default:
            ROS_INFO("motro cmd is invalid.");
            break;
        }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "thread_test_node");
    ros::NodeHandle nh;

    ros::Subscriber sub_motion_cmd = nh.subscribe("thread_test", 1, motion_cmd_callback);
    boost::thread reveive_loop(&receive_loop_msg);

    ros::spin();
    return 0;
}
