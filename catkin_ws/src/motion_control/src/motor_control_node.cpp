#include "motion_control/motion_control.h"
#include "boost/thread.hpp"
#include "boost/interprocess/sync/interprocess_semaphore.hpp"
#include <ros/ros.h>
#include "motion_control/msg_serial_force.h"
#include "motion_control/msg_serial_pot.h"
#include "motion_control/msg_motion_cmd.h"
#include "motion_control/msg_motion_evt.h"
#include "motion_control/msg_gait.h"
#include "predefinition.h"
#include <std_msgs/Int16.h>

ros::Publisher pub_msg_motion_evt;

bool force_ready = false;
bool pot_ready = false;
u_int16_t motor_start_type = 0xFFFF;

void load_default_settings(void)
{

}

void force_callback(const motion_control::msg_serial_force& force_input)
{
    //force_ready = check_force_data(force_input)
}

void pot_callback(const motion_control::msg_serial_pot& pot_input)
{
    //pot_ready = check_pot_data(pot_input);
}

void motion_start_cmd_callback(const std_msgs::Int16& motion_start_cmd_input)
{
    motor_start_type = motion_start_cmd_input.data;
}

void motion_cmd_callback(const std_msgs::Int16& motion_cmd_input)
{
    if(force_ready && pot_ready){
        u_int16_t cmd_type = motion_cmd_input.data;
        //motion_control::msg_motion_evt motor_status = 0xFFFF;
        switch (cmd_type) {
        case CTL_CMDINITIAL:
            motor_init();
            break;
        case CTL_CMDPOWERDOWN:
            motor_powerdown();
            break;
        case CTL_CMDMOTIONSLEEP:
            motor_sleep();
            break;
        case CTL_CMDMOTIONSTOP:
            motor_stop();
            break;
        case CTL_CMDMOTIONSTART:
            //TODO if_process();
            if(motor_start_type != 0xFFFF) {
                motor_start(motor_start_type);
            }else{
            //TODO
            }
            break;
        default:
            ROS_INFO("motro cmd is invalid.");
            break;
        }
    }
    else{
        ROS_DEBUG("force or pot are not ready.");
    }
}

void gait_callback(const motion_control::msg_gait& gait_input)
{

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_motor_node");
    ros::NodeHandle nh;

    pub_msg_motion_evt = nh.advertise<motion_control::msg_motion_evt>("msg_motion_evt",50,true);
    ros::Subscriber sub_force = nh.subscribe("msg_serial_force", 50, force_callback);
    ros::Subscriber sub_pot = nh.subscribe("msg_serial_pot", 50, pot_callback);
    //ros::Subscriber sub_motion_cmd = nh.subscribe("node_motor_msg_to_sys", 50, motion_cmd_callback);
    ros::Subscriber sub_motion_cmd = nh.subscribe("motor_cmd", 50, motion_cmd_callback);
    ros::Subscriber sub_motion_start_cmd = nh.subscribe("motor_start_cmd", 50, motion_start_cmd_callback);
    //ros::Subscriber sub_gait = nh.subscribe("msg_gait", 50, gait_callback);

    ros::spin();
    return 0;
}
