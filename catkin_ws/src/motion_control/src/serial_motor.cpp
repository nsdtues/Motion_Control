//serial_motor.cpp  
//创建于 2018年5月3日
//更新于 2018年5月3日
#include "motion_control/motion_control.h"
#include "boost/thread.hpp"
#include "boost/interprocess/sync/interprocess_semaphore.hpp"
#include <ros/ros.h>
#include "motion_control/msg_serial_force.h"
#include "motion_control/msg_serial_pot.h"
#include "motion_control/msg_motion_cmd.h"
#include "motion_control/msg_gait.h"
#include "predefinition.h"

boost::interprocess::interprocess_semaphore sys_cmd_semaphore(0);
boost::interprocess::interprocess_semaphore pot_semaphore(0);
boost::interprocess::interprocess_semaphore force_semaphore(0);

int MotorPort;
uint32_t force;
float pot;
int32_t EnableFlag = MOTOR_EN_TRUE;
int32_t force_data_flag = 0,pot_data_flag = 0;
int32_t gait_state;

struct motion_cmd_t{
	uint32_t state;
	uint32_t mode;
	uint32_t foot;
	uint32_t forceaid;
	uint32_t max_force;
	int32_t max_position;
	int32_t zero_position;
	int32_t preload_position;
	int32_t max_velocity;
	int32_t nset_acc;
	float max_pot;
	float pid_kp;
	float pid_ki;
	int32_t pid_umax;
	int32_t pid_umin;
}motion_cmd_para;


void force_callback(const motion_control::msg_serial_force& force_input)
{
	force = force_input.force;
	if(force_input.serial_force_state == SENSER_OUT_RANGE){
		ROS_INFO("position out of range: [%f]\n", pot);
		EnableFlag = MOTOR_EN_FALSE;
	}
	
	if(pot_data_flag == 0){
		force_semaphore.post();
		force_data_flag = 1;
	}	
	ROS_INFO("force: [%d]\n", force);  
}

void pot_callback(const motion_control::msg_serial_pot& pot_input)
{
	pot = pot_input.pot;
	if(pot_input.serial_pot_state == SENSER_OUT_RANGE){
		ROS_INFO("position out of range: [%f]\n", pot);
		EnableFlag = MOTOR_EN_FALSE;
	}
	
	if(pot_data_flag == 0){
		pot_semaphore.post();
		pot_data_flag = 1;
	}	
	ROS_INFO("pot: [%f]\n", pot);  
}

void motion_cmd_callback(const motion_control::msg_motion_cmd& motion_cmd_input)
{
	sys_cmd_semaphore.post();
	motion_cmd_para.state = motion_cmd_input.state;
	motion_cmd_para.mode = motion_cmd_input.mode;
	motion_cmd_para.foot = motion_cmd_input.foot;
	motion_cmd_para.forceaid = motion_cmd_input.forceaid;
	motion_cmd_para.max_force = motion_cmd_input.max_force;
	motion_cmd_para.max_position = motion_cmd_input.max_position;
	motion_cmd_para.zero_position = motion_cmd_input.zero_position;
	motion_cmd_para.preload_position = motion_cmd_input.preload_position;
	motion_cmd_para.max_velocity = motion_cmd_input.max_velocity;
	motion_cmd_para.max_pot = motion_cmd_input.max_pot;
	motion_cmd_para.pid_kp = motion_cmd_input.pid_kp;
	motion_cmd_para.pid_ki = motion_cmd_input.pid_ki;
	motion_cmd_para.pid_umax = motion_cmd_input.pid_umax;
	motion_cmd_para.pid_umin = motion_cmd_input.pid_umin;	
	
}

void gait_callback(const motion_control::msg_gait& gait_input)
{
	if(gait_input.gait.find("Gait:GaitStopping") != std::string::npos){
		gait_state = 3;
	}else{
		if(motion_cmd_para.foot == 0){
			if(gait_input.gait.find("GaitL:A") != std::string::npos){
				gait_state = 1;
			}else if(gait_input.gait.find("GaitL:B") != std::string::npos){
				gait_state = 2;
			}else if(gait_input.gait.find("GaitL:C") != std::string::npos){
				gait_state = 3;
			}
		}else if(motion_cmd_para.foot == 1){
			if(gait_input.gait.find("GaitR:A") != std::string::npos){
				gait_state = 1;
			}else if(gait_input.gait.find("GaitR:B") != std::string::npos){
				gait_state = 2;
			}else if(gait_input.gait.find("GaitR:B") != std::string::npos){
				gait_state = 3;
			}			
		}else{
			gait_state = 3;
		}
	}		
}


void motor_ctrl_loop(void)
{
	struct timeval tv;
    struct timespec ts;
	
	uint32_t time_now,time_mark;
	
	gettimeofday(&tv,NULL);
	time_mark = (uint32_t)(tv.tv_sec*1000+tv.tv_usec/1000);		//获取系统时间，单位为ms
	
	sys_cmd_semaphore.wait();
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
	ros::Subscriber sub_pot = nh.subscribe("msg_serial_pot", 50, pot_callback);
	ros::Subscriber sub_motion_cmd = nh.subscribe("node_motor_msg_to_sys", 50, motion_cmd_callback);
	ros::Subscriber sub_gait = nh.subscribe("msg_gait", 50, gait_callback);
	boost::thread motor_ctrl(&motor_ctrl_loop);
	
	
	ros::spin();
    return 0;	
}