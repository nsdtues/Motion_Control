//system_pot.cpp  
//创建于 2018年5月4日
//更新于 2018年5月6日

#include "motion_control/motion_control.h"
#include "boost/thread.hpp"
#include <ros/ros.h>
#include "motion_control/msg_serial_pot.h"
#include "motion_control/sensor_position_msg.h"
#include "predefinition.h"

ros::Publisher pub_msg_pot;

sem_t sem_sub;
sem_t sem_pub;

uint16_t pot;

void position_callback(const motion_control::sensor_position_msg& pot_input)
{
	std::string serial_pot_state;	
	pot = pot_input.position;
	sem_post(&sem_sub);	
}


void pot_pub_loop(void){
	
	float pot_x,pot_y,pot_mid;
	float pot_t;
	
	motion_control::msg_serial_pot msg_serial_pot;
	
	pot_x = POT_VALUE_LONG;
    pot_y = POT_VALUE_SHORT;

    if(pot_x > pot_y){
        pot_mid = (pot_x + pot_y +3.3)/2;
    }else{
        pot_mid = (pot_x + pot_y)/2;
    }
	
	
	sem_wait(&sem_sub);
	
	msg_serial_pot.serial_pot_state = SENSER_OK;
	pot_t = (float)pot/1000;
	
    if(pot_t < pot_mid){
        pot_t = pot_t + 3.3;
    }
    pot_t = pot_t - pot_mid;	

	msg_serial_pot.pot = pot_t;	
	pub_msg_pot.publish(msg_serial_pot);
	
	for(;;){
		
		sem_wait(&sem_sub);
		pot_t = (float)pot/1000;
		
		if(pot_t < pot_mid){
			pot_t = pot_t + 3.3;
		}
		pot_t = pot_t - pot_mid;	
		
        if((pot_t < PROTECTION_POT_VALUE_L)&&(pot_t > PROTECTION_POT_VALUE_H)){
			msg_serial_pot.serial_pot_state = SENSER_OUT_RANGE;
        }		
		msg_serial_pot.pot = pot_t;
		pub_msg_pot.publish(msg_serial_pot);		
	}	
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_pot");
    ros::NodeHandle nh;
	
	pub_msg_pot = nh.advertise<motion_control::msg_serial_pot>("msg_serial_pot",50,true);
	ros::Subscriber sub_position = nh.subscribe("sersor_position_msg", 50, position_callback);
	boost::thread pot_pub(&pot_pub_loop);
	
	
	ros::spin();
    return 0;	
}