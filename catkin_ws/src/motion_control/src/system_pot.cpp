//system_pot.cpp  
//创建于 2018年5月4日
//更新于 2018年5月6日

#include "motion_control/motion_control.h"
#include "boost/thread.hpp"
#include "boost/interprocess/sync/interprocess_semaphore.hpp"
#include <ros/ros.h>
#include "motion_control/msg_serial_pot.h"
#include "motion_control/sensor_position_msg.h"
#include "motion_control/sensor_run_info_msg.h"
#include "predefinition.h"

ros::Publisher pub_msg_pot;
ros::Publisher pub_sensor_run_info_msg;

boost::interprocess::interprocess_semaphore sub_semaphore(0);

motion_control::sensor_run_info_msg sensor_run_info_msg;

#if(WHERE_MOTION == DESKTOP_VERSION)
int AdcPort = 0;			//电位计的端口号	

int get_pot(int port,float *msg)
{

    int32_t 		readcnt = 0,nread,ncheck=0,state = 0;
    uint8_t			data[256],datagram[64];
    float	  		adc_temp;
    int try_t = 256;
    while(try_t--){
        datagram[0] = 0;
        nread = read(port,datagram,1);

        if(nread<0){
            return -1;
        }

        if(nread == 0){
            return -1;
        }

        switch(state){
        case 0:
            if(datagram[0]==0x53){
                data[readcnt] = datagram[0];
                readcnt++;
                state=1;
            }
            break;
        case 1:
            if(readcnt < 3){
                data[readcnt] = datagram[0];
                readcnt++;
                break;
            }else{
                data[readcnt] = datagram[0];
                if(data[3] == data[1]^data[2]){
                    state=2;
                    readcnt++;
                    break;
                }else{
                    state = 0;
                    readcnt = 0;
                    sensor_run_info_msg.error_log++;
					pub_sensor_run_info_msg.publish(sensor_run_info_msg);

                    break;
                }
            }
        case 2:
            if(datagram[0]==0x59){
                data[readcnt] = datagram[0];
                adc_temp = ((float)((data[2]<<8)|data[1]))/1000;
                state = 0;
                readcnt= 0;
                *msg = adc_temp;
                return 0;
                break;
            }
            else{
                state = 0;
                readcnt= 0;
                sensor_run_info_msg.error_log++;
				pub_sensor_run_info_msg.publish(sensor_run_info_msg);
                break;
            }
        default:
            break;
        }
    }
    return -1;
}


void pot_pub_loop(void){
	
	motion_control::msg_serial_pot msg_serial_pot;
    int i,ret;
    float pot_t,pot_temp[12];

    // AdcPort = tty_init(ADC_PORT_NUM);

    if(AdcPort<0){
		msg_serial_pot.serial_pot_state = SENSER_NO_PORT;
		sensor_run_info_msg.state = "can not open /dev/ttyUSBpot";
        sensor_run_info_msg.error_log++;
		pub_sensor_run_info_msg.publish(sensor_run_info_msg);		
    }

    // driver_init(AdcPort,ADC_PORT_NUM);

    ret = get_pot(AdcPort,&pot_temp[0]);

    if(ret == -1){
		msg_serial_pot.serial_pot_state = SENSER_NO_DATA;
		sensor_run_info_msg.state = "pot sensor no data";
		sensor_run_info_msg.error_log++;
		pub_sensor_run_info_msg.publish(sensor_run_info_msg);		
    }else if(ret == 0){
		msg_serial_pot.serial_pot_state = SENSER_OK;
		sensor_run_info_msg.state = "pot sensor ok";
		pub_sensor_run_info_msg.publish(sensor_run_info_msg);
    }

    while(1){

        tcflush(AdcPort,TCIFLUSH);					//清除掉串口缓存，不然串口会缓存过多数据，导致实时性降低

        for(i=0;i<10;i++)
        {
            ret = get_pot(AdcPort,&pot_temp[i]);
            if((i>0)&(i<9)){
                pot_t = pot_temp[i]+pot_t ;
                if(ret == -1){
					sensor_run_info_msg.error_log++;
					pub_sensor_run_info_msg.publish(sensor_run_info_msg);
                    i--;
                }
            }
        }
        pot_t = pot_t /8;

        if((pot_t < PROTECTION_POT_VALUE_L)||(pot_t > PROTECTION_POT_VALUE_H)){
			msg_serial_pot.serial_pot_state = SENSER_OUT_RANGE;
			sensor_run_info_msg.state = "pot sensor out of range";
			sensor_run_info_msg.error_log++;
			pub_sensor_run_info_msg.publish(sensor_run_info_msg);
        }

		msg_serial_pot.pot = pot_t;
		pub_msg_pot.publish(msg_serial_pot);
		ROS_INFO("pub pot data: %f",msg_serial_pot.pot);
    }

    close(AdcPort);	
}
#endif

#if(WHERE_MOTION == EXOSUIT_VERSION)
uint16_t pot;

void position_callback(const motion_control::sensor_position_msg& pot_input)
{
	pot = pot_input.position;
	sub_semaphore.post();
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
		
	sub_semaphore.wait();
	
	msg_serial_pot.serial_pot_state = SENSER_OK;
	sensor_run_info_msg.state = "pot sensor ok";
	pub_sensor_run_info_msg.publish(sensor_run_info_msg);
	
	pot_t = (float)pot/1000;
	
    if(pot_t < pot_mid){
        pot_t = pot_t + 3.3;
    }
    pot_t = pot_t - pot_mid;	

	msg_serial_pot.pot = pot_t;	
	pub_msg_pot.publish(msg_serial_pot);
	
	for(;;){
		
		sub_semaphore.wait();
		pot_t = (float)pot/1000;
		
		if(pot_t < pot_mid){
			pot_t = pot_t + 3.3;
		}
		pot_t = pot_t - pot_mid;	
		
        if((pot_t < PROTECTION_POT_VALUE_L)||(pot_t > PROTECTION_POT_VALUE_H)){
			msg_serial_pot.serial_pot_state = SENSER_OUT_RANGE;
			sensor_run_info_msg.state = "pot sensor out of range";
			sensor_run_info_msg.error_log++;
			pub_sensor_run_info_msg.publish(sensor_run_info_msg);			
        }		
		msg_serial_pot.pot = pot_t;
		pub_msg_pot.publish(msg_serial_pot);
	}	
}
#endif

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_pot");
    ros::NodeHandle nh;
	
	pub_msg_pot = nh.advertise<motion_control::msg_serial_pot>("msg_serial_pot",1,true);
	pub_sensor_run_info_msg = nh.advertise<motion_control::sensor_run_info_msg>("pot_run_info_msg",1,true);
#if(WHERE_MOTION == EXOSUIT_VERSION)	
	ros::Subscriber sub_position = nh.subscribe("sensor_position_msg", 1, position_callback);
#endif
	boost::thread pot_pub(&pot_pub_loop);
	
	
	ros::spin();
    return 0;	
}