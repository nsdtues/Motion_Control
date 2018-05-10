//serial_force.cpp  
//创建于 2018年5月3日
//更新于 2018年5月4日
#include "motion_control/motion_control.h"
#include "boost/thread.hpp"
#include <ros/ros.h>
#include "motion_control/msg_serial_force.h"
#include "predefinition.h"

int FrocePort;
ros::Publisher pub_msg_force;


//获取力传感器数据，一帧的格式为 帧头：0x53 数据：0x** 0x** 校验：两个数据的与 帧尾：0x59
int get_force(int port,uint32_t *msg)
{

    int32_t 		readcnt = 0,nread,ncheck=0,state = 0;
    uint8_t			data[256],datagram[64];
    float	  		adc_temp;
    uint32_t 		force_temp;
    int try_t = 256;
    while(try_t--){
        datagram[0] = 0;
        nread = read(port,datagram,1);
        if(nread<0){
            printf("read error nread=%d\n",readcnt);
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

                    // pthread_mutex_lock(&mutex_info);
                    // motor_module_run_info.force_senser_error++;		//校验错误，收集错误信息
                    // pthread_mutex_unlock(&mutex_info);

                    break;
                }
            }
        case 2:
            if(datagram[0]==0x59){
                data[readcnt] = datagram[0];
                force_temp = (uint32_t)((data[2]<<8)|data[1]);
                state = 0;
                readcnt= 0;
                *msg = force_temp;
                return 0;
                break;
            }else{

                state = 0;
                readcnt= 0;

                // pthread_mutex_lock(&mutex_info);
                // motor_module_run_info.force_senser_error++;		//校验错误，收集错误信息
                // pthread_mutex_unlock(&mutex_info);

                break;
            }
        default:
            break;
        }
    }
    return -1;
}


//对力传感器的数据进行处理
//冒泡算法+平均，除去了最大和最小的一个数
uint32_t bubble_sort_and_average(uint32_t *msg, uint8_t len)
{
    uint32_t temp;
    uint32_t *s;
    int i,j,flag = 1;

    if(len < 3){
        return 0;
    }

    for(j=0;j<len-1&&flag;j++){
        s = msg;
        flag = 0;
        for(i=0;i<len-1-j;i++){
            if(*s > *(s +1)){
                temp = *s;
                *s = *(s+1);
                *(s+1) = temp;
                flag = 1;
            }
            s++;
        }
    }
    s = msg;
    s++;
    temp = 0;
    while((uint32_t*)s<(uint32_t*)msg+len-1){
        temp = temp + *s;
        s ++;
    }
    temp = (uint32_t)(temp/(len-2));
    return temp;
}

void serial_read_loop(void)
{
	motion_control::msg_serial_force msg_serial_force;
	uint32_t force_temp[12];
	int i;
	uint32_t force_t;
		
    FrocePort = tty_init(FORCE_PORT_NUM);
	driver_init(FrocePort,FORCE_PORT_NUM);
	
    int ret = get_force(FrocePort,&force_temp[0]);
    if(ret == -1){
		msg_serial_force.serial_force_state = SENSER_NO_DATA;
    }else if(ret == 0){
		msg_serial_force.serial_force_state = SENSER_OK;
    }

	while(1){
		tcflush(FrocePort,TCIFLUSH);					//清除掉串口缓存，不然串口会缓存过多数据，导致实时性降低

        for(i=0;i<5;i++){
            ret = get_force(FrocePort,&force_temp[i]);
            if(ret == -1){
				//错误信息收集
                i--;
            }
        }
        force_t = bubble_sort_and_average(force_temp,5);
		
		if(force_t > 129){
			force_t = (uint32_t)((0.1107*force_t-14.24)*9.8*10);
		}else{
			force_t = 0;
		}
        
        if(force_t > PROTECTION_FORCE_VALUE){
			msg_serial_force.serial_force_state = SENSER_OUT_RANGE;
        }
        
		msg_serial_force.force = force_t;
		pub_msg_force.publish(msg_serial_force);
	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "serial_force");
	ros::NodeHandle nh;
	
	pub_msg_force = nh.advertise<motion_control::msg_serial_force>("msg_serial_force",1,true);
	
	boost::thread serial_read(&serial_read_loop);
	
	ros::spin();
    return 0;
}