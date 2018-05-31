//serial_motor.cpp  
//source ~/Motion_Control/catkin_ws/devel/setup.bash
#include "motion_control/motion_control.h"
#include "boost/thread.hpp"
#include "boost/interprocess/sync/interprocess_semaphore.hpp"
#include <ros/ros.h>
#include "motion_control/msg_serial_force.h"
#include "motion_control/msg_serial_pot.h"
#include "motion_control/msg_motion_cmd.h"
#include "motion_control/msg_motion_evt.h"
#include "motion_control/msg_gait.h"
#include "motion_control/sensor_run_info_msg.h"
#include "predefinition.h"
#include <csignal>

boost::interprocess::interprocess_semaphore sys_cmd_semaphore(0);
boost::interprocess::interprocess_semaphore pot_semaphore(0);
boost::interprocess::interprocess_semaphore force_semaphore(0);

ros::Publisher pub_msg_motion_evt;
ros::Publisher pub_sensor_run_info_msg;

motion_control::sensor_run_info_msg sensor_run_info_msg;

int MotorPort = 0;
uint32_t force_now;
float pot_now;
int32_t EnableFlag = MOTOR_EN_TRUE;
int32_t force_data_flag = 0,pot_data_flag = 0,sys_cmd_flag = 0;
int32_t gait_state;

struct motion_cmd_t{
	uint32_t state;
	uint32_t mode;
	uint32_t foot;
	float forceaid;
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
}motion_cmd_para,motion_cmd_para_rawdata;

//驱动电机的接口，通过串口ASCII码的格式与驱动器通讯，通讯分两种
//一种是带有参数的，需要将参数传入到para
//不带参数的para传入NULL即可
//motion_control.h文件里有所需要的通讯格式的宏定义
//返回为驱动器的反馈（v ***; ok; e **;）
int motor_ctl(const char *msg, int *para,struct motor_ctl_t *rev,int port)
{
    int32_t readcnt = 0,nread,nwrite;
    char com[20];
    uint8_t data[256],datagram[64];
    int *p = para;
    int try_cmd = 5;
    int try_t = 1024;
    struct motor_ctl_t temp;

    if(para == NULL){
        sprintf(com,msg,NULL);
    }else{
        sprintf(com,msg,*p);
    }
    while(try_cmd--){
        nwrite = write(port,com,strlen(com));		//发送串口命令

        memset(data,'\0',256);
        while(try_t--){
            nread = read(port,datagram,1);	//获取串口命令
            if(nread<0){
                return -1;
            }
            if(nread == 0){
                ROS_INFO("Communication fail\n");
				sensor_run_info_msg.state = "can not communication with driver";
				sensor_run_info_msg.error_log++;
				pub_sensor_run_info_msg.publish(sensor_run_info_msg);
                return -1;
            }
            data[readcnt] = datagram[0];
            readcnt++;
            if(datagram[0]==0x0D){	//最后一位为“\n”判断接收到最后一位后再处理数据
                memcpy(temp.com,data,sizeof(data));
                // ROS_INFO("rev = %s\n",temp.com);
                if(strstr(temp.com,"v")!=0){

                    sscanf(temp.com,"%*s%d",&temp.temp);
                    if((void*)rev != NULL){
                        memcpy(rev,&temp,sizeof(struct motor_ctl_t));
                    }
                    return 0;

                }else if(strstr(temp.com,"e")!=0){
                    sscanf(temp.com,"%*s%d",&temp.state);
                    ROS_INFO("motor error = %d\n",temp.state);
					sensor_run_info_msg.error_log++;
					pub_sensor_run_info_msg.publish(sensor_run_info_msg);
                    break;
                }else if(strstr(temp.com,"ok")!=0){
                    return 0;
                }
            }
        }
    }
    return -1;
}

void force_callback(const motion_control::msg_serial_force& force_input)
{
	force_now = force_input.force;
	if(force_input.serial_force_state == SENSER_OUT_RANGE){
		ROS_INFO("position out of range: [%u]\n", force_now);
		EnableFlag = MOTOR_EN_FALSE;
	}
	
	if(pot_data_flag == 0){
		force_semaphore.post();
		force_data_flag = 1;
	}	
	// ROS_INFO("force_now: [%d]\n", force_now);  
}

void pot_callback(const motion_control::msg_serial_pot& pot_input)
{
	pot_now = pot_input.pot;
	if(pot_input.serial_pot_state == SENSER_OUT_RANGE){
		ROS_INFO("position out of range: [%f]\n", pot_now);
		EnableFlag = MOTOR_EN_FALSE;
	}
	
	if(pot_data_flag == 0){
		pot_semaphore.post();
		pot_data_flag = 1;
	}	
	// ROS_INFO("pot: [%f]\n", pot_now);  
}

void motion_cmd_callback(const motion_control::msg_motion_cmd& motion_cmd_input)
{
	motion_cmd_para.state = motion_cmd_input.state;
	motion_cmd_para.mode = motion_cmd_input.mode;
	motion_cmd_para.foot = motion_cmd_input.foot;
	motion_cmd_para.forceaid = motion_cmd_input.forceaid;
	motion_cmd_para.forceaid = PULL_RATIO_H;
	if(sys_cmd_flag == 0){
		motion_cmd_para.max_force = motion_cmd_input.max_force;
		motion_cmd_para.max_position = motion_cmd_input.max_position;
		motion_cmd_para.zero_position = motion_cmd_input.zero_position;
		motion_cmd_para.preload_position = motion_cmd_input.preload_position;
		motion_cmd_para.max_velocity = motion_cmd_input.max_velocity;
		motion_cmd_para.nset_acc = motion_cmd_input.nset_acc;
		motion_cmd_para.max_pot = motion_cmd_input.max_pot;
		motion_cmd_para.pid_kp = motion_cmd_input.pid_kp;
		motion_cmd_para.pid_ki = motion_cmd_input.pid_ki;
		motion_cmd_para.pid_umax = motion_cmd_input.pid_umax;
		motion_cmd_para.pid_umin = motion_cmd_input.pid_umin;
				
		motion_cmd_para.max_force = 900;
		motion_cmd_para.max_position = 6500;
		motion_cmd_para.zero_position = 28500;
		motion_cmd_para.preload_position = 23500;
		motion_cmd_para.max_velocity = 1400000;
		motion_cmd_para.nset_acc = 400000;
		motion_cmd_para.max_pot = 0;
		motion_cmd_para.pid_kp = 3000.0;
		motion_cmd_para.pid_ki = 200.0;
		motion_cmd_para.pid_umax = 100000;
		motion_cmd_para.pid_umin = 100000;
		
		sys_cmd_flag = 1;
		
		motion_cmd_para_rawdata = motion_cmd_para;
		sys_cmd_semaphore.post();
		ROS_INFO("sem post"); 		
	}
}

uint32_t state_check(const uint32_t cmd, const uint32_t old)
{
	if((cmd == 1)&&(old == 2)){
		return old;
	}else if((cmd == 2)&&(old == 3)){
		return old;
	}else if((cmd == 3)&&(old == 1)){
		return old;
	}else{
		return cmd;	
	}
}

void gait_callback(const motion_control::msg_gait& gait_input)
{
	uint32_t state_cmd;
	static uint32_t gait_2_cnt = 0;
	if(gait_input.gait.find("Gait:GaitStopping") != std::string::npos){
		gait_state = 3;
	}else{
		if(motion_cmd_para.foot == 0){
			if(gait_input.gait.find("GaitL:A") != std::string::npos){
				state_cmd = 1;
			}else if(gait_input.gait.find("GaitL:B") != std::string::npos){
				state_cmd = 2;
			}else if(gait_input.gait.find("GaitL:C") != std::string::npos){
				state_cmd = 3;
			}
		}else if(motion_cmd_para.foot == 1){
			if(gait_input.gait.find("GaitR:A") != std::string::npos){
				state_cmd = 1;
			}else if(gait_input.gait.find("GaitR:B") != std::string::npos){
				state_cmd = 2;
			}else if(gait_input.gait.find("GaitR:B") != std::string::npos){
				state_cmd = 3;
			}			
		}else{
			state_cmd = 3;
		}
	}
	
	if(gait_state == 2){
		gait_2_cnt++;
	}else{
		gait_2_cnt = 0;
	}
	if(gait_2_cnt == 25){
		state_cmd = 3;
	}
		
	gait_state = state_check(state_cmd, gait_state);		
	// ROS_INFO("gait_state: [%u]\n", gait_state);  	
}


void motor_ctrl_loop(void)
{
	
    int MotorPort;
    int deltav_motor=0,deltav_motor_old,motor_cmd_position,motor_cmd_velocity,max_position,max_position_adaptive;
    int i,nwrite,index,pndex,dndex,nset_acc,max_force_cnt,motor_speed_t,motor_speed_t_old;
    uint32_t gait_state_temp,state_old=0,max_force;
    uint32_t time_now,time_mark,init_position[10];
    int32_t deltav_force=0,integral_force=0,init_force[10],deltav_force_old,derivative_force,delatv_preload = 0;
    struct timeval tv;
    struct timespec ts;
    int timeout = 1000,is_check = 0;
    struct motor_ctl_t motor_position,motor_state,motor_speed,motor_current;
    int32_t motion_cmd_state,motor_state_old=0;
	
	motion_control::msg_motion_evt msg_motion_evt;
	
    MotorPort = tty_init(MOTOR_PORT_NUM);
    if(MotorPort<0){
		sensor_run_info_msg.state = "can not open /dev/ttyUSBmotor";
		sensor_run_info_msg.error_log++;
		pub_sensor_run_info_msg.publish(sensor_run_info_msg);
    }

    int driver_init_resualt = driver_init(MotorPort,MOTOR_PORT_NUM);
    if(driver_init_resualt == 0){
		sensor_run_info_msg.state = "can not open /dev/ttyUSBmotor";
		sensor_run_info_msg.error_log++;
		pub_sensor_run_info_msg.publish(sensor_run_info_msg);
    }		
	
#if(RUN_MOTION == REAL)
    time_t log_time = time(NULL);
    struct tm *log_tp = localtime(&log_time);
    char log_path[32];

    sprintf(log_path,"/home/pi/work/motor_pi/motor_log/motor_log_%d_%02d_%02d_%02d%02d.txt",log_tp->tm_year + 1900,log_tp->tm_mon+1,log_tp->tm_mday,log_tp->tm_hour,log_tp->tm_min);
    FILE *log_fp = fopen(log_path,"w");
#endif	
	
	sys_cmd_semaphore.wait();
	max_position_adaptive = 0;
	
	for(;;){
		max_position = motion_cmd_para.max_position;
        deltav_motor_old = 0;
        max_force_cnt = 0;		
		gettimeofday(&tv,NULL);
		time_mark = (uint32_t)(tv.tv_sec*1000+tv.tv_usec/1000);		//获取系统时间，单位为ms		
		for(;;){
			if(EnableFlag == MOTOR_EN_TRUE){
				motion_cmd_state = motion_cmd_para.state;
				switch(motion_cmd_state){
					case CTL_CMDINITIAL:
					    if((motor_state_old == 0)||(is_check == 0)){

                        gettimeofday(&tv,NULL);
                        ts.tv_sec = tv.tv_sec;
                        ts.tv_nsec = tv.tv_usec*1000 + timeout*1000*1000;
                        ts.tv_sec += ts.tv_nsec/(1000*1000*1000);
                        ts.tv_nsec %= (1000*1000*1000);

                        if(motor_state_old == 0){
                            int ret1 = pot_semaphore.timed_wait(boost::posix_time::second_clock::universal_time()+ boost::posix_time::seconds(1));
                            int ret2 = force_semaphore.timed_wait(boost::posix_time::second_clock::universal_time()+ boost::posix_time::seconds(1));
                            ROS_INFO("check reualt: %d %d\n",ret1,ret2);

                            if((ret1 == 1)&&(ret2 == 1)){
                                sensor_run_info_msg.state = "motion module is on checking";
								pub_sensor_run_info_msg.publish(sensor_run_info_msg);
                            }else{
                                ROS_INFO("motor module self check abort:can not get senser data\n");
								msg_motion_evt.check_results = no_sensor_data;
                                pub_msg_motion_evt.publish(msg_motion_evt);
								
								sensor_run_info_msg.state = "self check failed: no sensor data";
								sensor_run_info_msg.error_log++;
								pub_sensor_run_info_msg.publish(sensor_run_info_msg);
                                break;
                            }
                        }
                        
                        motor_ctl(GET_MOTOR_FUALT,NULL,&motor_state,MotorPort);
                        nwrite = motor_state.temp;
                        motor_ctl(CLEAR_MOTOR_FUALT,&nwrite,NULL,MotorPort);

                        nwrite = ABSOLUTE_MOVE;
                        motor_ctl(SET_MOVE_MODE,&nwrite,NULL,MotorPort);

                        nwrite = VELOCITY_MODE_MAX_ACC;						
                        motor_ctl(SET_VELOCITY_ACC,&nwrite,NULL,MotorPort);

                        nwrite = VELOCITY_MODE_MAX_ACC;						
                        motor_ctl(SET_VELOCITY_DEC,&nwrite,NULL,MotorPort);

                        nset_acc = motion_cmd_para.nset_acc;											//驱动器的最大加速度设置参数
                        motor_ctl(SET_MAX_DEC,&nset_acc,NULL,MotorPort);
                        nset_acc = motion_cmd_para.nset_acc;
                        motor_ctl(SET_MAX_ACC,&nset_acc,NULL,MotorPort);

                        motor_cmd_velocity = 200000;																		//设置运动速度为14000rpm 此参数需要可以配置
                        motor_ctl(SET_VELOCITY,&motor_cmd_velocity,NULL,MotorPort);

                        //寻零自检，循环次数超过400次认为无法达到
                        int pot_value_try = 400;

                        nwrite = ENABLE_POSITION_MODE;
                        motor_ctl(SET_DESIRED_STATE,&nwrite,NULL,MotorPort);

                        while(pot_value_try--){


                            motor_ctl(GET_POSITION,NULL,&motor_position,MotorPort);

                            if((pot_now>(SELF_CHECK_POT_VALUE+0.02))||(pot_now<(SELF_CHECK_POT_VALUE-0.08))){
#if(WHERE_MOTION == EXOSUIT_VERSION)
                                motor_cmd_position = motor_position.temp + MOTOR_ENCODER_DIRECTION*(SELF_CHECK_POT_VALUE - pot_now)*10000;
#elif(WHERE_MOTION == DESKTOP_VERSION)
                                motor_cmd_position =motor_position.temp - (SELF_CHECK_POT_VALUE - pot_now)*10000;
#endif
                                motor_ctl(SET_MOTION,&motor_cmd_position,NULL,MotorPort);
                                motor_ctl(TRAJECTORY_MOVE,NULL,NULL,MotorPort);
                                ROS_INFO("what is pot: %f pot_value_try:%d\n",pot_now,pot_value_try);

                            }else{
                                motor_ctl(TRAJECTORY_ABORT,NULL,NULL,MotorPort);
                                nwrite = ENCODER_DEFUALT_POSITON;
                                motor_ctl(SET_POSITION,&nwrite,NULL,MotorPort);
                                ROS_INFO("what is pot: %f try_cnt:%d\n",pot_now,400 - pot_value_try);
                                break;
                            }

                        }

                        if(pot_value_try <= 0){
                            ROS_INFO("motor module self check abort:can not reach zero position\n");
							msg_motion_evt.check_results = unreachable_zero_position;
                            pub_msg_motion_evt.publish(msg_motion_evt);		
                            is_check = 1;
							
							sensor_run_info_msg.state = "self check failed: can not reach zero position";
							sensor_run_info_msg.error_log++;
							pub_sensor_run_info_msg.publish(sensor_run_info_msg);							
                            break;
                        }
#if((RUN_MOTION == REAL)||(GAIT_B_MODE == STUDY_WALKING_POSITON))
                        motor_cmd_velocity = 100000;
                        motor_ctl(SET_VELOCITY,&motor_cmd_velocity,NULL,MotorPort);

                        //预紧力点自适应，电位计位置超出或者循环超过1分钟，认为自检失败

                        uint32_t init_mark;

                        gettimeofday(&tv,NULL);
                        init_mark = (uint32_t)(tv.tv_sec*1000+tv.tv_usec/1000);
                        init_mark = (init_mark - time_mark)&0x000fffff;

                        while(1){
							
                            gettimeofday(&tv,NULL);
                            time_now = (uint32_t)(tv.tv_sec*1000+tv.tv_usec/1000);
                            time_now = (time_now - time_mark)&0x000fffff;

                            if((pot>0.1)&&(pot<3.3)&&((time_now - init_mark) < 60000)){

                            }else{
                                ROS_INFO("motor module self check abort:can not reach preload position\n");
                                
                                motor_cmd_position = motion_cmd_para.zero_position;
                                motor_ctl(SET_MOTION,&motor_cmd_position,NULL,MotorPort);
                                motor_ctl(TRAJECTORY_MOVE,NULL,NULL,MotorPort);
								msg_motion_evt.check_results = unreachable_preload_position;
								pub_msg_motion_evt.publish(msg_motion_evt);	
                                is_check = 1;
								
								sensor_run_info_msg.state = "self check failed: can not reach preload position";
								sensor_run_info_msg.error_log++;
								pub_sensor_run_info_msg.publish(sensor_run_info_msg);										
								
                                break;
                            }

                            int32_t init_force_temp[10];
                            uint32_t init_position_temp[10],init_position_overrange;

                            motor_ctl(GET_POSITION,NULL,&motor_position,MotorPort);

                            memcpy(init_force_temp,init_force+1,sizeof(init_force)-4);
                            ROS_INFO("init_temp = ");
                            for(i=0;i<10;i++){
                                ROS_INFO("%d ",init_force_temp[i]);
                            }
                            ROS_INFO("\n");
                            memcpy(init_force,init_force_temp,sizeof(init_force)-4);
                            init_force[9] = SELF_CHECK_FORCE_VALUE - force_now;
                            ROS_INFO("init_force = ");
                            for(i=0;i<10;i++){
                                ROS_INFO("%d ",init_force[i]);
                            }
                            ROS_INFO("\n");


                            memcpy(init_position_temp,init_position+1,sizeof(init_position)-4);
                            memcpy(init_position,init_position_temp,sizeof(init_position)-4);
                            init_position[9] = motor_position.temp;
                            ROS_INFO("init_position = ");
                            for(i=0;i<10;i++){
                                ROS_INFO("%d ",init_position[i]);
                            }
                            ROS_INFO("\n");

                            int init_ret;
                            init_ret = 1;
                            for(i=0;i<10;i++){
                                if((abs(init_force[i]) - 20) < 0){
                                    init_ret = init_ret&1;
                                }else{
                                    init_ret = init_ret&0;
                                }
                            }

                            if(!init_ret){
#if(WHERE_MOTION == EXOSUIT_VERSION)
                                motor_cmd_position = motor_position.temp - MOTOR_ENCODER_DIRECTION*init_force[9]*100;
#elif(WHERE_MOTION == DESKTOP_VERSION)
                                motor_cmd_position =motor_position.temp - init_force[9]*100;
#endif
                                ROS_INFO("what is motor_cmd_position = %d\n",motor_cmd_position);
                                motor_ctl(SET_MOTION,&motor_cmd_position,NULL,MotorPort);
                                motor_ctl(TRAJECTORY_MOVE,NULL,NULL,MotorPort);
                                ROS_INFO("what is init_force = %d\n",init_force[9]);

                            }else{
                                motor_ctl(TRAJECTORY_ABORT,NULL,NULL,MotorPort);
                                for(i=0;i<10;i++){
                                    init_position_overrange = init_position_overrange + init_position[i];
                                }
                                
                                //根据测到的预紧力位置计算零位和最大位置
                                delatv_preload = init_position_overrange/10 - motion_cmd_para.preload_position;
                                motion_cmd_para.preload_position = delatv_preload + motion_cmd_para.preload_position;
								printf("what is preload_position = %d\n",motion_cmd_para.preload_position);
                                if(motion_cmd_para.preload_position > motion_cmd_para_rawdata.preload_position + SELF_CHECK_ADJUST_MM*ENCODE_1MM){
                                    motion_cmd_para.preload_position = motion_cmd_para_rawdata.preload_position + SELF_CHECK_ADJUST_MM*ENCODE_1MM;
                                    motion_cmd_para.zero_position = motion_cmd_para_rawdata.zero_position + SELF_CHECK_ADJUST_MM*ENCODE_1MM;
                                    motion_cmd_para.max_position = motion_cmd_para_rawdata.max_position + SELF_CHECK_ADJUST_MM*ENCODE_1MM;
                                }else if(motion_cmd_para.preload_position < motion_cmd_para_rawdata.preload_position - SELF_CHECK_ADJUST_MM*ENCODE_1MM){
                                    motion_cmd_para.preload_position = motion_cmd_para_rawdata.preload_position - SELF_CHECK_ADJUST_MM*ENCODE_1MM;
                                    motion_cmd_para.zero_position = motion_cmd_para_rawdata.zero_position - SELF_CHECK_ADJUST_MM*ENCODE_1MM;
                                    motion_cmd_para.max_position = motion_cmd_para_rawdata.max_position - SELF_CHECK_ADJUST_MM*ENCODE_1MM;
                                }else{
                                    motion_cmd_para.zero_position = delatv_preload + motion_cmd_para.zero_position;
                                    motion_cmd_para.max_position = delatv_preload + motion_cmd_para.max_position;
                                }

                                max_position_adaptive = 0;
                                printf("what is preload_position = %d\n",motion_cmd_para.preload_position);
                                break;
                            }
                        }

                        if(is_check == 1){
                            break;
                        }

#endif
                        //自检成功配置参数

                        motor_cmd_velocity = 1400000;																		//设置运动速度为14000rpm 此参数需要可以配置
                        motor_ctl(SET_VELOCITY,&motor_cmd_velocity,NULL,MotorPort);

                        //motor_cmd_position = motion_cmd_para.zero_position;
                        motor_cmd_position = motion_cmd_para.preload_position;
                        motor_ctl(SET_MOTION,&motor_cmd_position,NULL,MotorPort);
                        motor_ctl(TRAJECTORY_MOVE,NULL,NULL,MotorPort);
						msg_motion_evt.check_results = module_check_success;
                        pub_msg_motion_evt.publish(msg_motion_evt);	
                        is_check = 1;
						
						sensor_run_info_msg.state = "self check sucsess";
						pub_sensor_run_info_msg.publish(sensor_run_info_msg);	
								
                        ROS_INFO("motion module selfcheck ok\n");
                    }else if((is_check == 1)&&(motor_state_old!=CTL_CMDINITIAL)){
                        is_check = 0;
                    }else{
                        usleep(200000);
                    }
                    break;
                case CTL_CMDPOWERDOWN:																				//关机

                    nwrite = DISABLE_MOTOR;
                    motor_ctl(SET_DESIRED_STATE,&nwrite,NULL,MotorPort);
					msg_motion_evt.check_results = module_check_success;
                    pub_msg_motion_evt.publish(msg_motion_evt);	
                    return;
                    break;	

                case CTL_CMDMOTIONSLEEP:																		//停机

                    if(motion_cmd_state != motor_state_old){
                        nwrite = ENABLE_POSITION_MODE;
                        motor_ctl(SET_DESIRED_STATE,&nwrite,NULL,MotorPort);

                        motor_cmd_position = motion_cmd_para.zero_position;
                        motor_ctl(SET_MOTION,&motor_cmd_position,NULL,MotorPort);
                        motor_ctl(TRAJECTORY_MOVE,NULL,NULL,MotorPort);
						msg_motion_evt.check_results = module_check_success;
                        pub_msg_motion_evt.publish(msg_motion_evt);	
                    }else{
                        usleep(200000);
                    }
                    break;
					
                case CTL_CMDMOTIONSTOP:																						//停止

                    if(motion_cmd_state != motor_state_old){

					    motor_ctl(GET_MOTOR_FUALT,NULL,&motor_state,MotorPort);
                        nwrite = motor_state.temp;
                        motor_ctl(CLEAR_MOTOR_FUALT,&nwrite,NULL,MotorPort);

                        nwrite = DISABLE_MOTOR;
                        motor_ctl(SET_DESIRED_STATE,&nwrite,NULL,MotorPort);
						msg_motion_evt.check_results = module_check_success;
                        pub_msg_motion_evt.publish(msg_motion_evt);	
                    }else{
                        usleep(200000);
                    }
                    break;

                case CTL_CMDMOTIONSTART:																					//开始工作
                    if(motion_cmd_state != motor_state_old){	
						max_position_adaptive = 0;
						max_force_cnt = 1;
						max_force = motion_cmd_para.max_force*motion_cmd_para.forceaid;						
                        nwrite = ENABLE_POSITION_MODE;
                        motor_ctl(SET_DESIRED_STATE,&nwrite,NULL,MotorPort);
						msg_motion_evt.check_results = module_check_success;
                        pub_msg_motion_evt.publish(msg_motion_evt);	
                    }

#if(GAIT_B_MODE == STUDY_WALKING_POSITON)
                    gait_state = 2;
#endif
					gait_state_temp = gait_state;
                    switch(gait_state_temp){
                    case 01:																//预紧点，不能是单个位置点，需要在合适的步态和力矩下开始运动
                        if(gait_state_temp != state_old){

                            integral_force = 0;

                            motor_cmd_velocity = motion_cmd_para.max_velocity*0.5;																		//设置运动速度为14000rpm 此参数需要可以配置
                            motor_ctl(SET_VELOCITY,&motor_cmd_velocity,NULL,MotorPort);
                            
                            nwrite = ENABLE_POSITION_MODE;
                            motor_ctl(SET_DESIRED_STATE,&nwrite,NULL,MotorPort);

                            motor_cmd_position = motion_cmd_para.preload_position;
                            motor_ctl(SET_MOTION,&motor_cmd_position,NULL,MotorPort);
                            motor_ctl(TRAJECTORY_MOVE,NULL,NULL,MotorPort);
                        }
                        motor_ctl(GET_CURRENT,NULL,&motor_current,MotorPort);
                        motor_ctl(GET_POSITION,NULL,&motor_position,MotorPort);
                        gettimeofday(&tv,NULL);
                        time_now = (uint32_t)(tv.tv_sec*1000+tv.tv_usec/1000);
                        time_now = (time_now - time_mark)&0x000fffff;

						if(force_now > 500){						//考虑到预紧力的存在，设置50N的保护限制
//							EnableFlag = MOTOR_EN_FALSE;						
						}						
						
#if((RUN_MOTION == REAL)&&(SYSTEM_TEST_CONFIGURATION == CONFIGURATION_ONE))
						ROS_INFO("time=%u force=%d position=%d state=%d speed_cmd=%d current=%d\n",time_now,force_temp,motor_position.temp,state_temp,0,motor_current.temp);
                        fprintf(log_fp,"time=%u force=%d position=%d state=%d speed_cmd=%d current=%d\n",time_now,force_temp,motor_position.temp,state_temp,0,motor_current.temp);
#elif(RUN_MOTION == REAL)
						ROS_INFO("time=%u force=%d position=%d state=%d speed_cmd=%d\n",time_now,force_temp,motor_position.temp,state_temp,0);
                        fprintf(log_fp,"time=%u force=%d position=%d state=%d speed_cmd=%d\n",time_now,force_temp,motor_position.temp,state_temp,0);
#endif
                        break;

                    case 02:																//拉扯阶段，此阶段需要快速。因此将此阶段分为两段，一段是直接快速运动，当靠近最大位置时再引入力矩环

#if(GAIT_B_MODE==STUDY_WALKING_POSITON)					
                        if(gait_state_temp != state_old){
                            motor_ctl(TRAJECTORY_ABORT,NULL,NULL,MotorPort);
                            nwrite = ENABLE_VELOCITY_MODE;
                            motor_ctl(SET_DESIRED_STATE,&nwrite,NULL,MotorPort);
                        }

                        motor_ctl(GET_POSITION,NULL,&motor_position,MotorPort);
                        deltav_force = 220 - force_now;
                        motor_speed_t = -10000*deltav_force;

                        if((deltav_force < 10)&&(deltav_force >-10)){
                            motor_speed_t = 0;
                        }

                        motor_ctl(SET_VELOCITY_MODE_SPEED,&motor_speed_t,NULL,MotorPort);

                        gettimeofday(&tv,NULL);
                        time_now = (uint32_t)(tv.tv_sec*1000+tv.tv_usec/1000);
                        time_now = (time_now - time_mark)&0x000fffff;
                        ROS_INFO("UTC=%d.%d time=%u force=%d position=%d\n",tv.tv_sec,tv.tv_usec,time_now,force_now,motor_position.temp);
                        break;

#endif					


#if(GAIT_B_MODE==PULL_FORCE_TORQUE_TEST)
                        if(gait_state_temp != state_old){
                            motor_ctl(TRAJECTORY_ABORT,NULL,NULL,MotorPort);

                            motor_cmd_velocity = motion_cmd_para.max_velocity;
                            motor_ctl(SET_VELOCITY,&motor_cmd_velocity,NULL,MotorPort);

                            nwrite = ENABLE_VELOCITY_MODE;
                            motor_ctl(SET_DESIRED_STATE,&nwrite,NULL,MotorPort);
                        }
                        motor_ctl(GET_ACTUAL_SPEED,NULL,&motor_speed,MotorPort);
                        deltav_force = motion_cmd_para.max_force*motion_cmd_para.forceaid - force_now;
                        motor_speed_t = motor_speed.temp - deltav_force*1200;

                        if(pot_now > 2.3){
                            if(motor_speed_t < 0){
                                motor_speed_t = 0;
                            }
                        }

                        if(pot_now < 0.3){
                            motor_speed_t = 0;
                        }

                        if(motor_speed_t < -1600000){															//根据不同的人，设置不同的速度？
                            motor_speed_t = -1600000;
                        }

                        if(motor_speed_t > 1600000){
                            motor_speed_t = 1600000;
                        }
                        ROS_INFO("what is para %d %d %d %f\n",motor_speed_t,deltav_force,motor_speed.temp,pot_now);
                        motor_ctl(SET_VELOCITY_MODE_SPEED,&motor_speed_t,NULL,MotorPort);
                        motor_position.temp = ((2.61 - pot_now)/2.31)*35000;
                        gettimeofday(&tv,NULL);
                        time_now = (uint32_t)(tv.tv_sec*1000+tv.tv_usec/1000);
                        time_now = (time_now - time_mark)&0x000fffff;
                        ROS_INFO("time=%u force=%d position=%d\n",time_now,force_now,motor_position.temp);
#if(RUN_MOTION == REAL)
                        fprintf(log_fp,"time=%u force=%d position=%d state=%d speed_cmd=%d\n",time_now,force_now,motor_position.temp,gait_state_temp,0);
#endif
                        break;
#endif          //PULL_FORCE_TORQUE_TEST

#if(GAIT_B_MODE==PULL_FORCE_TORQUE)

                        if(gait_state_temp != state_old){

                            motor_ctl(TRAJECTORY_ABORT,NULL,NULL,MotorPort);

                            motor_cmd_velocity = motion_cmd_para.max_velocity;																		//设置运动速度为14000rpm 此参数需要可以配置
                            motor_ctl(SET_VELOCITY,&motor_cmd_velocity,NULL,MotorPort);
                            
                            nwrite = ENABLE_VELOCITY_MODE;
                            motor_ctl(SET_DESIRED_STATE,&nwrite,NULL,MotorPort);
                        }

                        motor_ctl(GET_POSITION,NULL,&motor_position,MotorPort);
                        //motor_ctl(GET_ACTUAL_SPEED,NULL,&motor_speed,MotorPort);

                        deltav_force = motion_cmd_para.max_force*motion_cmd_para.forceaid - force_now;

                        if(deltav_motor > motion_cmd_para.pid_umax){
                            if(abs(deltav_force) > 200){
                                index = 0;
                            }else{
                                index = 1;
                                if(deltav_force < 0){
                                    integral_force = integral_force + deltav_force;
                                }
                            }
                        }else if(deltav_motor < motion_cmd_para.pid_umin){
                            if(abs(deltav_force) > 200){
                                index = 0;
                            }else{
                                index = 1;
                                if(deltav_force > 0){
                                    integral_force = integral_force + deltav_force;
                                }
                            }
                        }else{
                            if(abs(deltav_force) > 200){
                                index = 0;
                            }else{
                                index = 1;
                                integral_force = integral_force + deltav_force;
                            }
                        }

                        if(abs(deltav_force) < 10){
                            pndex = 0;
                        }else{
                            pndex = motion_cmd_para.pid_kp;	//1000
                        }

                        index = motion_cmd_para.pid_ki*index;

                        derivative_force = deltav_force_old - deltav_force;
#if(WHERE_MOTION == EXOSUIT_VERSION)
                        motor_speed_t = 0-MOTOR_ENCODER_DIRECTION*(deltav_force*pndex + index*integral_force - derivative_force*0);
#elif(WHERE_MOTION == DESKTOP_VERSION)
                        motor_speed_t = 0-(deltav_force*pndex + index*integral_force - derivative_force*0);
#endif

                        if(motor_speed_t > VELOCITY_MODE_MAX_SPEED)
                            motor_speed_t = VELOCITY_MODE_MAX_SPEED;
                        if(motor_speed_t < -VELOCITY_MODE_MAX_SPEED)
                            motor_speed_t = -VELOCITY_MODE_MAX_SPEED;

                        if(motor_position.temp < motion_cmd_para.max_position - MAX_POSITION_ADJUST ){
                            if(motor_speed_t  < 0){
                                motor_speed_t = 0;
                            }
                        }

                        motor_ctl(SET_VELOCITY_MODE_SPEED,&motor_speed_t,NULL,MotorPort);
#if((RUN_MOTION == REAL)&&(SYSTEM_TEST_CONFIGURATION == CONFIGURATION_ONE))						
                        motor_ctl(GET_CURRENT,NULL,&motor_current,MotorPort);
#endif
                        gettimeofday(&tv,NULL);
                        time_now = (uint32_t)(tv.tv_sec*1000+tv.tv_usec/1000);
                        time_now = (time_now - time_mark)&0x000fffff;

                        deltav_force_old = deltav_force;
#if((RUN_MOTION == REAL)&&(SYSTEM_TEST_CONFIGURATION == CONFIGURATION_ONE))
						ROS_INFO("time=%u force=%d position=%d state=%d speed_cmd=%d current=%d\n",time_now,force_temp,motor_position.temp,state_temp,motor_speed_t,motor_current.temp);
                        fprintf(log_fp,"time=%u force=%d position=%d state=%d speed_cmd=%d current=%d\n",time_now,force_temp,motor_position.temp,state_temp,motor_speed_t,motor_current.temp);
#elif(RUN_MOTION == REAL)
						ROS_INFO("time=%u force=%d position=%d state=%d speed_cmd=%d\n",time_now,force_temp,motor_position.temp,state_temp,motor_speed_t);
                        fprintf(log_fp,"time=%u force=%d position=%d state=%d speed_cmd=%d\n",time_now,force_temp,motor_position.temp,state_temp,motor_speed_t);
#endif
                        break;

#endif // (GAIT_B_MODE==PULL_FORCE_TORQUE)

#if(GAIT_B_MODE==PULL_FIX_POSITION)

                        if(gait_state_temp != state_old){
							
							if(max_force_cnt != 0){
								max_force = (uint32_t)(max_force/max_force_cnt);
								deltav_force = motion_cmd_para.max_force*forceaid_temp - max_force;
								max_position_adaptive = max_position_adaptive - deltav_force*2;		
								// printf("max_position_adaptive : [%d]\n", max_position_adaptive);
							}

							if(max_position_adaptive < 0 - MAX_POSITION_ADJUST){
								max_position_adaptive = 0 - MAX_POSITION_ADJUST;
							}else if(max_position_adaptive > MAX_POSITION_ADJUST){
								max_position_adaptive = MAX_POSITION_ADJUST;
							}	
							
                            motor_cmd_velocity = motion_cmd_para.max_velocity;																		//设置运动速度为14000rpm 此参数需要可以配置
                            motor_ctl(SET_VELOCITY,&motor_cmd_velocity,NULL,MotorPort);
                            motor_cmd_position = max_position_adaptive + motion_cmd_para.max_position;;
                            motor_ctl(SET_MOTION,&motor_cmd_position,NULL,MotorPort);
                            motor_ctl(TRAJECTORY_MOVE,NULL,NULL,MotorPort);

							max_force = 0;
							max_force_cnt = 0;							
                        }

                        motor_ctl(GET_POSITION,NULL,&motor_position,MotorPort);
#if((RUN_MOTION == REAL)&&(SYSTEM_TEST_CONFIGURATION == CONFIGURATION_ONE))						
                        motor_ctl(GET_CURRENT,NULL,&motor_current,MotorPort);
#endif						
                        gettimeofday(&tv,NULL);
                        time_now = (uint32_t)(tv.tv_sec*1000+tv.tv_usec/1000);
                        time_now = (time_now - time_mark)&0x000fffff;

                        if((abs(motor_position.temp - motor_cmd_position) < 100)&&(max_force_cnt < 5)){
                            max_force = max_force + force_now;
                            max_force_cnt++;
                        }

                        ROS_INFO("time=%u force=%d position=%d\n",time_now,force_now,motor_position.temp);
#if((RUN_MOTION == REAL)&&(SYSTEM_TEST_CONFIGURATION == CONFIGURATION_ONE))
                        fprintf(log_fp,"time=%u force=%d position=%d state=%d speed_cmd=%d current=%d\n",time_now,force_now,motor_position.temp,gait_state_temp,0,motor_current.temp);
#elif(RUN_MOTION == REAL)
                        fprintf(log_fp,"time=%u force=%d position=%d state=%d speed_cmd=%d\n",time_now,force_now,motor_position.temp,gait_state_temp,0);
#endif
                        break;

#endif // (GAIT_B_MODE==PULL_FIX_POSITION)

                    case 03:					//回归零点，这个阶段就是快速就够了
                        if(gait_state_temp != state_old){

                            integral_force = 0;

                            motor_cmd_velocity = 1400000;																		//设置运动速度为14000rpm 此参数需要可以配置
                            motor_ctl(SET_VELOCITY,&motor_cmd_velocity,NULL,MotorPort);
                            
                            nwrite = ENABLE_POSITION_MODE;
                            motor_ctl(SET_DESIRED_STATE,&nwrite,NULL,MotorPort);

                            motor_cmd_position = motion_cmd_para.zero_position;
                            motor_ctl(SET_MOTION,&motor_cmd_position,NULL,MotorPort);
                            motor_ctl(TRAJECTORY_MOVE,NULL,NULL,MotorPort);

                            motor_ctl(GET_MOTOR_FUALT,NULL,&motor_state,MotorPort);
							//motor_state
#if(CHANGE_PRELOAD_POSITION == 1)														
                            delatv_preload = (pre_force - SELF_CHECK_FORCE_VALUE)*10;			//自适应调整预紧力位置
                            if(delatv_preload > 2*ENCODE_1MM){
                                delatv_preload = 2*ENCODE_1MM;
                            }else if(delatv_preload < -2*ENCODE_1MM){
                                delatv_preload = -2*ENCODE_1MM;
                            }

                            motion_cmd_para.preload_position = delatv_preload + motion_cmd_para.preload_position;
                            if(motion_cmd_para.preload_position > motion_cmd_para_rawdata.preload_position + MOTION_ADJUST_MM*ENCODE_1MM){
                                motion_cmd_para.preload_position = motion_cmd_para_rawdata.preload_position + MOTION_ADJUST_MM*ENCODE_1MM;
                                motion_cmd_para.zero_position = motion_cmd_para_rawdata.zero_position + MOTION_ADJUST_MM*ENCODE_1MM;
                                motion_cmd_para.max_position = motion_cmd_para_rawdata.max_position + MOTION_ADJUST_MM*ENCODE_1MM;
                            }else if(motion_cmd_para.preload_position <motion_cmd_para_rawdata.preload_position - MOTION_ADJUST_MM*ENCODE_1MM){
                                motion_cmd_para.preload_position = motion_cmd_para_rawdata.preload_position - MOTION_ADJUST_MM*ENCODE_1MM;
                                motion_cmd_para.zero_position = motion_cmd_para_rawdata.zero_position - MOTION_ADJUST_MM*ENCODE_1MM;
                                motion_cmd_para.max_position = motion_cmd_para_rawdata.max_position - MOTION_ADJUST_MM*ENCODE_1MM;
                            }else{
                                motion_cmd_para.zero_position = delatv_preload + motion_cmd_para.zero_position;
                                motion_cmd_para.max_position = delatv_preload + motion_cmd_para.max_position;
                            }
#endif
                        }

                        motor_ctl(GET_ACTUAL_SPEED,NULL,&motor_speed,MotorPort);
#if((RUN_MOTION == REAL)&&(SYSTEM_TEST_CONFIGURATION == CONFIGURATION_ONE))						
                        motor_ctl(GET_CURRENT,NULL,&motor_current,MotorPort);
#endif						
                        motor_ctl(GET_POSITION,NULL,&motor_position,MotorPort);
						
						if(motor_position.temp > motor_para_init_temp.zero_position - 50*ENCODE_1MM){										//在距离零点5cm以内，力传感器反馈大于300N在时保护
							if(force_now > 300){
//								EnableFlag = MOTOR_EN_FALSE;
							}							
						}							
						
                        gettimeofday(&tv,NULL);
                        time_now = (uint32_t)(tv.tv_sec*1000+tv.tv_usec/1000);
                        time_now = (time_now - time_mark)&0x000fffff;
#if((RUN_MOTION == REAL)&&(SYSTEM_TEST_CONFIGURATION == CONFIGURATION_ONE))
						ROS_INFO("time=%u force=%d position=%d state=%d speed_cmd=%d current=%d\n",time_now,force_temp,motor_position.temp,state_temp,0,motor_current.temp);
                        fprintf(log_fp,"time=%u force=%d position=%d state=%d speed_cmd=%d current=%d\n",time_now,force_temp,motor_position.temp,state_temp,0,motor_current.temp);
#elif(RUN_MOTION == REAL)
						ROS_INFO("time=%u force=%d position=%d state=%d speed_cmd=%d\n",time_now,force_temp,motor_position.temp,state_temp,0);
                        fprintf(log_fp,"time=%u force=%d position=%d state=%d speed_cmd=%d\n",time_now,force_temp,motor_position.temp,state_temp,0);
#endif                     
                        break;
                    default:
                        usleep(10000);
                        break;
                    }
                    state_old = gait_state_temp;
                    break;
                default:
                    usleep(100000);
                    break;
                }
                motor_state_old = motion_cmd_state;
            }else{
                nwrite = DISABLE_MOTOR;
                motor_ctl(SET_DESIRED_STATE,&nwrite,NULL,MotorPort);
                usleep(100000);
            }
		}
	}
#if(RUN_MOTION == REAL)
    fclose(log_fp);
#endif
    close(MotorPort);	
}

//退出前需要把电机使能关掉。
void signalHandler( int signum )
{
    ROS_INFO("disable driver ...");
 
	EnableFlag = MOTOR_EN_FALSE;
	usleep(10000);
	close(MotorPort);
	ros::shutdown();
	// exit(signum);  
 
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_motor");
    ros::NodeHandle nh;
	
	pub_msg_motion_evt = nh.advertise<motion_control::msg_motion_evt>("msg_motion_evt",1,true);
	pub_sensor_run_info_msg = nh.advertise<motion_control::sensor_run_info_msg>("motor_run_info_msg",1,true);
	ros::Subscriber sub_force = nh.subscribe("msg_serial_force", 1, force_callback);
	ros::Subscriber sub_pot = nh.subscribe("msg_serial_pot", 1, pot_callback);
	ros::Subscriber sub_motion_cmd = nh.subscribe("msg_motion_cmd", 1, motion_cmd_callback);
	ros::Subscriber sub_gait = nh.subscribe("msg_gait", 1, gait_callback);
	boost::thread motor_ctrl(&motor_ctrl_loop);
	
    // 注册信号 SIGINT 和信号处理程序
    signal(SIGINT, signalHandler);  	
	
	ros::spin();
    return 0;	
}