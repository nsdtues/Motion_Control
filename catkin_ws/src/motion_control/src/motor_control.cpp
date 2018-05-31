#include "predefinition.h"
#include <time.h>
#include <unistd.h>
#include "motion_control/motion_control.h"

typedef struct motion_cmd_t{
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
}motion_cmd_t;

int MotorPort;
int deltav_motor=0,deltav_motor_old,motor_cmd_position,motor_cmd_velocity,max_position;
int i,nwrite,index,pndex,dndex,nset_acc,max_force_cnt,motor_speed_t,motor_speed_t_old;
uint32_t gait_state_temp,state_old=0,max_force;
uint32_t time_now,time_mark;
int32_t deltav_force=0,integral_force=0,init_force[10],deltav_force_old,derivative_force,delatv_preload = 0;
struct timeval tv;
struct timespec ts;
int timeout = 1000,is_check = 0;
struct motor_ctl_t motor_position,motor_state,motor_speed,motor_current;
int32_t motion_cmd_state,motor_state_old=0;
FILE *log_fp;
struct motor_run_info_t motor_run_info;


motion_cmd_t motion_cmd_para, motion_cmd_para_rawdata;

void load_parameter(motion_cmd_t motion_cmd_para_data)
{
    motion_cmd_para = motion_cmd_para_data;
    motion_cmd_para_rawdata = motion_cmd_para_data;
    max_position = motion_cmd_para_rawdata.max_position;
}

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
                printf("Communication fail\n");
                //run_info
                return -1;
            }
            data[readcnt] = datagram[0];
            readcnt++;
            if(datagram[0]==0x0D){	//最后一位为“\n”判断接收到最后一位后再处理数据
                memcpy(temp.com,data,sizeof(data));
                //printf("rev = %s\n",temp.com);
                if(strstr(temp.com,"v")!=0){

                    sscanf(temp.com,"%*s%d",&temp.temp);
                    if((void*)rev != NULL){
                        memcpy(rev,&temp,sizeof(struct motor_ctl_t));
                    }
                    return 0;

                }else if(strstr(temp.com,"e")!=0){
                    sscanf(temp.com,"%*s%d",&temp.state);
                    printf("motor error = %d\n",temp.state);
                    //run_info
                    break;
                }else if(strstr(temp.com,"ok")!=0){
                    return 0;
                }
            }
        }
    }
    return -1;
}



struct motor_run_info_t motor_init(const float pot_now, const uint32_t force_now)
{
    uint32_t init_position[10];
    MotorPort = tty_init(MOTOR_PORT_NUM);
    if(MotorPort<0){
        //run_info
    }

    int driver_init_resualt = driver_init(MotorPort,MOTOR_PORT_NUM);
    if(driver_init_resualt == 0){
        //run_info
    }

#if(RUN_MOTION == REAL)
    time_t log_time = time(NULL);
    struct tm *log_tp = localtime(&log_time);
    char log_path[32];

    sprintf(log_path,"./motor_log/motor_log_%d_%02d_%02d_%02d%02d.txt",log_tp->tm_year + 1900,log_tp->tm_mon+1,log_tp->tm_mday,log_tp->tm_hour,log_tp->tm_min);
    log_fp = fopen(log_path,"w");
#endif

    if((motor_state_old == 0)||(is_check == 0)){

        //不再需要检查传感器数值？

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

               motor_cmd_position = motor_position.temp + MOTOR_ENCODER_DIRECTION*(SELF_CHECK_POT_VALUE - pot_now)*10000;
               motor_ctl(SET_MOTION,&motor_cmd_position,NULL,MotorPort);
               motor_ctl(TRAJECTORY_MOVE,NULL,NULL,MotorPort);
               printf("what is pot_now: %f pot_value_try:%d\n",pot_now,pot_value_try);

            }else{
                motor_ctl(TRAJECTORY_ABORT,NULL,NULL,MotorPort);
                nwrite = ENCODER_DEFUALT_POSITON;
                motor_ctl(SET_POSITION,&nwrite,NULL,MotorPort);
                printf("what is pot_now: %f try_cnt:%d\n",pot_now,400 - pot_value_try);
                break;
            }

        }

        if(pot_value_try <= 0){
            printf("motor module self check abort:can not reach zero position\n");
            motor_run_info.state = unreachable_zero_position;
            motor_run_info.error_log ++;
            return motor_run_info;
        }
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

            if((pot_now>0.1)&&(pot_now<3.3)&&((time_now - init_mark) < 60000)){

            }else{
                printf("motor module self check abort:can not reach preload position\n");

                motor_cmd_position = motion_cmd_para.zero_position;
                motor_ctl(SET_MOTION,&motor_cmd_position,NULL,MotorPort);
                motor_ctl(TRAJECTORY_MOVE,NULL,NULL,MotorPort);

                //return error can not reach preload position
                motor_run_info.state = unreachable_preload_position;
                motor_run_info.error_log ++;
                return motor_run_info;
            }

            int32_t init_force_temp[10];
            uint32_t init_position_temp[10],init_position_overrange;

            motor_ctl(GET_POSITION,NULL,&motor_position,MotorPort);

            memcpy(init_force_temp,init_force+1,sizeof(init_force)-4);
            printf("init_temp = ");
            for(i=0;i<10;i++){
                printf("%d ",init_force_temp[i]);
            }
            printf("\n");
            memcpy(init_force,init_force_temp,sizeof(init_force)-4);


            init_force[9] = SELF_CHECK_FORCE_VALUE - force_now;
            printf("init_force = ");
            for(i=0;i<10;i++){
                printf("%d ",init_force[i]);
            }
            printf("\n");


            memcpy(init_position_temp,init_position+1,sizeof(init_position)-4);
            memcpy(init_position,init_position_temp,sizeof(init_position)-4);
            init_position[9] = motor_position.temp;
            printf("init_position = ");
            for(i=0;i<10;i++){
                printf("%d ",init_position[i]);
            }
            printf("\n");

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
                motor_cmd_position = motor_position.temp - MOTOR_ENCODER_DIRECTION*init_force[9]*100;
                printf("what is motor_cmd_position = %d\n",motor_cmd_position);
                motor_ctl(SET_MOTION,&motor_cmd_position,NULL,MotorPort);
                motor_ctl(TRAJECTORY_MOVE,NULL,NULL,MotorPort);
                printf("what is init_force = %d\n",init_force[9]);
            }else{
                motor_ctl(TRAJECTORY_ABORT,NULL,NULL,MotorPort);
                for(i=0;i<10;i++){
                     init_position_overrange = init_position_overrange + init_position[i];
                }

                //根据测到的预紧力位置计算零位和最大位置
                delatv_preload = init_position_overrange/10 - motion_cmd_para.preload_position;
                motion_cmd_para.preload_position = delatv_preload + motion_cmd_para.preload_position;

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

                max_position = motion_cmd_para.max_position;
                printf("what is preload_position = %d\n",motion_cmd_para.preload_position);
                break;
            }
        }

        if(is_check == 1){
            break;
        }

        //自检成功配置参数

        motor_cmd_velocity = 1400000;						//设置运动速度为14000rpm 此参数需要可以配置
        motor_ctl(SET_VELOCITY,&motor_cmd_velocity,NULL,MotorPort);

        //motor_cmd_position = motion_cmd_para.zero_position;
        motor_cmd_position = motion_cmd_para.preload_position;
        motor_ctl(SET_MOTION,&motor_cmd_position,NULL,MotorPort);
        motor_ctl(TRAJECTORY_MOVE,NULL,NULL,MotorPort);

        //return check sucsess
        motor_run_info.state = module_check_success;
        return motor_run_info;
        printf("this is a test start\n");
    }
}

void motor_powerdown()
{
    nwrite = DISABLE_MOTOR;
    motor_ctl(SET_DESIRED_STATE,&nwrite,NULL,MotorPort);
    sem_post(&sem_client);
}

void motor_sleep()
{
    if(motor_state_flag_temp != motor_state_old)
    {
        nwrite = ENABLE_POSITION_MODE;
        motor_ctl(SET_DESIRED_STATE,&nwrite,NULL,MotorPort);
        motor_cmd_position = motion_cmd_para.zero_position;
        motor_ctl(SET_MOTION,&motor_cmd_position,NULL,MotorPort);
        motor_ctl(TRAJECTORY_MOVE,NULL,NULL,MotorPort);
        sem_post(&sem_client);
     }else{
        usleep(200000);
     }

}

void motor_stop()
{
    if(motor_state_flag_temp != motor_state_old){

    motor_ctl(GET_MOTOR_FUALT,NULL,&motor_state,MotorPort);
    nwrite = motor_state.temp;
    motor_ctl(CLEAR_MOTOR_FUALT,&nwrite,NULL,MotorPort);

    nwrite = DISABLE_MOTOR;
    motor_ctl(SET_DESIRED_STATE,&nwrite,NULL,MotorPort);
    sem_post(&sem_client);
    }else{
        usleep(200000);
    }
}

void motor_start_01()
{
    if(state_temp != state_old){
        integral_force = 0;
        motor_cmd_velocity = 1400000;																		//设置运动速度为14000rpm 此参数需要可以配置
        motor_ctl(SET_VELOCITY,&motor_cmd_velocity,NULL,MotorPort);
        nwrite = ENABLE_POSITION_MODE;
        motor_ctl(SET_DESIRED_STATE,&nwrite,NULL,MotorPort);
        motor_cmd_position = motion_cmd_para.preload_position;
        motor_ctl(SET_MOTION,&motor_cmd_position,NULL,MotorPort);
        motor_ctl(TRAJECTORY_MOVE,NULL,NULL,MotorPort);
    }

    motor_ctl(GET_CURRENT,NULL,&motor_speed,MotorPort);
    motor_ctl(GET_POSITION,NULL,&motor_position,MotorPort);
    gettimeofday(&tv,NULL);
    time_now = (uint32_t)(tv.tv_sec*1000+tv.tv_usec/1000);
    time_now = (time_now - time_mark)&0x000fffff;


    //printf("%u  motor_position = %d motor_speed = %d\n",time_now,motor_position.temp,motor_speed.temp);
    printf("time=%u force=%d position=%d\n",time_now,force_now,motor_position.temp);//time=0 force=123 position=0
    fprintf(log_fp,"time=%u force=%d position=%d state=%d speed_cmd=%d\n",time_now,force_now,motor_position.temp,state_temp,0);
}

void motor_start_02()
{
    if(state_temp != state_old){

        motor_ctl(TRAJECTORY_ABORT,NULL,NULL,MotorPort);

        motor_cmd_velocity = motion_cmd_para.max_velocity;																		//设置运动速度为14000rpm 此参数需要可以配置
        motor_ctl(SET_VELOCITY,&motor_cmd_velocity,NULL,MotorPort);

        nwrite = ENABLE_VELOCITY_MODE;
        motor_ctl(SET_DESIRED_STATE,&nwrite,NULL,MotorPort);


    }

    motor_ctl(GET_POSITION,NULL,&motor_position,MotorPort);
    //motor_ctl(GET_ACTUAL_SPEED,NULL,&motor_speed,MotorPort);

    deltav_force = motion_cmd_para.max_force*forceaid_temp - force_now;

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
    motor_speed_t = 0-MOTOR_ENCODER_DIRECTION*(deltav_force*pndex + index*integral_force - derivative_force*0);

    if(motor_speed_t > VELOCITY_MODE_MAX_SPEED)
        motor_speed_t = VELOCITY_MODE_MAX_SPEED;
    if(motor_speed_t < -VELOCITY_MODE_MAX_SPEED)
        motor_speed_t = -VELOCITY_MODE_MAX_SPEED;

    if(motor_position.temp < motion_cmd_para.max_position - 4000 ){
        if(motor_speed_t  < 0){
            motor_speed_t = 0;
        }
    }

    motor_ctl(SET_VELOCITY_MODE_SPEED,&motor_speed_t,NULL,MotorPort);

    gettimeofday(&tv,NULL);
    time_now = (uint32_t)(tv.tv_sec*1000+tv.tv_usec/1000);
    time_now = (time_now - time_mark)&0x000fffff;

    deltav_force_old = deltav_force;
    printf("time=%u force=%d position=%d\n",time_now,force_now,motor_position.temp);//time=0 force=123 position=0
    fprintf(log_fp,"time=%u force=%d position=%d state=%d speed_cmd=%d\n",time_now,force_now,motor_position.temp,state_temp,motor_speed_t);
}

void motor_start_03()
{
    if(state_temp != state_old){

        integral_force = 0;

        motor_cmd_velocity = 1400000;																		//设置运动速度为14000rpm 此参数需要可以配置
        motor_ctl(SET_VELOCITY,&motor_cmd_velocity,NULL,MotorPort);

        nwrite = ENABLE_POSITION_MODE;
        motor_ctl(SET_DESIRED_STATE,&nwrite,NULL,MotorPort);

        motor_cmd_position = motion_cmd_para.zero_position;
        motor_ctl(SET_MOTION,&motor_cmd_position,NULL,MotorPort);
        motor_ctl(TRAJECTORY_MOVE,NULL,NULL,MotorPort);

        motor_ctl(GET_MOTOR_FUALT,NULL,&motor_state,MotorPort);
        pthread_mutex_lock(&mutex_info);
        motor_module_run_info.motor_driver_state = motor_state.temp;		//获取驱动器的状态，每个步态周期一次，0为全部正常，512为过流报警
        pthread_mutex_unlock(&mutex_info);

        delatv_preload = (pre_force - SELF_CHECK_FORCE_VALUE)*10;			//自适应调整预紧力位置
        if(delatv_preload > 2*ENCODE_1MM){
            delatv_preload = 2*ENCODE_1MM;
        }else if(delatv_preload < -2*ENCODE_1MM){
            delatv_preload = -2*ENCODE_1MM;
        }

        motion_cmd_para.preload_position = delatv_preload + motion_cmd_para.preload_position;
        if(motion_cmd_para.preload_position > motor_para_init_rawdata.preload_position + MOTION_ADJUST_MM*ENCODE_1MM){
            motion_cmd_para.preload_position = motor_para_init_rawdata.preload_position + MOTION_ADJUST_MM*ENCODE_1MM;
            motion_cmd_para.zero_position = motor_para_init_rawdata.zero_position + MOTION_ADJUST_MM*ENCODE_1MM;
            motion_cmd_para.max_position = motor_para_init_rawdata.max_position + MOTION_ADJUST_MM*ENCODE_1MM;
        }else if(motion_cmd_para.preload_position <motor_para_init_rawdata.preload_position - MOTION_ADJUST_MM*ENCODE_1MM){
            motion_cmd_para.preload_position = motor_para_init_rawdata.preload_position - MOTION_ADJUST_MM*ENCODE_1MM;
            motion_cmd_para.zero_position = motor_para_init_rawdata.zero_position - MOTION_ADJUST_MM*ENCODE_1MM;
            motion_cmd_para.max_position = motor_para_init_rawdata.max_position - MOTION_ADJUST_MM*ENCODE_1MM;
        }else{
            motion_cmd_para.zero_position = delatv_preload + motion_cmd_para.zero_position;
            motion_cmd_para.max_position = delatv_preload + motion_cmd_para.max_position;
        }
        max_position = motion_cmd_para.max_position;
    }

    motor_ctl(GET_ACTUAL_SPEED,NULL,&motor_speed,MotorPort);
    motor_ctl(GET_POSITION,NULL,&motor_position,MotorPort);
    gettimeofday(&tv,NULL);
    time_now = (uint32_t)(tv.tv_sec*1000+tv.tv_usec/1000);
    time_now = (time_now - time_mark)&0x000fffff;
    //printf("%u  motor_position = %d motor_speed = %d\n",time_now,motor_position.temp,motor_speed.temp);
    printf("time=%u force=%d position=%d\n",time_now,force_now,motor_position.temp);
    fprintf(log_fp,"time=%u force=%d position=%d state=%d speed_cmd=%d\n",time_now,force_now,motor_position.temp,state_temp,0);
}

void motor_start( u_int16_t motor_start_type)
{
    if(motor_state_flag_temp != motor_state_old){
        pthread_mutex_lock(&mutex_client_msg);
        forceaid_temp = forceaid;
        pthread_mutex_unlock(&mutex_client_msg);

        nwrite = ENABLE_POSITION_MODE;
        motor_ctl(SET_DESIRED_STATE,&nwrite,NULL,MotorPort);
        sem_post(&sem_client);
        printf("where are you\n");
    }

    pthread_mutex_lock(&mutex_gait_msg);					//获取电机运动状态，socket
    state_temp = state_now;
    pthread_mutex_unlock(&mutex_gait_msg);

    switch(state_temp){
    case 01:																//预紧点，不能是单个位置点，需要在合适的步态和力矩下开始运动
    motor_start_01();
    break;

    case 02:																//拉扯阶段，此阶段需要快速。因此将此阶段分为两段，一段是直接快速运动，当靠近最大位置时再引入力矩环
    motor_start_01();
    break;

    case 03:					//回归零点，这个阶段就是快速就够了
    motor_start_01();
    break;
    default:
        usleep(10000);
        break;
    }
}
