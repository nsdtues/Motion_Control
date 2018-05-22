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

motion_cmd_t motion_cmd_para;

void load_parameter(motion_cmd_t motion_cmd_para_data)
{
    motion_cmd_para = motion_cmd_para_data;
}

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
        nwrite = write(port,com,strlen(com));		//���ʹ�������

        memset(data,'\0',256);
        while(try_t--){
            nread = read(port,datagram,1);	//��ȡ��������
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
            if(datagram[0]==0x0D){	//����һλΪ��\n���жϽ��յ�����һλ���ٴ�������
                memcpy(temp.com,data,sizeof(data));
                //			printf("rev = %s\n",temp.com);
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



void motor_init()
{
    int MotorPort;
    MotorPort = tty_init(MOTOR_PORT_NUM);
    if(MotorPort<0){
        //run_info
    }

    int driver_init_resualt = driver_init(MotorPort,MOTOR_PORT_NUM);
    if(driver_init_resualt == 0){
        //run_info
    }

    if((motor_state_flag_temp_old == 0)||(is_check == 0)){
        gettimeofday(&tv,NULL);
        ts.tv_sec = tv.tv_sec;
        ts.tv_nsec = tv.tv_usec*1000 + timeout*1000*1000;
        ts.tv_sec += ts.tv_nsec/(1000*1000*1000);
        ts.tv_nsec %= (1000*1000*1000);

        if(motor_state_flag_temp_old == 0){
            int ret1 = sem_timedwait(&sem_pot_check,&ts);
            int ret2 = sem_timedwait(&sem_force_check,&ts);
            printf("check reualt: %d %d\n",ret1,ret2);

                if((ret1 == 0)&&(ret2 == 0)){
                    printf("success get senser data\n");
                    pthread_mutex_lock(&mutex_info);
                    motor_module_check_info.motor_module_check_results = on_checking;
                    pthread_mutex_unlock(&mutex_info);
                }else{
                    printf("motor module self check abort:can not get senser data\n");
                    pthread_mutex_lock(&mutex_info);
                    motor_module_check_info.motor_module_check_results = no_sensor_data;
                    pthread_mutex_unlock(&mutex_info);
                    is_check = 1;
                    sem_post(&sem_client);
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

    nset_acc = motor_para_init_temp.nset_acc;											//驱动器的最大加速度设置参数
    motor_ctl(SET_MAX_DEC,&nset_acc,NULL,MotorPort);
    nset_acc = motor_para_init_temp.nset_acc;
    motor_ctl(SET_MAX_ACC,&nset_acc,NULL,MotorPort);

    motor_cmd_velocity = 200000;																		//设置运动速度为14000rpm 此参数需要可以配置
    motor_ctl(SET_VELOCITY,&motor_cmd_velocity,NULL,MotorPort);

    //寻零自检，循环次数超过400次认为无法达到
    int pot_value_try = 400;

    nwrite = ENABLE_POSITION_MODE;
    motor_ctl(SET_DESIRED_STATE,&nwrite,NULL,MotorPort);

    while(pot_value_try--){


        motor_ctl(GET_POSITION,NULL,&motor_position,MotorPort);

        pthread_mutex_lock(&mutex_pot);			//互斥锁
        pot_temp = pot_now;
        pthread_mutex_unlock(&mutex_pot);

        if((pot_temp>(SELF_CHECK_POT_VALUE+0.02))||(pot_temp<(SELF_CHECK_POT_VALUE-0.08))){

            motor_cmd_position = motor_position.temp + MOTOR_ENCODER_DIRECTION*(SELF_CHECK_POT_VALUE - pot_temp)*10000;
            motor_ctl(SET_MOTION,&motor_cmd_position,NULL,MotorPort);
            motor_ctl(TRAJECTORY_MOVE,NULL,NULL,MotorPort);
            printf("what is pot_temp: %f pot_value_try:%d\n",pot_temp,pot_value_try);

        }else{
            motor_ctl(TRAJECTORY_ABORT,NULL,NULL,MotorPort);
            nwrite = ENCODER_DEFUALT_POSITON;
            motor_ctl(SET_POSITION,&nwrite,NULL,MotorPort);
            printf("what is pot_temp: %f try_cnt:%d\n",pot_temp,400 - pot_value_try);
            break;
        }

    }

    if(pot_value_try <= 0){
        printf("motor module self check abort:can not reach zero position\n");
        pthread_mutex_lock(&mutex_info);
        motor_module_check_info.motor_module_check_results = unreachable_zero_position;
        pthread_mutex_unlock(&mutex_info);
        is_check = 1;
        sem_post(&sem_client);
        break;
    }
    motor_cmd_velocity = 100000;
    motor_ctl(SET_VELOCITY,&motor_cmd_velocity,NULL,MotorPort);

    //预紧力点自适应，电位计位置超出或者循环超过1分钟，认为自检失败

    uint32_t init_mark;

    gettimeofday(&tv,NULL);
    init_mark = (uint32_t)(tv.tv_sec*1000+tv.tv_usec/1000);
    init_mark = (init_mark - time_mark)&0x000fffff;

    while(1){

        pthread_mutex_lock(&mutex_pot);			//互斥锁
        pot_temp = pot_now;
        pthread_mutex_unlock(&mutex_pot);
        gettimeofday(&tv,NULL);

        time_now = (uint32_t)(tv.tv_sec*1000+tv.tv_usec/1000);
        time_now = (time_now - time_mark)&0x000fffff;

        if((pot_temp>0.1)&&(pot_temp<3.3)&&((time_now - init_mark) < 60000)){

        }else{
            printf("motor module self check abort:can not reach preload position\n");

            motor_cmd_position = motor_para_init_temp.zero_position;
            motor_ctl(SET_MOTION,&motor_cmd_position,NULL,MotorPort);
            motor_ctl(TRAJECTORY_MOVE,NULL,NULL,MotorPort);

            pthread_mutex_lock(&mutex_info);
            motor_module_check_info.motor_module_check_results = unreachable_zero_position;
            pthread_mutex_unlock(&mutex_info);
            is_check = 1;
            sem_post(&sem_client);
            break;
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

        pthread_mutex_lock(&mutex_force);			//互斥锁
        force_temp = force_now;
        pthread_mutex_unlock(&mutex_force);

        init_force[9] = SELF_CHECK_FORCE_VALUE - force_temp;
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
            if((abmotor_state_flags(init_force[i]) - 20) < 0){
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
            delatv_preload = init_position_overrange/10 - motor_para_init_temp.preload_position;
            motor_para_init_temp.preload_position = delatv_preload + motor_para_init_temp.preload_position;

            if(motor_para_init_temp.preload_position > motor_para_init_rawdata.preload_position + SELF_CHECK_ADJUST_MM*ENCODE_1MM){
                motor_para_init_temp.preload_position = motor_para_init_rawdata.preload_position + SELF_CHECK_ADJUST_MM*ENCODE_1MM;
                motor_para_init_temp.zero_position = motor_para_init_rawdata.zero_position + SELF_CHECK_ADJUST_MM*ENCODE_1MM;
                motor_para_init_temp.max_position = motor_para_init_rawdata.max_position + SELF_CHECK_ADJUST_MM*ENCODE_1MM;
            }else if(motor_para_init_temp.preload_position < preload_position_rawdata - SELF_CHECK_ADJUST_MM*ENCODE_1MM){
                motor_para_init_temp.preload_position = preload_position_rawdata - SELF_CHECK_ADJUST_MM*ENCODE_1MM;
                motor_para_init_temp.zero_position = motor_para_init_rawdata.zero_position - SELF_CHECK_ADJUST_MM*ENCODE_1MM;
                motor_para_init_temp.max_position = motor_para_init_rawdata.max_position - SELF_CHECK_ADJUST_MM*ENCODE_1MM;
            }else{
                motor_para_init_temp.zero_position = delatv_preload + motor_para_init_temp.zero_position;
                motor_para_init_temp.max_position = delatv_preload + motor_para_init_temp.max_position;
            }

            max_position = motor_para_init_temp.max_position;
            printf("what is preload_position = %d\n",motor_para_init_temp.preload_position);
            break;
        }
    }

    if(is_check == 1){
        break;
    }

    //自检成功配置参数

    motor_cmd_velocity = 1400000;						//设置运动速度为14000rpm 此参数需要可以配置
    motor_ctl(SET_VELOCITY,&motor_cmd_velocity,NULL,MotorPort);

    //					motor_cmd_position = motor_para_init_temp.zero_position;
    motor_cmd_position = motor_para_init_temp.preload_position;
    motor_ctl(SET_MOTION,&motor_cmd_position,NULL,MotorPort);
    motor_ctl(TRAJECTORY_MOVE,NULL,NULL,MotorPort);

    pthread_mutex_lock(&mutex_info);
    sprintf(motor_module_check_info.motor_state,"%s",MOTOR_OK);
    motor_module_check_info.motor_module_check_results = module_check_success;
    pthread_mutex_unlock(&mutex_info);

    is_check = 1;
    sem_post(&sem_client);
    printf("this is a test start\n");
}else if((is_check == 1)&&(motor_state_flag_temp_old!=CTL_CMDINITIAL)){
    is_check = 0;
}else{
    usleep(200000);
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
    if(motor_state_flag_temp != motor_state_flag_temp_old)
    {
        nwrite = ENABLE_POSITION_MODE;
        motor_ctl(SET_DESIRED_STATE,&nwrite,NULL,MotorPort);
        motor_cmd_position = motor_para_init_temp.zero_position;
        motor_ctl(SET_MOTION,&motor_cmd_position,NULL,MotorPort);
        motor_ctl(TRAJECTORY_MOVE,NULL,NULL,MotorPort);
        sem_post(&sem_client);
     }else{
        usleep(200000);
     }

}

void motor_stop()
{
    if(motor_state_flag_temp != motor_state_flag_temp_old){

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
        motor_cmd_position = motor_para_init_temp.preload_position;
        motor_ctl(SET_MOTION,&motor_cmd_position,NULL,MotorPort);
        motor_ctl(TRAJECTORY_MOVE,NULL,NULL,MotorPort);
    }

    motor_ctl(GET_CURRENT,NULL,&motor_speed,MotorPort);
    motor_ctl(GET_POSITION,NULL,&motor_position,MotorPort);
    gettimeofday(&tv,NULL);
    time_now = (uint32_t)(tv.tv_sec*1000+tv.tv_usec/1000);
    time_now = (time_now - time_mark)&0x000fffff;

    pthread_mutex_lock(&mutex_force);			//获取当前力矩
    force_temp = force_now;
    pthread_mutex_unlock(&mutex_force);

    //printf("%u  motor_position = %d motor_speed = %d\n",time_now,motor_position.temp,motor_speed.temp);
    printf("time=%u force=%d position=%d\n",time_now,force_temp,motor_position.temp);//time=0 force=123 position=0
    fprintf(log_fp,"time=%u force=%d position=%d state=%d speed_cmd=%d\n",time_now,force_temp,motor_position.temp,state_temp,0);
}

void motor_start_02()
{
    if(state_temp != state_old){

        pthread_mutex_lock(&mutex_force);			//获取当前预紧力
        pre_force = force_now;
        pthread_mutex_unlock(&mutex_force);

        motor_ctl(TRAJECTORY_ABORT,NULL,NULL,MotorPort);

        motor_cmd_velocity = motor_para_init_temp.max_velocity;																		//设置运动速度为14000rpm 此参数需要可以配置
        motor_ctl(SET_VELOCITY,&motor_cmd_velocity,NULL,MotorPort);

        nwrite = ENABLE_VELOCITY_MODE;
        motor_ctl(SET_DESIRED_STATE,&nwrite,NULL,MotorPort);


    }

    motor_ctl(GET_POSITION,NULL,&motor_position,MotorPort);
    //motor_ctl(GET_ACTUAL_SPEED,NULL,&motor_speed,MotorPort);


    pthread_mutex_lock(&mutex_force);			//获取当前力矩
    force_temp = force_now;
    pthread_mutex_unlock(&mutex_force);

    deltav_force = motor_para_init_temp.max_force*forceaid_temp - force_temp;

    if(deltav_motor > motor_para_init_temp.pid_umax){
        if(abs(deltav_force) > 200){
            index = 0;
        }else{
            index = 1;
            if(deltav_force < 0){
                integral_force = integral_force + deltav_force;
            }
        }
    }else if(deltav_motor < motor_para_init_temp.pid_umin){
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
        pndex = motor_para_init_temp.pid_kp;	//1000
    }

    index = motor_para_init_temp.pid_ki*index;

    derivative_force = deltav_force_old - deltav_force;
    motor_speed_t = 0-MOTOR_ENCODER_DIRECTION*(deltav_force*pndex + index*integral_force - derivative_force*0);

    if(motor_speed_t > VELOCITY_MODE_MAX_SPEED)
        motor_speed_t = VELOCITY_MODE_MAX_SPEED;
    if(motor_speed_t < -VELOCITY_MODE_MAX_SPEED)
        motor_speed_t = -VELOCITY_MODE_MAX_SPEED;

    if(motor_position.temp < motor_para_init_temp.max_position - 4000 ){
        if(motor_speed_t  < 0){
            motor_speed_t = 0;
        }
    }

    motor_ctl(SET_VELOCITY_MODE_SPEED,&motor_speed_t,NULL,MotorPort);

    gettimeofday(&tv,NULL);
    time_now = (uint32_t)(tv.tv_sec*1000+tv.tv_usec/1000);
    time_now = (time_now - time_mark)&0x000fffff;

    deltav_force_old = deltav_force;
    printf("time=%u force=%d position=%d\n",time_now,force_temp,motor_position.temp);//time=0 force=123 position=0
    fprintf(log_fp,"time=%u force=%d position=%d state=%d speed_cmd=%d\n",time_now,force_temp,motor_position.temp,state_temp,motor_speed_t);
}

void motor_start_03()
{
    if(state_temp != state_old){

        integral_force = 0;

        motor_cmd_velocity = 1400000;																		//设置运动速度为14000rpm 此参数需要可以配置
        motor_ctl(SET_VELOCITY,&motor_cmd_velocity,NULL,MotorPort);

        nwrite = ENABLE_POSITION_MODE;
        motor_ctl(SET_DESIRED_STATE,&nwrite,NULL,MotorPort);

        motor_cmd_position = motor_para_init_temp.zero_position;
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

        motor_para_init_temp.preload_position = delatv_preload + motor_para_init_temp.preload_position;
        if(motor_para_init_temp.preload_position > motor_para_init_rawdata.preload_position + MOTION_ADJUST_MM*ENCODE_1MM){
            motor_para_init_temp.preload_position = motor_para_init_rawdata.preload_position + MOTION_ADJUST_MM*ENCODE_1MM;
            motor_para_init_temp.zero_position = motor_para_init_rawdata.zero_position + MOTION_ADJUST_MM*ENCODE_1MM;
            motor_para_init_temp.max_position = motor_para_init_rawdata.max_position + MOTION_ADJUST_MM*ENCODE_1MM;
        }else if(motor_para_init_temp.preload_position <motor_para_init_rawdata.preload_position - MOTION_ADJUST_MM*ENCODE_1MM){
            motor_para_init_temp.preload_position = motor_para_init_rawdata.preload_position - MOTION_ADJUST_MM*ENCODE_1MM;
            motor_para_init_temp.zero_position = motor_para_init_rawdata.zero_position - MOTION_ADJUST_MM*ENCODE_1MM;
            motor_para_init_temp.max_position = motor_para_init_rawdata.max_position - MOTION_ADJUST_MM*ENCODE_1MM;
        }else{
            motor_para_init_temp.zero_position = delatv_preload + motor_para_init_temp.zero_position;
            motor_para_init_temp.max_position = delatv_preload + motor_para_init_temp.max_position;
        }
        max_position = motor_para_init_temp.max_position;
    }

    pthread_mutex_lock(&mutex_force);			//获取当前力矩
    force_temp = force_now;
    pthread_mutex_unlock(&mutex_force);

    motor_ctl(GET_ACTUAL_SPEED,NULL,&motor_speed,MotorPort);
    motor_ctl(GET_POSITION,NULL,&motor_position,MotorPort);
    gettimeofday(&tv,NULL);
    time_now = (uint32_t)(tv.tv_sec*1000+tv.tv_usec/1000);
    time_now = (time_now - time_mark)&0x000fffff;
    //printf("%u  motor_position = %d motor_speed = %d\n",time_now,motor_position.temp,motor_speed.temp);
    printf("time=%u force=%d position=%d\n",time_now,force_temp,motor_position.temp);
    fprintf(log_fp,"time=%u force=%d position=%d state=%d speed_cmd=%d\n",time_now,force_temp,motor_position.temp,state_temp,0);
}

void motor_start( u_int16_t motor_start_type)
{
    if(motor_state_flag_temp != motor_state_flag_temp_old){
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
