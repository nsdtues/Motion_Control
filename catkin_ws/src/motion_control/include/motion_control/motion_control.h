﻿#ifndef _MOTION_CONTROL_HEADER
#define _MOTION_CONTROL_HEADER


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <termios.h>
#include <time.h>
#include <sys/time.h>
#include <stdint.h>
#include <sys/un.h> 
// #include <zmq.h>
#include <assert.h>
#include <stdbool.h>
#include <semaphore.h>
// #include "Properties.h"


#ifdef __cplusplus
extern "C" {
#endif

//控制消息类型
typedef enum 
{
    CTL_CMDINITIAL = 55,
    CTL_CMDPOWERDOWN,
    CTL_CMDMOTIONSTOP,
    CTL_CMDMOTIONSTART,
    CTL_CMDMOTIONSLEEP,
} CTL_CMDTypeDef;

struct motor_ctl_t{
	char com[256];
	int32_t temp;
	int state;
};

struct motor_module_check_info_t{
	char pot_port_state[32];
	char force_port_state[32];
	char motor_port_state[32];
	char motor_state[32];
	uint32_t motor_module_check_results;	
};

enum check_results_t{
	on_checking = 1,
	no_sensor_data ,
	unreachable_zero_position,
	unreachable_preload_position,
	module_check_success,
	check_results_max
};


#define SENSER_OK			"sensor_ok"
#define SENSER_NO_PORT		"sensor_noport"
#define SENSER_NO_DATA		"sensor_nodata"
#define SENSER_OUT_RANGE	"sensor_outofrange"

#define MOTOR_OK					"motor_ok"
#define MOTOR_OVER_C			"motor_over_current"
#define MOTOR_OVER_V			"motor_over_voltage"
#define MOTOR_NO_MOTOR		"motor_no_motor"
#define MOTOR_NO_ENCODER	"motor_no_encoder"
#define MOTOR_NO_PORT			"motor_no_port"
#define MOTOR_NO_DATA			"motor_no_data"

struct motor_module_run_info_t{
	uint32_t force_senser_error;
	uint32_t pot_senser_error;
	uint32_t motor_driver_state;
	uint32_t motor_driver_error;
	uint32_t zeromq_recv_client_error;
	uint32_t zeromq_recv_gait_error;
};

struct motor_para_init_t{
	uint32_t max_force;
	int32_t max_position;
	int32_t zero_position;
	int32_t preload_position;
	int max_velocity;
	int nset_acc;
	float	max_pot;
	float pid_kp;
	float pid_ki;
	int pid_umax;
	int pid_umin;
};

//pot.c
int set_opt(int fd,int nSpeed,int nBits,char nEvent,int nStop);
int driver_init(int port,const char* port_num);
int tty_init(const char *CurrentPort);
//user_time.c
struct timespec gettimeout(uint32_t timeout);
uint32_t gettime_ms(struct timeval tv);


#ifdef __cplusplus
}
#endif


#define FORCE "Force"
#define GAIT "Gait"

#define ADC_PORT_NUM  "/dev/ttyUSBpot"
#define MOTOR_PORT_NUM "/dev/ttyUSBmotor"
#define FORCE_PORT_NUM "/dev/ttyUSBforce"

#define DRIVER_G "g"
#define DRIVER_V "v"
#define DRIVER_E "e"
#define DRIVER_OK "ok"

#define SET_POSITION "s r0x32 %d\n"
#define GET_POSITION "g r0x32\n"
#define SET_MOTION "s r0xca %d\n"
#define SET_VELOCITY "s r0xcb %d\n"
#define GET_CURRENT "g r0x0c\n"
#define TRAJECTORY_ABORT "t 0\n"
#define TRAJECTORY_MOVE "t 1\n"
#define ABSOLUTE_MOVE 0
#define RELATIVE_MOVE 256
#define SET_MOVE_MODE "s r0xc8 %d\n"	//0 absolute move   256 relative move
#define DISABLE_MOTOR 0
#define ENABLE_POSITION_MODE 21
#define ENABLE_VELOCITY_MODE 11
#define SET_DESIRED_STATE "s r0x24 %d\n"		//0 disable the move 
																						//21 enable position mode
																						//11 enable velocity mode
#define SET_BAUD_115200 "s r0x90 115200\n"
#define GET_MOTOR_FUALT	"g r0xa4\n"
#define CLEAR_MOTOR_FUALT "s r0xa4 %d\n"
#define SET_MAX_ACC "s r0xcc %d\n"
#define SET_MAX_DEC "s r0xcd %d\n"

#define SET_VELOCITY_ACC "s r0x36 %d\n"			//velocity mode acc limit
#define SET_VELOCITY_DEC "s r0x37 %d\n"			//velocity mode dec limit
#define SET_VELOCITY_MODE_SPEED "s r0x2f %d\n"		
#define GET_ACTUAL_SPEED "g r0x18\n"

#endif