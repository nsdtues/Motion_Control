#ifndef _PREDEFINITION_HEADER
#define _PREDEFINITION_HEADER

#define DESKTOP_VERSION 0                           //穿戴版
#define EXOSUIT_VERSION 1                           //桌面版

#define ENCODE_1MM 778								//1mm对应的电位计计数  778
#define SELF_CHECK_ADJUST_MM 15							//自检对应的最大调整位置 mm
#define MOTION_ADJUST_MM 10									//自适应对应的最大调制位置 mm


#define WHERE_MOTION DESKTOP_VERSION

#if(WHERE_MOTION == EXOSUIT_VERSION)
#define SELF_CHECK_POT_VALUE 2.6
#define ENCODER_DEFUALT_POSITON 100000				//总行程为100000  设置最远点为0
#define MOTOR_ENCODER_DIRECTION 1
#define POT_VALUE_LONG 	1.02						//波登线拉到最长时电位计的数据
#define POT_VALUE_SHORT 1.76						//波登线拉到最短时电位计的数据
#endif

#if(WHERE_MOTION == DESKTOP_VERSION)
#define SELF_CHECK_POT_VALUE 0.45					//自检位置
#define ENCODER_DEFUALT_POSITON 30000				//自检编码器取值
#endif

#define VELOCITY_MODE_MAX_SPEED 1500000			
#define VELOCITY_MODE_MAX_ACC 2000000
#define SELF_CHECK_FORCE_VALUE 200					//自检对应的预紧力值

#define PROTECTION_FORCE_VALUE 2000                 //超过200N自动保护
#define PROTECTION_POT_VALUE_H 3.15                 //电位计保护最大 3.15
#define PROTECTION_POT_VALUE_L 0.3                  //电位计保护最小 0.3

#define PULL_FIX_POSITION 0                         //固定位置模式
#define PULL_FORCE_TORQUE 1                         //力矩环模式
#define PULL_FORCE_TORQUE_TEST 2                    //力矩环测试用
#define STUDY_WALKING_POSITON 3						//跟随行走

#define GAIT_B_MODE PULL_FORCE_TORQUE

#define DEBUG 0                                     //测试用
#define REAL 1

#define RUN_MOTION DEBUG

#define MOTOR_EN_TRUE 1
#define MOTOR_EN_FALSE 0

#define PULL_RATIO_H 1
#define PULL_RATIO_M 0.83
#define PULL_RATIO_L 0.67

#define SYSTEM_OFF 0
#define SYSTEM_ON 1
#define SYSTEM_CLIENT_TEST SYSTEM_OFF				//是否接收系统模块通讯

#define CHANGE_PRELOAD_POSITION 1   				

#define MOTION_MODE_GAIT 1
#define MOTION_MODE_FIXED 2
#define MOTION_MODE_RELAX 3

#define MAX_POSITION_ADJUST 1000					

#define CONFIGURATION_ONE 1
#define SYSTEM_TEST_CONFIGURATION CONFIGURATION_ONE

#endif