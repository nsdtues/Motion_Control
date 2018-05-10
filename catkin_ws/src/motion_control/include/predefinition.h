#ifndef _PREDEFINITION_HEADER
#define _PREDEFINITION_HEADER

#define DESKTOP_VERSION 0                           //桌面版
#define EXOSUIT_VERSION 1                           //穿戴版

#define ENCODE_1MM 325								//1mm对应的电位计计数  325
#define SELF_CHECK_ADJUST_MM 30							//自检对应的最大调整位置 mm
#define MOTION_ADJUST_MM 10									//自适应对应的最大调制位置 mm


#define WHERE_MOTION EXOSUIT_VERSION

#if(WHERE_MOTION == EXOSUIT_VERSION)
#define SELF_CHECK_POT_VALUE 2.5
#define ENCODER_DEFUALT_POSITON 30000
#define MOTOR_ENCODER_DIRECTION 1
#define POT_VALUE_LONG 	1.55						//波登线拉到最长时电位计的数据
#define POT_VALUE_SHORT 3.05						//波登线拉到最短时电位计的数据
#endif

#if(WHERE_MOTION == DESKTOP_VERSION)
#define SELF_CHECK_POT_VALUE 0.3					//自检到达的位置
#define ENCODER_DEFUALT_POSITON 35000				//自检设置的编码器取值
#endif

#define VELOCITY_MODE_MAX_SPEED 1500000			
#define VELOCITY_MODE_MAX_ACC 2000000
#define SELF_CHECK_FORCE_VALUE 200					//自检对应的预紧力值

#define PROTECTION_FORCE_VALUE 2000                 //超过200N自动保护
#define PROTECTION_POT_VALUE_H 3.15                  //电位计保护最大3.2
#define PROTECTION_POT_VALUE_L 0.3                  //电位计保护最小0.01

#define PULL_FIX_POSITION 0                         //固定位置模式
#define PULL_FORCE_TORQUE 1                         //力矩还模式
#define PULL_FORCE_TORQUE_TEST 2                    //力矩环，测试用
#define STUDY_WALKING_POSITON 3						//测试人行走位置变化

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
#define SYSTEM_CLIENT_TEST SYSTEM_OFF				//是否接收系统模块的通讯

#define CHANGE_PRELOAD_POSITION 1   				//0 不启用  1 启用    自适应调整预紧力位置

#define MOTION_MODE_GAIT 1
#define MOTION_MODE_FIXED 2
#define MOTION_MODE_RELAX 3


#endif