//user_time.c
//创建于2018年5月6日
//更新于2018年5月6日
#include "motion_control/motion_control.h"


//用于sem_timedwait 单位ms
struct timespec gettimeout(uint32_t timeout)
{
    struct timeval tv;
    struct timespec ts;

    gettimeofday(&tv,NULL);
    ts.tv_sec = tv.tv_sec + timeout;
    ts.tv_nsec = tv.tv_usec*1000 + timeout*1000*1000;
    ts.tv_sec += ts.tv_nsec/(1000*1000*1000);
    ts.tv_nsec %= (1000*1000*1000);
    return ts;
}

//获取时间ms
uint32_t gettime_ms(struct timeval tv)
{
	uint32_t now;
	gettimeofday(&tv,NULL);
	now = (uint32_t)(tv.tv_sec*1000+tv.tv_usec/1000);	
	return now;
}