#pragma once
#ifndef __VELOCITYPLAN_H
#define __VELOCITYPLAN_H
#include <stdio.h>
#include <math.h>
#include "includeCfg.h"
#if defined(_WIN32)
#include <windows.h>
#pragma comment(lib,"Winmm.lib")

#ifdef USE_SERIAL_SEND
#include "serial.h"
#endif // USE_SERIAL_SEND

#endif

typedef struct {
	float q0;  //起始位置
	float q1;  //停止位置
	float v0;  //起始速度
	float v1;  //停止速度
	float vmax;  //目标最大速度
	float vlim;  //实际可达到的最大速度
	float amax;  //最大加速度

	float cnt;  //时间戳

	float ta;  //加速时间
	float sa;  //加速阶段的位移

	float tv;  //平均速度时间
	float sv;  //平均速度位移

	float td;  //减速时间
	float sd;  //减速位移

	float t;  //时间总和(单位：s)
}velocityPlanTypedef;

extern void velocityPlanInit();
extern void velocityPlanCallBack();
extern float getTrapezoidDis(velocityPlanTypedef &t, float curTime);
extern float getTrapezoidVelocity(velocityPlanTypedef &t, float curTime);
#endif
