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
	float q0;  //��ʼλ��
	float q1;  //ֹͣλ��
	float v0;  //��ʼ�ٶ�
	float v1;  //ֹͣ�ٶ�
	float vmax;  //Ŀ������ٶ�
	float vlim;  //ʵ�ʿɴﵽ������ٶ�
	float amax;  //�����ٶ�

	float cnt;  //ʱ���

	float ta;  //����ʱ��
	float sa;  //���ٽ׶ε�λ��

	float tv;  //ƽ���ٶ�ʱ��
	float sv;  //ƽ���ٶ�λ��

	float td;  //����ʱ��
	float sd;  //����λ��

	float t;  //ʱ���ܺ�(��λ��s)
}velocityPlanTypedef;

extern void velocityPlanInit();
extern void velocityPlanCallBack();
extern float getTrapezoidDis(velocityPlanTypedef &t, float curTime);
extern float getTrapezoidVelocity(velocityPlanTypedef &t, float curTime);
#endif
