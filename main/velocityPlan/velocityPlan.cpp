#include "velocityPlan.h"

velocityPlanTypedef tVel = { 0 };
//���ù滮�ı�Ҫ��ʼ����
void setVelocityPlanParam(velocityPlanTypedef &p, float q0, float q1, float v0, float v1, float vmax, float amax) {
	p.q0 = q0;
	p.q1 = q1;
	p.v0 = v0;
	p.v1 = v1;
	p.vmax = vmax;
	p.amax = amax;
}

//�����ٶȹ滮����
void trapezoidVelocityPlan(velocityPlanTypedef &t) {
	float v_temp = 0.0f;

	v_temp = sqrt((2.0*t.amax*(t.q1 - t.q0) + (pow(t.v1, 2) + pow(t.v0, 2))) / 2.0f);

	t.vlim = (v_temp < t.vmax) ? v_temp : t.vmax;    //����ɴﵽ����ٶ�

	t.ta = (t.vlim - t.v0) / t.amax;
	t.sa = t.v0*t.ta + t.amax*pow(t.ta, 2) / 2;      //���ٽ׶ε�ʱ���λ�Ƽ���

	t.tv = (t.q1 - t.q0
		- ((pow(t.vlim, 2) - pow(t.v0, 2)) / (2 * t.amax))
		- ((pow(t.v1, 2) - pow(t.vlim, 2)) / (2 * -t.amax))) / t.vlim;
	t.sv = t.vlim * t.tv;                //���ٽ׶�ʱ���λ�Ƽ���

	t.td = (t.vlim - t.v1) / t.amax;
	t.sd = t.vlim * t.td - t.amax * pow(t.td, 2) / 2;  //���ٽ׶�ʱ���λ�Ƽ���

	t.t = t.ta + t.tv + t.td;              //����Ӽ����ܺ�ʱ
}

//�������μӼ�������ʱ�̵��ٶ�
float getTrapezoidVelocity(velocityPlanTypedef &t, float curTime) {

	float qd = 0.0f;//�ٶ�
	if (curTime >= 0 && curTime < t.ta) {          //���ٽ׶�
		qd = t.v0 + t.amax*curTime;
	}
	else if (curTime >= t.ta && curTime < t.ta + t.tv) {  //���ٽ׶�
		qd = t.vlim;
	}
	else if (curTime >= t.ta + t.tv && curTime <= t.t) {  //���ٽ׶�
		qd = t.vlim - t.amax*(curTime - t.ta - t.tv);
	}
	else {
		printf("Parameter exception!!!\r\n");        //����
	}
	return qd;
}

//�������μӼ�������ʱ�̵�λ��
float getTrapezoidDis(velocityPlanTypedef &t, float curTime) {

	float q = 0.0f;      //λ��
	if (curTime >= 0 && curTime < t.ta) {          //���ٽ׶�
		q = t.q0 + t.v0*curTime + t.amax*pow(curTime, 2) / 2;
	}
	else if (curTime >= t.ta && curTime < t.ta + t.tv) {  //���ٽ׶�
		q = t.q0 + t.sa + t.vlim*(curTime - t.ta);
	}
	else if (curTime >= t.ta + t.tv && curTime <= t.t) {  //���ٽ׶�
		q = t.q0 + t.sa + t.sv + t.vlim*(curTime - t.ta - t.tv) - t.amax*pow(curTime - t.ta - t.tv, 2) / 2;
	}
	else {
		printf("Parameter exception!!!\r\n");        //����
	}
	return q;
}

//�滮������ӡ
void planLog(velocityPlanTypedef t) {
	printf("---------------------------------------------------------\r\n");
	printf("q0 = %.3f v0 = %.3f \r\n", t.q0, t.v0);
	printf("q1 = %.3f v1 = %.3f \r\n", t.q1, t.v1);
	printf("vmax = %.3f vlim = %.3f amax = %.3f \r\n", t.vmax, t.vlim, t.amax);

	printf("ta = %.3f sa = %.3f \r\n", t.ta, t.sa);
	printf("tv = %.3f sv = %.3f \r\n", t.tv, t.sv);
	printf("td = %.3f sd = %.3f \r\n", t.td, t.sd);

	printf("t = %.3f \r\n", t.t);
	printf("---------------------------------------------------------\r\n");
}

//�����ٶȹ滮����
void velocityPlanMainTest(velocityPlanTypedef &t) {
#define FUN_TEST      //���Ժ����궨��
	int k = 1;
	float unit = 0.010f;  //�жϹ滮ʱ��
	float curTime = 0.0f;
	float q = 0.0f;      //λ��
	float qd = 0.0f;    //�ٶ�
	float qdd = 0.0f;    //���ٶ�
	do {
		curTime = unit * k;
#if !defined(FUN_TEST)
		if (curTime >= 0 && curTime < t.ta) {          //���ٽ׶�
			q = t.q0 + t.v0*curTime + t.amax*pow(curTime, 2) / 2;
			qd = t.v0 + t.amax*curTime;
			qdd = t.amax;
		}
		else if (curTime >= t.ta && curTime < t.ta + t.tv) {  //���ٽ׶�
			q = t.q0 + t.sa + t.vlim*(curTime - t.ta);
			qd = t.vlim;
			qdd = 0;
		}
		else if (curTime >= t.ta + t.tv && curTime <= t.t) {  //���ٽ׶�
			q = t.q0 + t.sa + t.sv + t.vlim*(curTime - t.ta - t.tv) - t.amax*pow(curTime - t.ta - t.tv, 2) / 2;
			qd = t.vlim - t.amax*(curTime - t.ta - t.tv);
			qdd = -t.amax;
		}
#else
		qd = getTrapezoidVelocity(t, curTime);          //��ȡ����ʱ����ٶ�
		q = getTrapezoidDis(t, curTime);            //��ȡ����ʱ���λ��
#endif

#ifdef USE_SERIAL_SEND
		sendWave(3, (int)(q * 100), (int)(qd * 100), (int)(qdd * 100));
#endif // 
		printf("k %d, t = %.3f,q=%.3f,qd=%.3f,qdd=%.3f\r\n", k, curTime, q, qd, qdd);
		Sleep(1);
		k++;
	} while (curTime < t.t);
}

//��ʱ���ص�
void velocityPlanCallBack() {
}

void velocityPlanInit() {
	DWORD_PTR dwUser = 0;
	MMRESULT timerID = timeSetEvent(100, 1, (LPTIMECALLBACK)velocityPlanCallBack, dwUser, TIME_PERIODIC);
	if (timerID != 0) {
		printf(">>> ��ʱ�������ɹ�\n");
		memset(&tVel, 0, sizeof(velocityPlanTypedef));
	}

	setVelocityPlanParam(tVel, 0, 300, 0, 0, 100, 100);
	trapezoidVelocityPlan(tVel);
	planLog(tVel);
	velocityPlanMainTest(tVel);
}