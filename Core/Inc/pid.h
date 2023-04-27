#ifndef __PID_H
#define __PID_H 

#include "stm32f1xx_hal.h"



struct pidData       //���������������� 
{
    float sRef;
    float sFeedBack;
    float sPreError;  //�ٶ�PID��ǰһ�Σ��ٶ����,vi_Ref - vi_FeedBack 
    float sPreDerror; //�ٶ�PID��ǰһ�Σ��ٶ����֮�d_error-PreDerror; 
  
    float fKp;      //�ٶ�PID��Ka = Kp 
    float fKi;      //�ٶ�PID��Kb = Kp * ( T / Ti ) 
    float fKd;      //�ٶ�PID��
    
		float i_erro_limit;
	
		float fSum;
	
    float fOutput;    //����������ֵ    
		float maxOut;
};
typedef struct pidData PIDData;

extern void PID_init(void);

extern PIDData pid[4];
extern int balance(float Angle, float Gyro);

extern void PID_zengliang(PIDData * pid,int i);
extern void PID_Realize(PIDData * pid,int i );
extern void PIDIncCompute(PIDData * pid, int16_t sTunning);

#endif
