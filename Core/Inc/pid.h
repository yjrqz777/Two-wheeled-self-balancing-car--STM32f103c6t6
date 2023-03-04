#ifndef __PID_H
#define __PID_H 

#include "stm32f1xx_hal.h"



struct pidData       //定义数法核心数据 
{
    float sRef;
    float sFeedBack;
    float sPreError;  //速度PID，前一次，速度误差,vi_Ref - vi_FeedBack 
    float sPreDerror; //速度PID，前一次，速度误差之差，d_error-PreDerror; 
  
    float fKp;      //速度PID，Ka = Kp 
    float fKi;      //速度PID，Kb = Kp * ( T / Ti ) 
    float fKd;      //速度PID，
    
		float fSum;
	
    float fOutput;    //电机控制输出值    
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
