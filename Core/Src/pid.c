#include "stm32f1xx_hal.h"
#include "pid.h"


PIDData pid[4];

void PID_init()
{
	pid[1].fKp=0.01;
	pid[1].fKd=0.005;
	pid[1].fKi=0.0;
	pid[1].fOutput=0;
	pid[1].fSum=0;
	pid[1].sFeedBack=0;
	pid[1].sRef=0;
	pid[1].sPreError=0;
	pid[1].maxOut=1000;
	
	
	
	pid[2].fKp=34;
	pid[2].fKd=120;
	pid[2].fKi=0.1;
	pid[2].fOutput=0;
	pid[2].fSum=0;
	pid[2].sFeedBack=0;
	pid[2].sRef=0;
	pid[2].sPreError=0;
	pid[2].maxOut=600;
	
	
	
	
	pid[3].fKp=1.9;
	pid[3].fKd=20;
	pid[3].fKi=0.3;
	pid[3].fOutput=0;
	pid[3].fSum=0;
	pid[3].sFeedBack=0;
	pid[3].sRef=0;
	pid[3].sPreError=0;
	pid[3].maxOut=1000;
	
	
}


int balance(float Angle, float Gyro)
{
    float Bias;
    int balance;
    Bias = Angle - 0;                   //===求出平衡的角度中值 和机械相关
    balance = -70 * Bias + Gyro * -200; //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数
    return balance;
}



void PID_zengliang(PIDData * pid,int i)
{
//	  static uint8_t   K_I=1;
	float error=0;
    static float d_error[7]={0},dd_error[7]={0};
    error = pid->sRef - pid->sFeedBack; // 偏差计算
	
	pid->fOutput += (int)(pid->fKp*(error-d_error[i])
												+pid->fKi* error
												+ pid->fKd*(error-2*d_error[i]+dd_error[i]));//增量式
		dd_error[i] = d_error[i];
	d_error[i] = error;
    if(pid->fOutput > pid->maxOut)
    {
        pid->fOutput = pid->maxOut;
    }
    if(pid->fOutput < -pid->maxOut)
    {
        pid->fOutput = -pid->maxOut;
    }
}

 
// 位置式PID控制
void PID_Realize(PIDData * pid,int i )
{
 
	  static uint8_t   K_I=1;
    float  error;
 static float dd_error[7]={0},d_error[7]={0};
	error = pid->sRef - pid->sFeedBack; // 偏差计算	// 计算当前误差
	
	
	dd_error[i] +=  error;	// 误差积分

		dd_error[i]=dd_error[i]*K_I;
	pid->fOutput = pid->fKp * error       //比例P
            + pid->fKi *dd_error[i]   //积分I
			+ pid->fKd * (error - d_error[i]);  //微分D
	d_error[i] = error;		  	// 更新上次误差
	
	
    if(pid->fOutput > pid->maxOut)
    {
        pid->fOutput = pid->maxOut;
    }
    if(pid->fOutput < -pid->maxOut)
    {
        pid->fOutput = -pid->maxOut;
    }
}



void PIDIncCompute(PIDData * pid, int16_t sTunning)
{
//    float  error,d_error,dd_error;
	  static uint8_t   K_I=1;
    float  error,d_error,dd_error;
    error = pid->sRef + sTunning - pid->sFeedBack; // 偏差计算
    d_error = error - pid->sPreError;
    dd_error = d_error - pid->sPreDerror;
//	    if((error > 50)||(error < -50))
//        K_I=0;
//    else
//        K_I=1;
    pid->fOutput += (pid->fKp * d_error + K_I*pid->fKi * error  + pid->fKd*dd_error);

    if(pid->fOutput > pid->maxOut)
    {
        pid->fOutput = pid->maxOut;
    }
    if(pid->fOutput < -pid->maxOut)
    {
        pid->fOutput = -pid->maxOut;
    }
}
