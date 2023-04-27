#include "moto.h"
#include "stm32f1xx_hal.h"
#include "tim.h"
#include "pid.h"
#include "encoder.h"
#include "icm.h"
#include <math.h>
#include "filter.h"
#include "key.h"
	float sum=0;

void contorl_speed(float speed)
{
	pid[1].sRef=speed;
	pid[1].sFeedBack=enc1;
	PID_Realize(&pid[1],1);
	
	
	static float x=0;
		if(pid[1].fOutput==x)pid[1].fOutput=0;
	x=pid[1].fOutput;
}


void contorl_balance(void)
{
	pid[2].sRef=0;
	pid[2].sFeedBack=eulerAngle.pitch;
	PID_Realize(&pid[2],2);
	
}




void contorl_gyro(void)
{
//	
		
//	sum=0;
//	static float y =0;
//	static float dy =0;
//	static float ddy =0;
//		sum = icm_gyro_y+y+dy+ddy;
//	ddy=dy;
//	dy=y;
//	y=icm_gyro_y;
//	
//	sum=sum/4;
	pid[3].sRef=0;
	pid[3].sFeedBack=icm_gyro_y+9;
 	PID_Realize(&pid[3],3);
	

//	moto_contorl(pid[3].fOutput+pid[2].fOutput,pid[3].fOutput+pid[2].fOutput);
	//if((pid[1].fOutput<30 && pid[1].fOutput>0)|(pid[1].fOutput>-30 && pid[1].fOutput<0))
		pid[1].fOutput=0;

	moto_contorl(pid[3].fOutput+pid[2].fOutput+pid[1].fOutput,pid[3].fOutput+pid[2].fOutput+pid[1].fOutput);
//	moto_contorl(pid[1].fOutput,pid[1].fOutput);
//	moto_contorl(30,30);
	
}

////////////////////////////////////////////////////////////////////////
void contorl(void)
{

//	pid[3].fKp=key1_num;
	
	contorl_speed(0);
	contorl_balance();
	contorl_gyro();				
//  pid[3].fOutput=balance((float)eulerAngle.pitch+2,(float)icm_gyro_y);
//	moto_contorl(pid[3].fOutput,pid[3].fOutput);
}



void moto_contorl1(float speed)
{
//	if(speed>1000)speed=1000;
//	if(speed<-1000)speed=-1000;
	

	if(speed>0)
	{
		
		HAL_GPIO_WritePin(B1_GPIO_Port,B1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(B2_GPIO_Port,B2_Pin,GPIO_PIN_SET);//1
		HAL_GPIO_WritePin(A1_GPIO_Port,A1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(A2_GPIO_Port,A2_Pin,GPIO_PIN_SET);//1
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,speed);	//B
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,speed);	//A
	}

	if(speed<0)
	{
		speed=-speed;
		HAL_GPIO_WritePin(B1_GPIO_Port,B1_Pin,GPIO_PIN_SET);//1
		HAL_GPIO_WritePin(B2_GPIO_Port,B2_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(A1_GPIO_Port,A1_Pin,GPIO_PIN_SET);//1
		HAL_GPIO_WritePin(A2_GPIO_Port,A2_Pin,GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,speed);	//B
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,speed);	//A
	}
	if(speed==0)
	{
		HAL_GPIO_WritePin(B1_GPIO_Port,B1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(B2_GPIO_Port,B2_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(A1_GPIO_Port,A1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(A2_GPIO_Port,A2_Pin,GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,0);	
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,0);	
	}
}

void moto_contorl(float speed_A, float speed_B)
{
	if(speed_A>1000)speed_A=1000;
	if(speed_A<-1000)speed_A=-1000;
	if(speed_B>1000)speed_B=1000;
	if(speed_B<-1000)speed_B=-1000;
////////////////////////////////////////////////////////////////////AAAAAA
	if(speed_A>0)
	{
		HAL_GPIO_WritePin(A1_GPIO_Port,A1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(A2_GPIO_Port,A2_Pin,GPIO_PIN_SET);//1
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,speed_A);	
	}

	if(speed_A<0)
	{
		speed_A=-speed_A;
		HAL_GPIO_WritePin(A1_GPIO_Port,A1_Pin,GPIO_PIN_SET);//1
		HAL_GPIO_WritePin(A2_GPIO_Port,A2_Pin,GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,speed_A);	
	}
/////////////////////////////////////////////////////////////////////BBBBBB
	if(speed_B>0)
	{
		
		HAL_GPIO_WritePin(B1_GPIO_Port,B1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(B2_GPIO_Port,B2_Pin,GPIO_PIN_SET);//1
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,speed_B);	
	}

	if(speed_B<0)
	{
		speed_B=-speed_B;
		HAL_GPIO_WritePin(B1_GPIO_Port,B1_Pin,GPIO_PIN_SET);//1
		HAL_GPIO_WritePin(B2_GPIO_Port,B2_Pin,GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,speed_B);	
	}
	
//////////////////////////////////////////////////00000000000
	if(speed_A==0)
	{

		HAL_GPIO_WritePin(A1_GPIO_Port,A1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(A2_GPIO_Port,A2_Pin,GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,0);	
	}
	if(speed_B==0)
	{
		HAL_GPIO_WritePin(B1_GPIO_Port,B1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(B2_GPIO_Port,B2_Pin,GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,0);	
	}
}
