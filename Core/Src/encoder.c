#include "encoder.h"
#include "stm32f1xx_hal.h"
#include "tim.h"
#include "moto.h"


int enc1=0;
int enc2=0;
int enc1_last=0;
int enc2_last=0;



//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//    if(htim->Instance==htim1.Instance)//编码器输入定时器溢出中断                    
//		{
//	moto_contorl(100,0);
//		float k=0.3;
//	enc1 = (uint32_t)(__HAL_TIM_GET_COUNTER(&htim1));//获取定时器的值
//	
////	__HAL_TIM_GET_AUTORELOAD(&htim1);
//	__HAL_TIM_SET_COUNTER(&htim1,0);  // 计数器清零

//	
//	enc2=(uint32_t)(__HAL_TIM_GET_COUNTER(&htim3));
//	__HAL_TIM_SET_COUNTER(&htim3,0);  // 计数器清零
////	if(enc1>30000)enc1=enc1-65536;
////	if(enc2>30000)enc2=enc2-65536;
//	
//	enc1=k*enc1+(1-k)*enc1_last;
//	enc2=k*enc2+(1-k)*enc2_last;
//	enc1_last=enc1;
//	enc2_last=enc2;
//	
//	if(enc1>3000)enc1=65535-enc1;
//	if(enc2>3000)enc2=65535-enc2;
//}
//}
void encoder(void)
{
//	uint16_t i=0;
	float k=0.3;
	enc1 = (uint32_t)(__HAL_TIM_GET_COUNTER(&htim1));//获取定时器的值
	
//	__HAL_TIM_GET_AUTORELOAD(&htim1);
	__HAL_TIM_SET_COUNTER(&htim1,2000);  // 计数器清零

	
	enc2=(uint32_t)(__HAL_TIM_GET_COUNTER(&htim3));
	__HAL_TIM_SET_COUNTER(&htim3,2000);  // 计数器清零
//	if(enc1>30000)enc1=enc1-65536;
//	if(enc2>30000)enc2=enc2-65536;
	
	enc1=k*enc1+(1-k)*enc1_last;
	enc2=k*enc2+(1-k)*enc2_last;
	enc1_last=enc1;
	enc2_last=enc2;
	
	enc1=enc1-2000+3;
	enc2=enc2-2000+3;
//	if(enc1>3000)enc1=65535-enc1;
//	if(enc2>3000)enc2=65535-enc2;
}



//void uncode()  
//{  
//float k=0.3;  
//GetData = __HAL_TIM_GET_COUNTER(&htim8);//0位正，1为负  
//__HAL_TIM_SET_COUNTER(&htim8,0);  //设置定时器
//if(GetData>5000)GetData=GetData-9999;  
//GetData= k*GetData+(1-k)*GetData_last;  //低通滤波
//}  
