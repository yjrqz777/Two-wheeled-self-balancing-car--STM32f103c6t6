#include "filter.h"
#include "stm32f1xx_hal.h"
#include "icm.h"
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
float K1 =0.02; 
float angle2, angle_dot; 	
float Q_angle=0.001;// 过程噪声的协方差
float Q_gyro=0.003;//0.003 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
float R_angle=0.5;// 测量噪声的协方差 既测量偏差
float dt=0.005;//                 
char  C_0 = 1;
float Q_bias, Angle_err;
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;
float Pdot[4] ={0,0,0,0};
float PP[2][2] = { { 1, 0 },{ 0, 1 } };


KFP KFP_gyro={0.02,0,0,0,0.001,0.543};
KFP KFP_gyro2={0.02,0,0,0,0.001,0.543};


KFP KFP_gyro22={0.02,0,0,0,0.001,0.543};
/**************************************************************************
函数功能：简易卡尔曼滤波
入口参数：加速度、角速度
返回  值：无
**************************************************************************/
void Kalman_Filter(float Accel,float Gyro)		
{
	angle2+=(Gyro - Q_bias) * dt; //先验估计
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

	Pdot[1]=-PP[1][1];
	Pdot[2]=-PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - angle2;	//zk-先验估计
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	angle2	+= K_0 * Angle_err;	 //后验估计
	Q_bias	+= K_1 * Angle_err;	 //后验估计
	angle_dot   = Gyro - Q_bias; //输出值(后验估计)的微分=角速度
}

/**************************************************************************
函数功能：一阶互补滤波
入口参数：加速度、角速度
返回  值：无
**************************************************************************/
void Yijielvbo(float angle_m, float gyro_m)
{
   angle2 = K1 * angle_m+ (1-K1) * (angle2 + gyro_m * 0.005);
}


float adc_data[12] = {0};
float adc_fitter_data=0;
float ADs[6]={0};

float get_em_fitter_data()
{
	uint16_t sum_em = 0,min_position = 0,max_position = 0;
	uint8_t	i=0,j=0;

	
	for(i=0;i<12;i++)
	{
		adc_data[i] = icm_gyro_y;   

		//adc_data[5][i] = adc_once(ADC_P06,ADC_12BIT);
	}	

		min_position=0;
		max_position=0;
		sum_em=0;
		for(j=0;j<12;j++)
		{
			if(adc_data[j]>adc_data[max_position])
				max_position = j;
			if(adc_data[j]<adc_data[min_position])
				min_position = j;
			
			sum_em = sum_em+adc_data[j];
		}
		adc_fitter_data=(sum_em-adc_data[max_position]-adc_data[min_position])/(12-2);
		
		return adc_fitter_data;


}

#define N 12
float filter9( void )
{
    unsigned int count = 0;
    unsigned int new_value = 0, value = 0;
    new_value = icm_gyro_y;
    while( value != new_value )
    {
        count++;
        if( count >= N )
        {
            value = new_value;
            return new_value;
        }
        new_value = icm_gyro_y;
    }
    return value;
}


/**
 *卡尔曼滤波器
 *@param KFP *kfp 卡尔曼结构体参数
 *   float input 需要滤波的参数的测量值（即传感器的采集值）
 *@return 滤波后的参数（最优值）
 */
 float kalmanFilter(KFP *kfp,float input)
 {
     //预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
     kfp->Now_P = kfp->LastP + kfp->Q;
     //卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
     kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
     //更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
     kfp->out = kfp->out + kfp->Kg * (input -kfp->out);//因为这一次的预测值就是上一次的输出值
     //更新协方差方程: 本次的系统协方差付给 kfp->LastP 威下一次运算准备。
     kfp->LastP = (1-kfp->Kg) * kfp->Now_P;
     return kfp->out;
 }



//kalmanFilter(&KFP_height,(float)height);

