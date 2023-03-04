#include "main.h"

/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		ICM20602
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		查看doc内version文件 版本说明
 * @Software 		IAR 8.3 or MDK 5.26
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2018-05-24
 * @note		
					接线定义：
					------------------------------------ 
						SCL                 查看SEEKFREE_IIC文件内的SEEKFREE_SCL宏定义
						SDA                 查看SEEKFREE_IIC文件内的SEEKFREE_SDA宏定义
					------------------------------------ 
 ********************************************************************************************************************/


#include "main.h"
#include "stm32f1xx_hal.h"
#include "icm.h"
#include <math.h>
#include "filter.h"
int16_t icm_gyro_x,icm_gyro_y,icm_gyro_z;
int16_t icm_acc_x,icm_acc_y,icm_acc_z;



//-------------------------------------------------------------------------------------------------------------------
//  以下函数是使用软件SPI通信，相比较硬件SPI，软件SPI引脚更加灵活，可以使用任意普通IO
//-------------------------------------------------------------------------------------------------------------------


#define ICM20602_SCK(x)		HAL_GPIO_WritePin(ICM20602_SCK_GPIO_Port,ICM20602_SCK_Pin,(GPIO_PinState)x)
#define ICM20602_MOSI(x) 	HAL_GPIO_WritePin(ICM20602_MOSI_GPIO_Port,ICM20602_MOSI_Pin,(GPIO_PinState)x)
#define ICM20602_CSN(x)  	HAL_GPIO_WritePin(ICM20602_CS_GPIO_Port,ICM20602_CS_Pin,(GPIO_PinState)x)
#define ICM20602_MISO    	(uint8_t)HAL_GPIO_ReadPin(ICM20602_MISO_GPIO_Port,ICM20602_MISO_Pin)

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ICM20602自检函数
//  @param      NULL
//  @return     void					
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void icm20602_self4_check(void)
{
    uint8_t val;
    do
    {
        icm_simspi_r_reg_byte(ICM20602_WHO_AM_I,&val);
        //卡在这里原因有以下几点
        //1 MPU6050坏了，如果是新的这样的概率极低
        //2 接线错误或者没有接好
        //3 可能你需要外接上拉电阻，上拉到3.3V
    }while(0x12 != val);
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      软件SPI，初始化ICM20602
//  @param      NULL
//  @return     void					
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void icm20602_init_simspi(void)
{
    uint8_t val;
    

    icm20602_self4_check();//检测
    
    icm_simspi_w_reg_byte(ICM20602_PWR_MGMT_1,0x80);//复位设备
    HAL_Delay(2);
    do
    {//等待复位成功
        icm_simspi_r_reg_byte(ICM20602_PWR_MGMT_1,&val);
    }while(0x41 != val);
    
    icm_simspi_w_reg_byte(ICM20602_PWR_MGMT_1,     0x01);            //时钟设置
    icm_simspi_w_reg_byte(ICM20602_PWR_MGMT_2,     0x00);            //开启陀螺仪和加速度计
    icm_simspi_w_reg_byte(ICM20602_CONFIG,         0x01);            //176HZ 1KHZ
    icm_simspi_w_reg_byte(ICM20602_SMPLRT_DIV,     0x07);            //采样速率 SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
    icm_simspi_w_reg_byte(ICM20602_GYRO_CONFIG,    0x18);            //±2000 dps
    icm_simspi_w_reg_byte(ICM20602_ACCEL_CONFIG,   0x10);            //±8g
    icm_simspi_w_reg_byte(ICM20602_ACCEL_CONFIG_2, 0x03);            //Average 8 samples   44.8HZ
	//ICM20602_GYRO_CONFIG寄存器
    //设置为:0x00 陀螺仪量程为:±250 dps     获取到的陀螺仪数据除以131           可以转化为带物理单位的数据， 单位为：°/s
    //设置为:0x08 陀螺仪量程为:±500 dps     获取到的陀螺仪数据除以65.5          可以转化为带物理单位的数据，单位为：°/s
    //设置为:0x10 陀螺仪量程为:±1000dps     获取到的陀螺仪数据除以32.8          可以转化为带物理单位的数据，单位为：°/s
    //设置为:0x18 陀螺仪量程为:±2000dps     获取到的陀螺仪数据除以16.4          可以转化为带物理单位的数据，单位为：°/s

    //ICM20602_ACCEL_CONFIG寄存器
    //设置为:0x00 加速度计量程为:±2g          获取到的加速度计数据 除以16384      可以转化为带物理单位的数据，单位：g(m/s^2)
    //设置为:0x08 加速度计量程为:±4g          获取到的加速度计数据 除以8192       可以转化为带物理单位的数据，单位：g(m/s^2)
    //设置为:0x10 加速度计量程为:±8g          获取到的加速度计数据 除以4096       可以转化为带物理单位的数据，单位：g(m/s^2)
    //设置为:0x18 加速度计量程为:±16g         获取到的加速度计数据 除以2048       可以转化为带物理单位的数据，单位：g(m/s^2)
    
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取ICM20602加速度计数据
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				执行该函数后，直接查看对应的变量即可
//-------------------------------------------------------------------------------------------------------------------
void get_icm20602_accdata_simspi(void)
{
    uint8_t dat[6];
    
    icm_simspi_r_reg_bytes(ICM20602_ACCEL_XOUT_H, dat, 6);
    icm_acc_x = (int16_t)(((uint16_t)dat[0]<<8 | dat[1]));
    icm_acc_y = (int16_t)(((uint16_t)dat[2]<<8 | dat[3]));
    icm_acc_z = (int16_t)(((uint16_t)dat[4]<<8 | dat[5]));
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取ICM20602陀螺仪数据
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				执行该函数后，直接查看对应的变量即可
//-------------------------------------------------------------------------------------------------------------------
void get_icm20602_gyro_simspi(void)
{
    uint8_t dat[6];
    
    icm_simspi_r_reg_bytes(ICM20602_GYRO_XOUT_H, dat, 6);
    icm_gyro_x = (int16_t)(((uint16_t)dat[0]<<8 | dat[1]));
    icm_gyro_y = (int16_t)(((uint16_t)dat[2]<<8 | dat[3]));
    icm_gyro_z = (int16_t)(((uint16_t)dat[4]<<8 | dat[5]));
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      通过SPI写一个byte,同时读取一个byte
//  @param      byte        发送的数据    
//  @return     uint8_t       return 返回status状态
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
uint8_t icm_simspi_wr_byte(uint8_t byte)
{
    uint8_t i;
    for(i=0; i<8; i++)
    {
        ICM20602_MOSI((GPIO_PinState)(byte&0x80));
        byte <<= 1;
        ICM20602_SCK (0);    
        byte |= ICM20602_MISO;        
        ICM20602_SCK (1);
    }	
    return(byte);                                      		
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      将val写入cmd对应的寄存器地址,同时返回status字节
//  @param      cmd         命令字
//  @param      val         待写入寄存器的数值
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void icm_simspi_w_reg_byte(uint8_t cmd, uint8_t val)
{
    ICM20602_CSN (0);
    cmd |= ICM20602_SPI_W;
    icm_simspi_wr_byte(cmd);                      	
    icm_simspi_wr_byte(val);                               	
    ICM20602_CSN (1);                                    	
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      读取cmd所对应的寄存器地址
//  @param      cmd         命令字
//  @param      *val        存储读取的数据地址
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void icm_simspi_r_reg_byte(uint8_t cmd, uint8_t *val)
{
    ICM20602_CSN (0);
    cmd |= ICM20602_SPI_R;
    icm_simspi_wr_byte(cmd);                               	
    *val = icm_simspi_wr_byte(0);                           	
    ICM20602_CSN (1);                                    	
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      读取cmd所对应的寄存器地址
//  @param      cmd         命令字
//  @param      *val        存储读取的数据地址
//  @param      num         读取的数量
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void icm_simspi_r_reg_bytes(uint8_t cmd, uint8_t *val, uint8_t num)
{
    uint16_t i;
    ICM20602_CSN (0);
    cmd |= ICM20602_SPI_R;
    icm_simspi_wr_byte(cmd);                      	            
    for(i=0; i<num; i++)	
        val[i] = icm_simspi_wr_byte(0);                      	
    ICM20602_CSN (1);                                    		
}






//-------------------------------------------------------------------------------------------------------------------
//  @brief      将ICM20602数据转化为带有物理单位的数据
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:               执行该函数后，直接查看对应的变量即可
//-------------------------------------------------------------------------------------------------------------------
float unit_icm_gyro_x;
float unit_icm_gyro_y;
float unit_icm_gyro_z;

float unit_icm_acc_x;
float unit_icm_acc_y;
float unit_icm_acc_z;
void icm20602_data_change(void)
{
    //ICM20602_GYRO_CONFIG寄存器
    //设置为:0x00 陀螺仪量程为:±250 dps     获取到的陀螺仪数据除以131           可以转化为带物理单位的数据， 单位为：°/s
    //设置为:0x08 陀螺仪量程为:±500 dps     获取到的陀螺仪数据除以65.5          可以转化为带物理单位的数据，单位为：°/s
    //设置为:0x10 陀螺仪量程为:±1000dps     获取到的陀螺仪数据除以32.8          可以转化为带物理单位的数据，单位为：°/s
    //设置为:0x18 陀螺仪量程为:±2000dps     获取到的陀螺仪数据除以16.4          可以转化为带物理单位的数据，单位为：°/s

    //ICM20602_ACCEL_CONFIG寄存器
    //设置为:0x00 加速度计量程为:±2g          获取到的加速度计数据 除以16384      可以转化为带物理单位的数据，单位：g(m/s^2)
    //设置为:0x08 加速度计量程为:±4g          获取到的加速度计数据 除以8192       可以转化为带物理单位的数据，单位：g(m/s^2)
    //设置为:0x10 加速度计量程为:±8g          获取到的加速度计数据 除以4096       可以转化为带物理单位的数据，单位：g(m/s^2)
    //设置为:0x18 加速度计量程为:±16g         获取到的加速度计数据 除以2048       可以转化为带物理单位的数据，单位：g(m/s^2)
    unit_icm_gyro_x = (float)icm_gyro_x/131;
    unit_icm_gyro_y = (float)icm_gyro_y/131;
    unit_icm_gyro_z = (float)icm_gyro_z/131;

    unit_icm_acc_x = (float)icm_acc_x/4096;
    unit_icm_acc_y = (float)icm_acc_y/4096;
    unit_icm_acc_z = (float)icm_acc_z/4096;

}

/*
 * icm20602.c
 *
 *  Created on: 2022年5月3日
 *      Author: li
 */

#define eps 0.5
#define PI 3.1415926
#define  DELTA_T    0.01f  //5ms计算一次
//#define PI 3.1415926
float I_ex,I_ey,I_ez;   //误差积分

QUAETR_T    Q_info = {1,0,0};   //四元数
ANGLE_T     eulerAngle;         //欧拉角
ICM_T       icm_data;
GYRO_T      GyroOffset;

//BOOL GyroOffset_init = 0;

float Kp = 10;//加速度计收敛速率比例增益
float Ki = 0.01;//陀螺仪收敛速率的积分增益

//陀螺仪去零飘
void gyroOffset_init(void)
{
	int i = 0;
    GyroOffset.Xdata = 0;
    GyroOffset.Ydata = 0;
    GyroOffset.Zdata = 0;
    for (i = 0; i < 100; i ++)//采集100次数据
    {
				get_icm20602_accdata_simspi();
				get_icm20602_gyro_simspi();
        GyroOffset.Xdata += icm_gyro_x;
        GyroOffset.Ydata += icm_gyro_y;
        GyroOffset.Zdata += icm_gyro_z;
        HAL_Delay(2);
    }
    GyroOffset.Xdata /= 100;//计算平均偏差
    GyroOffset.Ydata /= 100;
    GyroOffset.Zdata /= 100;

//    GyroOffset_init = 1;
}
float myRsqrt(float x)
{
	float val=x;//初始值
	float last;
	do
	{
		last = val;
		val =(val + x/val) / 2;
	}while(fabs(val-last) > eps);
	return val; 
}
#define  ALPHA          0.3    //低通比例

//转化为实际物理量
void ICM_getCalues(void)
{
    //一阶低通
    icm_data.acc_x = ((((float) icm_acc_x) * ALPHA) / 4096 + icm_data.acc_x * (1 - ALPHA));
    icm_data.acc_y = ((((float) icm_acc_y) * ALPHA) / 4096 + icm_data.acc_y * (1 - ALPHA));
    icm_data.acc_z = ((((float) icm_acc_z) * ALPHA) / 4096 + icm_data.acc_z * (1 - ALPHA));

    //陀螺仪
    icm_data.gyro_x = (((float) icm_gyro_x - GyroOffset.Xdata)*ALPHA) * PI / 180 / 16.4f;
    icm_data.gyro_y = (((float) icm_gyro_y - GyroOffset.Ydata)*ALPHA) * PI / 180 / 16.4f;
    icm_data.gyro_z = (((float) icm_gyro_z - GyroOffset.Zdata)*ALPHA) * PI / 180 / 16.4f;
}
//互补滤波
void ICM_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
    float half_DELTA = 0.5*DELTA_T;
    float vx, vy, vz;
    float ex, ey, ez;//计算值与测量值的误差
    float q0 = Q_info.q0;//四元数初始化
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;
    float q0_2 = q0*q0;//平方
    float q1_2 = q1*q1;
    float q2_2 = q2*q2;
    float q3_2 = q3*q3;
    float q0_q1 = q0*q1;//互乘
    float q0_q2 = q0*q2;
    float q0_q3 = q0*q3;
    float q1_q2 = q1*q2;
    float q1_q3 = q1*q3;
    float q2_q3 = q2*q3;    
	
	
	
		if(icm_data.acc_x * icm_data.acc_y * icm_data.acc_z == 0) // 加计处于自由落体状态时(此时g = 0)不进行姿态解算，因为会产生分母无穷大的情况
        return;

		
    //归一化加速度数据
   float norm = fast_sqrt(ax*ax+ay*ay+az*az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

    //根据四元数的姿态值估算重力加速度分量，用于修正姿态
    vx = 2*(q1_q3 - q0_q2);
    vy = 2*(q0_q1 + q2_q3);
    vz = q0_2-q1_2-q2_2+q3_2;

    //计算估算与实际测量的误差
    ex = ay*vz - az*vy;
    ey = az*vx - ax*vz;
    ez = ax*vy - ay*vx;

    //矫正零偏
    I_ex += half_DELTA*ex;//误差积分
    I_ey += half_DELTA*ey;
    I_ez += half_DELTA*ez;

    gx = gx + Kp * ex + Ki * I_ex;
    gy = gy + Kp * ey + Ki * I_ey;
    gz = gz + Kp * ez + Ki * I_ez;

    //四元数微分方程
    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * half_DELTA;
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * half_DELTA;
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * half_DELTA;
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * half_DELTA;

    //更新数据
    norm = fast_sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);

    Q_info.q0 = q0*norm;
    Q_info.q1 = q1*norm;
    Q_info.q2 = q2*norm;
    Q_info.q3 = q3*norm;
}

//姿态解算
void ICM_getAngle(void)
{
    static float pitch_last;
//    static float roll_last;
//    static float yaw_last;
    float q0 =0;
    float q1 = 0;
    float q2 = 0;
    float q3 = 0;
    float k = 0.3;
//		uint16_t i=0;
//		uint16_t i2=0;
//		uint16_t i3=0;
//		float temp_pitch=0;
//		float temp_pitch_last=0;
//	  float last_yaw = 0;
	
		get_icm20602_accdata_simspi();
	  get_icm20602_gyro_simspi();

    ICM_getCalues();
    ICM_AHRSupdate(icm_data.gyro_x, icm_data.gyro_y, icm_data.gyro_z, icm_data.acc_x, icm_data.acc_y, icm_data.acc_z);
    q0 = Q_info.q0;
    q1 = Q_info.q1;
    q2 = Q_info.q2;
    q3 = Q_info.q3;

    //记录上一次的数据
    pitch_last = eulerAngle.pitch;
//    roll_last = eulerAngle.roll;
//    yaw_last = eulerAngle.yaw;
    //计算欧拉角
//    eulerAngle.pitch = asin(-2 * q1 * q3 + 2 * q0 * q2)*180 / PI;
//////		eulerAngle.roll = asin(2*(q0*q1 + q2*q3 )) * 57.2957795f; // 横滚
//    eulerAngle.roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1)*180 / PI;
//    eulerAngle.yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 180 / PI;
		
			eulerAngle.pitch = asin(-2 * q1 * q3 + 2 * q0 * q2)*180 / PI;
//			eulerAngle.roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1)*180 / PI;
//			eulerAngle.yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 180 / PI;


    //低通滤波
    eulerAngle.pitch = k*eulerAngle.pitch+(1-k)*pitch_last;
		kalmanFilter(&KFP_gyro2,(float)eulerAngle.pitch);
		eulerAngle.pitch=KFP_gyro2.out;
//    eulerAngle.roll = k*eulerAngle.roll+(1-k)*roll_last;
//    eulerAngle.yaw = k*eulerAngle.yaw+(1-k)*yaw_last;
////		if(yaw_last>179)
////		{
//////			if(eulerAngle.yaw<0)
//////				i2=1;
////		}
////		if(i2==1)
////		{eulerAngle.yaw=eulerAngle.yaw+180;}
//		if(yaw_last<-179)
//		{
//			if(eulerAngle.yaw>0)
//				i3++;
//		}
		
		
//          if(ABS(eulerAngle.yaw - yaw_last) >= 2.0 * ABS(eulerAngle_static.final_yaw - eulerAngle_static.final_yaw_last))
//          {
//            Reduction_yaw += (eulerAngle.final_yaw - eulerAngle.final_yaw_last) - (eulerAngle_static.final_yaw - eulerAngle_static.final_yaw_last);
//          }
		
		
		
		
		
//		PRINTF("%.2f,%.2f,%.2f\n",eulerAngle.pitch,eulerAngle.roll,eulerAngle.yaw);
		
//			if (eulerAngle.roll > 90 || eulerAngle.roll < -90) {
//        if (eulerAngle.pitch > 0) {
//            eulerAngle.pitch = 180 - eulerAngle.pitch;
//        }
//        if (eulerAngle.pitch < 0) {
//            eulerAngle.pitch = -(180 + eulerAngle.pitch);
//        }
//    }
//		temp_pitch = eulerAngle.yaw;
//		i++;
//		if(i%10==0)temp_pitch_last = eulerAngle.yaw;
//		if(i==50)i=0;
//		if((temp_pitch-temp_pitch_last)>350)
//		{
//			eulerAngle.yaw=eulerAngle.yaw+360;
//		}
//		if((temp_pitch-temp_pitch_last)<-350)
//		{
//			eulerAngle.yaw=eulerAngle.yaw-360;
//		}
//    if (eulerAngle.yaw > 360) {
//        eulerAngle.yaw -= 360;
//    } 
//			if (eulerAngle.yaw < 0) {
//        eulerAngle.yaw += 360;
//    }
		
}

//快速开根号
float fast_sqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *) &y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *) &i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
double my_asin(double x)
{
   return atan2 (x, sqrt (1.0 - x * x));
}
float invSqrt(float x)	/*快速开平方求倒*/
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

