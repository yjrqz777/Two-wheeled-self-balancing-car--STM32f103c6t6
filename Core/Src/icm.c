/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		ICM20602
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		IAR 8.3 or MDK 5.26
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2018-05-24
 * @note		
					���߶��壺
					------------------------------------ 
						SCL                 �鿴SEEKFREE_IIC�ļ��ڵ�SEEKFREE_SCL�궨��
						SDA                 �鿴SEEKFREE_IIC�ļ��ڵ�SEEKFREE_SDA�궨��
					------------------------------------ 
 ********************************************************************************************************************/

#include "stm32f1xx_hal.h"
#include "icm.h"
#include <math.h>
#include "filter.h"
int16_t icm_gyro_x,icm_gyro_y,icm_gyro_z;
int16_t icm_acc_x,icm_acc_y,icm_acc_z;

int16_t kicm_gyro_x,kicm_gyro_y,kicm_gyro_z;
int16_t kicm_acc_x,kicm_acc_y,kicm_acc_z;

//-------------------------------------------------------------------------------------------------------------------
//  ���º�����ʹ�����SPIͨ�ţ���Ƚ�Ӳ��SPI�����SPI���Ÿ���������ʹ��������ͨIO
//-------------------------------------------------------------------------------------------------------------------


#define ICM20602_SCK(x)		HAL_GPIO_WritePin(ICM20602_SCK_GPIO_Port,ICM20602_SCK_Pin,(GPIO_PinState)x)
#define ICM20602_MOSI(x) 	HAL_GPIO_WritePin(ICM20602_MOSI_GPIO_Port,ICM20602_MOSI_Pin,(GPIO_PinState)x)
#define ICM20602_CSN(x)  	HAL_GPIO_WritePin(ICM20602_CS_GPIO_Port,ICM20602_CS_Pin,(GPIO_PinState)x)
#define ICM20602_MISO    	(uint8_t)HAL_GPIO_ReadPin(ICM20602_MISO_GPIO_Port,ICM20602_MISO_Pin)

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ICM20602�Լ캯��
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
        //��������ԭ�������¼���
        //1 MPU6050���ˣ�������µ������ĸ��ʼ���
        //2 ���ߴ������û�нӺ�
        //3 ��������Ҫ����������裬������3.3V
    }while(0x12 != val);
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ���SPI����ʼ��ICM20602
//  @param      NULL
//  @return     void					
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void icm20602_init_simspi(void)
{
    uint8_t val;
    

    icm20602_self4_check();//���
    
    icm_simspi_w_reg_byte(ICM20602_PWR_MGMT_1,0x80);//��λ�豸
    HAL_Delay(2);
    do
    {//�ȴ���λ�ɹ�
        icm_simspi_r_reg_byte(ICM20602_PWR_MGMT_1,&val);
    }while(0x41 != val);
    
    icm_simspi_w_reg_byte(ICM20602_PWR_MGMT_1,     0x01);            //ʱ������
    icm_simspi_w_reg_byte(ICM20602_PWR_MGMT_2,     0x00);            //���������Ǻͼ��ٶȼ�
    icm_simspi_w_reg_byte(ICM20602_CONFIG,         0x01);            //176HZ 1KHZ
    icm_simspi_w_reg_byte(ICM20602_SMPLRT_DIV,     0x07);            //�������� SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
    icm_simspi_w_reg_byte(ICM20602_GYRO_CONFIG,    0x18);            //��2000 dps
    icm_simspi_w_reg_byte(ICM20602_ACCEL_CONFIG,   0x10);            //��8g
    icm_simspi_w_reg_byte(ICM20602_ACCEL_CONFIG_2, 0x03);            //Average 8 samples   44.8HZ
	//ICM20602_GYRO_CONFIG�Ĵ���
    //����Ϊ:0x00 ����������Ϊ:��250 dps     ��ȡ�������������ݳ���131           ����ת��Ϊ������λ�����ݣ� ��λΪ����/s
    //����Ϊ:0x08 ����������Ϊ:��500 dps     ��ȡ�������������ݳ���65.5          ����ת��Ϊ������λ�����ݣ���λΪ����/s
    //����Ϊ:0x10 ����������Ϊ:��1000dps     ��ȡ�������������ݳ���32.8          ����ת��Ϊ������λ�����ݣ���λΪ����/s
    //����Ϊ:0x18 ����������Ϊ:��2000dps     ��ȡ�������������ݳ���16.4          ����ת��Ϊ������λ�����ݣ���λΪ����/s

    //ICM20602_ACCEL_CONFIG�Ĵ���
    //����Ϊ:0x00 ���ٶȼ�����Ϊ:��2g          ��ȡ���ļ��ٶȼ����� ����16384      ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    //����Ϊ:0x08 ���ٶȼ�����Ϊ:��4g          ��ȡ���ļ��ٶȼ����� ����8192       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    //����Ϊ:0x10 ���ٶȼ�����Ϊ:��8g          ��ȡ���ļ��ٶȼ����� ����4096       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    //����Ϊ:0x18 ���ٶȼ�����Ϊ:��16g         ��ȡ���ļ��ٶȼ����� ����2048       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡICM20602���ٶȼ�����
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
//-------------------------------------------------------------------------------------------------------------------
void get_icm20602_accdata_simspi(void)
{
    uint8_t dat[6];
    
    icm_simspi_r_reg_bytes(ICM20602_ACCEL_XOUT_H, dat, 6);
    icm_acc_x = (int16_t)(((uint16_t)dat[0]<<8 | dat[1]));
    icm_acc_y = (int16_t)(((uint16_t)dat[2]<<8 | dat[3]));
    icm_acc_z = (int16_t)(((uint16_t)dat[4]<<8 | dat[5]));
	
		kalmanFilter(&KFP_acc_x,icm_acc_x);
		kalmanFilter(&KFP_acc_y,icm_acc_y);
		kalmanFilter(&KFP_acc_z,icm_acc_z);
	
//		icm_acc_x=KFP_acc_x.out;
//		icm_acc_y=KFP_acc_y.out;
//		icm_acc_z=KFP_acc_z.out;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡICM20602����������
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
//-------------------------------------------------------------------------------------------------------------------
void get_icm20602_gyro_simspi(void)
{
    uint8_t dat[6];
    
    icm_simspi_r_reg_bytes(ICM20602_GYRO_XOUT_H, dat, 6);
    icm_gyro_x = (int16_t)(((uint16_t)dat[0]<<8 | dat[1]));
    icm_gyro_y = (int16_t)(((uint16_t)dat[2]<<8 | dat[3]));
    icm_gyro_z = (int16_t)(((uint16_t)dat[4]<<8 | dat[5]));
	
	
		kalmanFilter(&KFP_gyro_x,icm_gyro_x);
		kalmanFilter(&KFP_gyro_y,icm_gyro_y);
		kalmanFilter(&KFP_gyro_z,icm_gyro_z);
	
		icm_gyro_x=KFP_gyro_x.out;
		icm_gyro_y=KFP_gyro_y.out;
		icm_gyro_z=KFP_gyro_z.out;
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      ͨ��SPIдһ��byte,ͬʱ��ȡһ��byte
//  @param      byte        ���͵�����    
//  @return     uint8_t       return ����status״̬
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
//  @brief      ��valд��cmd��Ӧ�ļĴ�����ַ,ͬʱ����status�ֽ�
//  @param      cmd         ������
//  @param      val         ��д��Ĵ�������ֵ
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
//  @brief      ��ȡcmd����Ӧ�ļĴ�����ַ
//  @param      cmd         ������
//  @param      *val        �洢��ȡ�����ݵ�ַ
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
//  @brief      ��ȡcmd����Ӧ�ļĴ�����ַ
//  @param      cmd         ������
//  @param      *val        �洢��ȡ�����ݵ�ַ
//  @param      num         ��ȡ������
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
//  @brief      ��ICM20602����ת��Ϊ��������λ������
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:               ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
//-------------------------------------------------------------------------------------------------------------------
float unit_icm_gyro_x;
float unit_icm_gyro_y;
float unit_icm_gyro_z;

float unit_icm_acc_x;
float unit_icm_acc_y;
float unit_icm_acc_z;
void icm20602_data_change(void)
{
    //ICM20602_GYRO_CONFIG�Ĵ���
    //����Ϊ:0x00 ����������Ϊ:��250 dps     ��ȡ�������������ݳ���131           ����ת��Ϊ������λ�����ݣ� ��λΪ����/s
    //����Ϊ:0x08 ����������Ϊ:��500 dps     ��ȡ�������������ݳ���65.5          ����ת��Ϊ������λ�����ݣ���λΪ����/s
    //����Ϊ:0x10 ����������Ϊ:��1000dps     ��ȡ�������������ݳ���32.8          ����ת��Ϊ������λ�����ݣ���λΪ����/s
    //����Ϊ:0x18 ����������Ϊ:��2000dps     ��ȡ�������������ݳ���16.4          ����ת��Ϊ������λ�����ݣ���λΪ����/s

    //ICM20602_ACCEL_CONFIG�Ĵ���
    //����Ϊ:0x00 ���ٶȼ�����Ϊ:��2g          ��ȡ���ļ��ٶȼ����� ����16384      ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    //����Ϊ:0x08 ���ٶȼ�����Ϊ:��4g          ��ȡ���ļ��ٶȼ����� ����8192       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    //����Ϊ:0x10 ���ٶȼ�����Ϊ:��8g          ��ȡ���ļ��ٶȼ����� ����4096       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    //����Ϊ:0x18 ���ٶȼ�����Ϊ:��16g         ��ȡ���ļ��ٶȼ����� ����2048       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
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
 *  Created on: 2022��5��3��
 *      Author: li
 */

#define eps 0.5
#define PI 3.1415926
#define  DELTA_T    0.01f  //5ms����һ��
//#define PI 3.1415926
float I_ex,I_ey,I_ez;   //������

QUAETR_T    Q_info = {1,0,0};   //��Ԫ��
ANGLE_T     eulerAngle;         //ŷ����
ICM_T       icm_data;
GYRO_T      GyroOffset;

//BOOL GyroOffset_init = 0;

float Kp = 10;//���ٶȼ��������ʱ�������
float Ki = 0.01;//�������������ʵĻ�������

//������ȥ��Ʈ
void gyroOffset_init(void)
{
	int i = 0;
    GyroOffset.Xdata = 0;
    GyroOffset.Ydata = 0;
    GyroOffset.Zdata = 0;
    for (i = 0; i < 100; i ++)//�ɼ�100������
    {
				get_icm20602_accdata_simspi();
				get_icm20602_gyro_simspi();
        GyroOffset.Xdata += icm_gyro_x;
        GyroOffset.Ydata += icm_gyro_y;
        GyroOffset.Zdata += icm_gyro_z;
        HAL_Delay(2);
    }
    GyroOffset.Xdata /= 100;//����ƽ��ƫ��
    GyroOffset.Ydata /= 100;
    GyroOffset.Zdata /= 100;

//    GyroOffset_init = 1;
}
float myRsqrt(float x)
{
	float val=x;//��ʼֵ
	float last;
	do
	{
		last = val;
		val =(val + x/val) / 2;
	}while(fabs(val-last) > eps);
	return val; 
}
#define  ALPHA          0.3    //��ͨ����

//ת��Ϊʵ��������
void ICM_getCalues(void)
{
    //һ�׵�ͨ
    icm_data.acc_x = ((((float) icm_acc_x) * ALPHA) / 4096 + icm_data.acc_x * (1 - ALPHA));
    icm_data.acc_y = ((((float) icm_acc_y) * ALPHA) / 4096 + icm_data.acc_y * (1 - ALPHA));
    icm_data.acc_z = ((((float) icm_acc_z) * ALPHA) / 4096 + icm_data.acc_z * (1 - ALPHA));

    //������
    icm_data.gyro_x = (((float) icm_gyro_x - GyroOffset.Xdata)*ALPHA) * PI / 180 / 16.4f;
    icm_data.gyro_y = (((float) icm_gyro_y - GyroOffset.Ydata)*ALPHA) * PI / 180 / 16.4f;
    icm_data.gyro_z = (((float) icm_gyro_z - GyroOffset.Zdata)*ALPHA) * PI / 180 / 16.4f;
}
//�����˲�
void ICM_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
    float half_DELTA = 0.5*DELTA_T;
    float vx, vy, vz;
    float ex, ey, ez;//����ֵ�����ֵ�����
    float q0 = Q_info.q0;//��Ԫ����ʼ��
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;
    float q0_2 = q0*q0;//ƽ��
    float q1_2 = q1*q1;
    float q2_2 = q2*q2;
    float q3_2 = q3*q3;
    float q0_q1 = q0*q1;//����
    float q0_q2 = q0*q2;
    float q0_q3 = q0*q3;
    float q1_q2 = q1*q2;
    float q1_q3 = q1*q3;
    float q2_q3 = q2*q3;    
	
	
	
		if(icm_data.acc_x * icm_data.acc_y * icm_data.acc_z == 0) // �Ӽƴ�����������״̬ʱ(��ʱg = 0)��������̬���㣬��Ϊ�������ĸ���������
        return;

		
    //��һ�����ٶ�����
   float norm = fast_sqrt(ax*ax+ay*ay+az*az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

    //������Ԫ������ֵ̬�����������ٶȷ���������������̬
    vx = 2*(q1_q3 - q0_q2);
    vy = 2*(q0_q1 + q2_q3);
    vz = q0_2-q1_2-q2_2+q3_2;

    //���������ʵ�ʲ��������
    ex = ay*vz - az*vy;
    ey = az*vx - ax*vz;
    ez = ax*vy - ay*vx;

    //������ƫ
    I_ex += half_DELTA*ex;//������
    I_ey += half_DELTA*ey;
    I_ez += half_DELTA*ez;

    gx = gx + Kp * ex + Ki * I_ex;
    gy = gy + Kp * ey + Ki * I_ey;
    gz = gz + Kp * ez + Ki * I_ez;

    //��Ԫ��΢�ַ���
    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * half_DELTA;
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * half_DELTA;
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * half_DELTA;
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * half_DELTA;

    //��������
    norm = fast_sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);

    Q_info.q0 = q0*norm;
    Q_info.q1 = q1*norm;
    Q_info.q2 = q2*norm;
    Q_info.q3 = q3*norm;
}

//��̬����
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

    //��¼��һ�ε�����
    pitch_last = eulerAngle.pitch;
//    roll_last = eulerAngle.roll;
//    yaw_last = eulerAngle.yaw;
    //����ŷ����
//    eulerAngle.pitch = asin(-2 * q1 * q3 + 2 * q0 * q2)*180 / PI;
//////		eulerAngle.roll = asin(2*(q0*q1 + q2*q3 )) * 57.2957795f; // ���
//    eulerAngle.roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1)*180 / PI;
//    eulerAngle.yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 180 / PI;
		
			eulerAngle.pitch = asin(-2 * q1 * q3 + 2 * q0 * q2)*180 / PI;
////			eulerAngle.roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1)*180 / PI;
//			eulerAngle.yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 180 / PI;


    //��ͨ�˲�
//    eulerAngle.pitch = k*eulerAngle.pitch+(1-k)*pitch_last;
//		kalmanFilter(&KFP_gyro2,(float)eulerAngle.pitch);
//		eulerAngle.pitch=KFP_gyro2.out;
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

//���ٿ�����
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
float invSqrt(float x)	/*���ٿ�ƽ����*/
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

