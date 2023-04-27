
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
#include "main.h"
//#include "stm32f1xx_hal.h"

#ifndef _SEEKFREE_ICM20602_h
#define _SEEKFREE_ICM20602_h



#define     ICM20602_DEV_ADDR           0x69 //SA0�ӵأ�0x68   SA0������0x69  ģ��Ĭ������


#define     ICM20602_SPI_W              0x00
#define     ICM20602_SPI_R              0x80


#define     ICM20602_XG_OFFS_TC_H       0x04
#define     ICM20602_XG_OFFS_TC_L       0x05
#define     ICM20602_YG_OFFS_TC_H       0x07
#define     ICM20602_YG_OFFS_TC_L       0x08
#define     ICM20602_ZG_OFFS_TC_H       0x0A
#define     ICM20602_ZG_OFFS_TC_L       0x0B
#define     ICM20602_SELF_TEST_X_ACCEL  0x0D
#define     ICM20602_SELF_TEST_Y_ACCEL  0x0E
#define     ICM20602_SELF_TEST_Z_ACCEL  0x0F
#define     ICM20602_XG_OFFS_USRH       0x13
#define     ICM20602_XG_OFFS_USRL       0x14
#define     ICM20602_YG_OFFS_USRH       0x15
#define     ICM20602_YG_OFFS_USRL       0x16
#define     ICM20602_ZG_OFFS_USRH       0x17
#define     ICM20602_ZG_OFFS_USRL       0x18
#define     ICM20602_SMPLRT_DIV         0x19
#define     ICM20602_CONFIG             0x1A
#define     ICM20602_GYRO_CONFIG        0x1B
#define     ICM20602_ACCEL_CONFIG       0x1C
#define     ICM20602_ACCEL_CONFIG_2     0x1D
#define     ICM20602_LP_MODE_CFG        0x1E
#define     ICM20602_ACCEL_WOM_X_THR    0x20
#define     ICM20602_ACCEL_WOM_Y_THR    0x21
#define     ICM20602_ACCEL_WOM_Z_THR    0x22
#define     ICM20602_FIFO_EN            0x23
#define     ICM20602_FSYNC_INT          0x36
#define     ICM20602_INT_PIN_CFG        0x37
#define     ICM20602_INT_ENABLE         0x38
#define     ICM20602_FIFO_WM_INT_STATUS 0x39 
#define     ICM20602_INT_STATUS         0x3A
#define     ICM20602_ACCEL_XOUT_H       0x3B
#define     ICM20602_ACCEL_XOUT_L       0x3C
#define     ICM20602_ACCEL_YOUT_H       0x3D
#define     ICM20602_ACCEL_YOUT_L       0x3E
#define     ICM20602_ACCEL_ZOUT_H       0x3F
#define     ICM20602_ACCEL_ZOUT_L       0x40
#define     ICM20602_TEMP_OUT_H         0x41
#define     ICM20602_TEMP_OUT_L         0x42
#define     ICM20602_GYRO_XOUT_H        0x43
#define     ICM20602_GYRO_XOUT_L        0x44
#define     ICM20602_GYRO_YOUT_H        0x45
#define     ICM20602_GYRO_YOUT_L        0x46
#define     ICM20602_GYRO_ZOUT_H        0x47
#define     ICM20602_GYRO_ZOUT_L        0x48
#define     ICM20602_SELF_TEST_X_GYRO   0x50
#define     ICM20602_SELF_TEST_Y_GYRO   0x51
#define     ICM20602_SELF_TEST_Z_GYRO   0x52
#define     ICM20602_FIFO_WM_TH1        0x60
#define     ICM20602_FIFO_WM_TH2        0x61
#define     ICM20602_SIGNAL_PATH_RESET  0x68
#define     ICM20602_ACCEL_INTEL_CTRL   0x69
#define     ICM20602_USER_CTRL          0x6A
#define     ICM20602_PWR_MGMT_1         0x6B
#define     ICM20602_PWR_MGMT_2         0x6C
#define     ICM20602_I2C_IF             0x70
#define     ICM20602_FIFO_COUNTH        0x72
#define     ICM20602_FIFO_COUNTL        0x73
#define     ICM20602_FIFO_R_W           0x74
#define     ICM20602_WHO_AM_I           0x75
#define     ICM20602_XA_OFFSET_H        0x77
#define     ICM20602_XA_OFFSET_L        0x78
#define     ICM20602_YA_OFFSET_H        0x7A
#define     ICM20602_YA_OFFSET_L        0x7B
#define     ICM20602_ZA_OFFSET_H        0x7D
#define     ICM20602_ZA_OFFSET_L        0x7E

/*
 * icm20602.h
 *
 *  Created on: 2022��5��3��
 *      Author: li
 */


extern int16_t acc[3];
extern int16_t gyro[3];
extern float angle[3];

typedef struct{
//������
    float gyro_x;
    float gyro_y;
    float gyro_z;
//���ٶȼ�
    float acc_x;
    float acc_y;
    float acc_z;
}ICM_T;

typedef struct{
//��Ԫ��
    float q0;
    float q1;
    float q2;
    float q3;
}QUAETR_T;

typedef struct{
    float pitch;    //������
    float roll;     //ƫ����
    float yaw;       //������
}ANGLE_T;

typedef struct{
//��������Ʈ
    float Xdata;
    float Ydata;
    float Zdata;
}GYRO_T;

extern ANGLE_T eulerAngle;
extern ICM_T       icm_data;

void gyroOffset_init(void);
void ICM_getCalues(void);
void ICM_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az);
void ICM_getAngle(void);
float fast_sqrt(float x);
float myRsqrt(float num);
double my_asin(double x);



extern int16_t icm_gyro_x,icm_gyro_y,icm_gyro_z;
extern int16_t icm_acc_x,icm_acc_y,icm_acc_z;
extern int16_t kicm_gyro_x,kicm_gyro_y,kicm_gyro_z;
extern int16_t kicm_acc_x,kicm_acc_y,kicm_acc_z;



//--------���SPI--------------
void icm20602_init_simspi(void);
void get_icm20602_accdata_simspi(void);
void get_icm20602_gyro_simspi(void);
uint8_t icm_simspi_wr_byte(uint8_t byte);

void icm_simspi_w_reg_byte(uint8_t cmd, uint8_t val);
void icm_simspi_r_reg_byte(uint8_t cmd, uint8_t *val);
void icm_simspi_r_reg_bytes(uint8_t cmd, uint8_t *val, uint8_t num);


float invSqrt(float x);




#endif
