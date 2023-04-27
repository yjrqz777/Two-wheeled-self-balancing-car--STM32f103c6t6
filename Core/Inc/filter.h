#ifndef __FILTER_H
#define __FILTER_H
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
extern float angle2, angle_dot; 	
void Kalman_Filter(float Accel,float Gyro);		
void Yijielvbo(float angle_m, float gyro_m);

//1. 结构体类型定义
typedef struct 
{
    float LastP;//上次估算协方差 初始化值为0.02
    float Now_P;//当前估算协方差 初始化值为0
    float out;//卡尔曼滤波器输出 初始化值为0
    float Kg;//卡尔曼增益 初始化值为0
    float Q;//过程噪声协方差 初始化值为0.001
    float R;//观测噪声协方差 初始化值为0.543
}KFP;//Kalman Filter parameter



//2. 以高度为例 定义卡尔曼结构体并初始化参数
extern KFP KFP_gyro;
extern KFP KFP_gyro_x;
extern KFP KFP_gyro_y;
extern KFP KFP_gyro_z;

extern KFP KFP_acc_x;
extern KFP KFP_acc_y;
extern KFP KFP_acc_z;

extern KFP KFP_gyro2;
extern KFP KFP_gyro22;
extern float kalmanFilter(KFP *kfp,float input);

float get_em_fitter_data(void);
float filter9( void );
#endif
