#include "filter.h"
#include "stm32f1xx_hal.h"
#include "icm.h"
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
float K1 =0.02; 
float angle2, angle_dot; 	
float Q_angle=0.001;// ����������Э����
float Q_gyro=0.003;//0.003 ����������Э���� ����������Э����Ϊһ��һ�����о���
float R_angle=0.5;// ����������Э���� �Ȳ���ƫ��
float dt=0.005;//                 
char  C_0 = 1;
float Q_bias, Angle_err;
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;
float Pdot[4] ={0,0,0,0};
float PP[2][2] = { { 1, 0 },{ 0, 1 } };


KFP KFP_gyro={0.02,0,0,0,0.001,0.543};


KFP KFP_gyro_x={0.02,0,0,0,0.4,1.5};
KFP KFP_gyro_y={0.02,0,0,0,0.4,1.5};
KFP KFP_gyro_z={0.02,0,0,0,0.4,1.5};

KFP KFP_acc_x={0.02,0,0,0,0.4,1.5};
KFP KFP_acc_y={0.02,0,0,0,0.4,1.5};
KFP KFP_acc_z={0.02,0,0,0,0.4,1.5};


KFP KFP_gyro2={0.02,0,0,0,0.001,0.543};


KFP KFP_gyro22={0.02,0,0,0,0.001,0.543};
/**************************************************************************
�������ܣ����׿������˲�
��ڲ��������ٶȡ����ٶ�
����  ֵ����
**************************************************************************/
void Kalman_Filter(float Accel,float Gyro)		
{
	angle2+=(Gyro - Q_bias) * dt; //�������
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-����������Э�����΢��

	Pdot[1]=-PP[1][1];
	Pdot[2]=-PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-����������Э����΢�ֵĻ���
	PP[0][1] += Pdot[1] * dt;   // =����������Э����
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - angle2;	//zk-�������
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //����������Э����
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	angle2	+= K_0 * Angle_err;	 //�������
	Q_bias	+= K_1 * Angle_err;	 //�������
	angle_dot   = Gyro - Q_bias; //���ֵ(�������)��΢��=���ٶ�
}

/**************************************************************************
�������ܣ�һ�׻����˲�
��ڲ��������ٶȡ����ٶ�
����  ֵ����
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
 *�������˲���
 *@param KFP *kfp �������ṹ�����
 *   float input ��Ҫ�˲��Ĳ����Ĳ���ֵ�����������Ĳɼ�ֵ��
 *@return �˲���Ĳ���������ֵ��
 */
 float kalmanFilter(KFP *kfp,float input)
 {
     //Ԥ��Э����̣�kʱ��ϵͳ����Э���� = k-1ʱ�̵�ϵͳЭ���� + ��������Э����
     kfp->Now_P = kfp->LastP + kfp->Q;
     //���������淽�̣����������� = kʱ��ϵͳ����Э���� / ��kʱ��ϵͳ����Э���� + �۲�����Э���
     kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
     //��������ֵ���̣�kʱ��״̬����������ֵ = ״̬������Ԥ��ֵ + ���������� * ������ֵ - ״̬������Ԥ��ֵ��
     kfp->out = kfp->out + kfp->Kg * (input -kfp->out);//��Ϊ��һ�ε�Ԥ��ֵ������һ�ε����ֵ
     //����Э�����: ���ε�ϵͳЭ����� kfp->LastP ����һ������׼����
     kfp->LastP = (1-kfp->Kg) * kfp->Now_P;
     return kfp->out;
 }



//kalmanFilter(&KFP_height,(float)height);

