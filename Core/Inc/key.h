


#ifndef _KEY_H
#define _KEY_H



//extern "C"
//{
//	
#include "stm32f1xx_hal.h"
//	
//}  
// stm32f103x6.h
/*
 *�����ں�Ƶ�ʵ��趨�����Բ鿴board.h�ļ�
 *��board_init��,�Ѿ���P54��������Ϊ��λ
 *�����Ҫʹ��P54����,������board.c�ļ��е�board_init()������ɾ��SET_P54_RESRT����
 */

//���尴������
#define KEY1_PIN    HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_15);
#define KEY2_PIN    HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_14);

extern float key1_num;
extern float key2_num;

//����״̬����
extern unsigned int key1_status;
extern unsigned int key2_status;


//��һ�ο���״̬����
extern unsigned int key1_last_status;
extern unsigned int key2_last_status;

//���ر�־λ
extern unsigned int key1_flag;
extern unsigned int key2_flag;



//board.h�ļ���FOSC��ֵ����Ϊ0,������Զ�ʶ��ϵͳƵ��

/*board.h�ļ���FOSC��ֵ���ò�Ϊ0����ϵͳƵ��ΪFOSC��ֵ��
��ʹ��stc-isp�������س����ʱ����Ҫ��IRCƵ������ΪFOSC��ֵ*/

/*��board_init��,�Ѿ���P54��������Ϊ��λ��
�����Ҫʹ��P54����,������board.c�ļ��е�board_init()������ɾ��SET_P54_RESRT����*/




void contorl_PID(void);

#endif
