


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
 *关于内核频率的设定，可以查看board.h文件
 *在board_init中,已经将P54引脚设置为复位
 *如果需要使用P54引脚,可以在board.c文件中的board_init()函数中删除SET_P54_RESRT即可
 */

//定义按键引脚
#define KEY1_PIN    HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_15);
#define KEY2_PIN    HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_14);

extern float key1_num;
extern float key2_num;

//开关状态变量
extern unsigned int key1_status;
extern unsigned int key2_status;


//上一次开关状态变量
extern unsigned int key1_last_status;
extern unsigned int key2_last_status;

//开关标志位
extern unsigned int key1_flag;
extern unsigned int key2_flag;



//board.h文件中FOSC的值设置为0,则程序自动识别系统频率

/*board.h文件中FOSC的值设置不为0，则系统频率为FOSC的值，
在使用stc-isp工具下载程序的时候需要将IRC频率设置为FOSC的值*/

/*在board_init中,已经将P54引脚设置为复位，
如果需要使用P54引脚,可以在board.c文件中的board_init()函数中删除SET_P54_RESRT即可*/




void contorl_PID(void);

#endif
