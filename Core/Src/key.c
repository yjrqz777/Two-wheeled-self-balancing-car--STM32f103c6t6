

#include "key.h"
//按键7.0,7.1,7.2,7.3
float key1_num=0;
float key2_num=0;

//开关状态变量
unsigned int key1_status = 1;
unsigned int key2_status = 1;


//上一次开关状态变量
unsigned int key1_last_status;
unsigned int key2_last_status;


//开关标志位
unsigned int key1_flag;
unsigned int key2_flag;




 void contorl_PID(void)
{
        
        //使用此方法优点在于，不需要使用while(1) 等待，避免处理器资源浪费
        //保存按键状态
        key1_last_status = key1_status;
        key2_last_status = key2_status;

        //读取当前按键状态
        key1_status = KEY1_PIN;
        key2_status = KEY2_PIN;

	

        if(key1_status && !key1_last_status)    key1_flag = 1;
        if(key2_status && !key2_last_status)    key2_flag = 1;

        
        //标志位置位之后，可以使用标志位执行自己想要做的事件
        if(key1_flag)   
        {
          key1_flag = 0;//使用按键之后，应该清除标志位
					key1_num=key1_num+0.5;
        }
        
        if(key2_flag)   
        {
            key2_flag = 0;//使用按键之后，应该清除标志位
					key1_num=key1_num-0.5;
        }

}
				
