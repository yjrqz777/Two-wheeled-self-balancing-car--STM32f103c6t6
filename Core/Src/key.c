

#include "key.h"
//����7.0,7.1,7.2,7.3
float key1_num=0;
float key2_num=0;

//����״̬����
unsigned int key1_status = 1;
unsigned int key2_status = 1;


//��һ�ο���״̬����
unsigned int key1_last_status;
unsigned int key2_last_status;


//���ر�־λ
unsigned int key1_flag;
unsigned int key2_flag;




 void contorl_PID(void)
{
        
        //ʹ�ô˷����ŵ����ڣ�����Ҫʹ��while(1) �ȴ������⴦������Դ�˷�
        //���水��״̬
        key1_last_status = key1_status;
        key2_last_status = key2_status;

        //��ȡ��ǰ����״̬
        key1_status = KEY1_PIN;
        key2_status = KEY2_PIN;

	

        if(key1_status && !key1_last_status)    key1_flag = 1;
        if(key2_status && !key2_last_status)    key2_flag = 1;

        
        //��־λ��λ֮�󣬿���ʹ�ñ�־λִ���Լ���Ҫ�����¼�
        if(key1_flag)   
        {
          key1_flag = 0;//ʹ�ð���֮��Ӧ�������־λ
					key1_num=key1_num+0.5;
        }
        
        if(key2_flag)   
        {
            key2_flag = 0;//ʹ�ð���֮��Ӧ�������־λ
					key1_num=key1_num-0.5;
        }

}
				
