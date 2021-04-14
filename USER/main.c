#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "usart1.h"
#include "usart3.h"

// �����շ����� typeΪ1,����1��,��������3  type��Ϊ1,����3,��������1
void usart_recv_send(u8 type);

int main(void)
{
    vu8 key = 0;
    delay_init();												 //��ʱ������ʼ��
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
    usart1_init(115200);									 // USART1��ʼ�� ������115200 Ĭ������λ8 ֹͣλ1 У��λnone
    usart3_init(115200);									 // USART3��ʼ�� ������115200 Ĭ������λ8 ֹͣλ1 У��λnone
    LED_Init();												 //LED�˿ڳ�ʼ��
    KEY_Init();										  	 	 //��ʼ���밴�����ӵ�Ӳ���ӿ�
    while (1)
    {
        // �����շ�
        usart_recv_send(1);
        usart_recv_send(3);

        // �õ���ֵ
        key = KEY_Scan(0);
        if (key)
        {
            switch (key)
            {
                case KEY1_PRES: // �򴮿�1����'1'
                    usart1_send_byte(0x31);
                    break;
                case KEY0_PRES: // �򴮿�3����'3'
                    usart3_send_byte(0x33);
                    break;
            }
        }

        LED0 = !LED0; //��˸LED,��ʾϵͳ��������.
        delay_ms(100);
    }
}

// �����շ����� typeΪ1,����1��,��������3  type��Ϊ1,����3,��������1
void usart_recv_send(u8 type)
{
    u8 i = 0;
    u8 tmp_len = 0;
    // ���ݻ���
    static u8 buf[255] = {0};
    // ���ݳ���
    u8 buf_len = 0;

    // ���ػ��������ݵĸ���
    if(1 == type)
        tmp_len = usart1_getdata_count();
    else
        tmp_len = usart3_getdata_count();

    for(i=0; i<tmp_len; i++)
    {
        // ���ػ�������ǰָ����ָ����
        if(1 == type)
            buf[i] = usart1_receive_data();
        else
            buf[i] = usart3_receive_data();

        buf_len++;
        // ����Լ�������޳���
        if(buf_len >= 250)
        {
            buf_len=0;
            break;
        }
    }

    // ���ݲ�Ϊ��
    if(0 != buf_len)
    {
        // �������ݷ���
        if(1 == type)
            usart3_send_bytes(buf, buf_len);
        else
            usart1_send_bytes(buf, buf_len);
    }
}
