#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "usart1.h"
#include "usart3.h"

// 串口收发函数 type为1,串口1收,发往串口3  type不为1,串口3,发往串口1
void usart_recv_send(u8 type);

int main(void)
{
    vu8 key = 0;
    delay_init();												 //延时函数初始化
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
    usart1_init(115200);									 // USART1初始化 波特率115200 默认数据位8 停止位1 校验位none
    usart3_init(115200);									 // USART3初始化 波特率115200 默认数据位8 停止位1 校验位none
    LED_Init();												 //LED端口初始化
    KEY_Init();										  	 	 //初始化与按键连接的硬件接口
    while (1)
    {
        // 串口收发
        usart_recv_send(1);
        usart_recv_send(3);

        // 得到键值
        key = KEY_Scan(0);
        if (key)
        {
            switch (key)
            {
                case KEY1_PRES: // 向串口1发送'1'
                    usart1_send_byte(0x31);
                    break;
                case KEY0_PRES: // 向串口3发送'3'
                    usart3_send_byte(0x33);
                    break;
            }
        }

        LED0 = !LED0; //闪烁LED,提示系统正在运行.
        delay_ms(100);
    }
}

// 串口收发函数 type为1,串口1收,发往串口3  type不为1,串口3,发往串口1
void usart_recv_send(u8 type)
{
    u8 i = 0;
    u8 tmp_len = 0;
    // 数据缓存
    static u8 buf[255] = {0};
    // 数据长度
    u8 buf_len = 0;

    // 返回缓存区数据的个数
    if(1 == type)
        tmp_len = usart1_getdata_count();
    else
        tmp_len = usart3_getdata_count();

    for(i=0; i<tmp_len; i++)
    {
        // 返回缓存区当前指针所指数据
        if(1 == type)
            buf[i] = usart1_receive_data();
        else
            buf[i] = usart3_receive_data();

        buf_len++;
        // 超过约定的上限长度
        if(buf_len >= 250)
        {
            buf_len=0;
            break;
        }
    }

    // 数据不为空
    if(0 != buf_len)
    {
        // 串口数据发送
        if(1 == type)
            usart3_send_bytes(buf, buf_len);
        else
            usart1_send_bytes(buf, buf_len);
    }
}
