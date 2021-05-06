#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "usart1.h"
#include "usart3.h"
#include "string.h"
#include "stdio.h"
#include "beep.h"

u8 login = 0;
u8 username[21] = "admin";
u8 password[21] = "admin";

// 串口收发函数 type为1,串口1收,发往串口3  type不为1,串口3收,发往串口1
void usart_recv_send(u8 type);
// 检查登录 传入收到的数据和数据长度 返回 0验证成功，1账号或密码错误，2数据超长
u8 check_login(u8* buf, u8 len);
/*
	函数功能： 命令解析
	传参：     收到的数据和数据长度
	命令格式： cmd#device#status#
	返回： 	   0不符合规则 1解析成功
*/
u8 cmd_analysis(u8* buf, u8 len);

int main(void)
{
    vu8 key = 0;
    delay_init();											 // 延时函数初始化
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 		 // 设置NVIC中断分组2:2位抢占优先级，2位响应优先级
    usart1_init(115200);									 // USART1初始化 波特率115200 默认数据位8 停止位1 校验位none
    usart3_init(115200);									 // USART3初始化 波特率115200 默认数据位8 停止位1 校验位none
    LED_Init();												 // LED端口初始化
    KEY_Init();										  	 	 // 初始化与按键连接的硬件接口
	BEEP_Init();									  	 	 // 蜂鸣器初始化
	LED0 = 1;
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

// 串口收发函数 type为1,串口1收,发往串口3  type不为1,串口3收,发往串口1
void usart_recv_send(u8 type)
{
    u8 i = 0;
    u8 tmp_len = 0;
    // 数据缓存
    static u8 buf[255] = {0};
    //static u8 buf2[255] = {0};
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
        {
            usart1_send_bytes(buf, buf_len);
			printf("\r\n");
            // 如果没有登录
            if(0 == login)
            {
                // 检查登录
                if(0 == check_login(buf, buf_len))
                {
                    login = 1;
                    printf("登录成功\r\n");
                }
                else if(2 == check_login(buf, buf_len))
                {
                    printf("账号或密码超长\r\n");
                }
				else if(3 == check_login(buf, buf_len))
                {
                    printf("命令过短，请发送命令\"login#账号#密码#\"登录\r\n");
                }
                else
                {
                    printf("账号或密码错误，请发送命令\"login#账号#密码#\"登录\r\n");
                }
            }
            // 已经登录
            else
            {
				cmd_analysis(buf, buf_len);
            }
        }
    }
}

// 检查登录 传入收到的数据 返回 0验证成功，1账号或密码错误，2数据超长，3数据过短
u8 check_login(u8* buf, u8 len)
{
    u8 i = 0, j = 0;
    u8 str_username[21] = {0};
    u8 str_password[21] = {0};
	if(len < 9)
	{
		return 3;
	}
    // 登录命令
    if(buf[0] == 'l' && buf[1] == 'o' && buf[2] == 'g' && buf[3] == 'i' && buf[4] == 'n' && buf[5] == '#')
    {
        // 解析数据获取username和password 分隔符为'#'
        j = 0;
        i = 6;
        while(buf[i] != '#' && i < len)
        {
            // 数据超长
            if(j >= 20)
            {
                return 2;
            }
            str_username[j++] = buf[i++];
        }
        str_username[j] = '\0';

        j = 0;
        i++;
        while(buf[i] != '#' && i < len)
        {
            // 数据超长
            if(j >= 20)
            {
                return 2;
            }
            str_password[j++] = buf[i++];
        }
        str_password[j] = '\0';

        if(0 == strcmp((char *)str_username, (char *)username) && 0 == strcmp((char *)str_password, (char *)password))
        {
            return 0;
        }
        else
        {
            return 1;
        }
    }
    else
    {
        return 1;
    }
}

/*
	函数功能： 命令解析
	命令格式： cmd#device#status#
	返回： 	   0不符合规则 1解析成功
*/
u8 cmd_analysis(u8* buf, u8 len)
{
	u8 device[10] = {0};
	u8 status[6] = {0};
	u8 i = 0, j = 0;
	if(len < 7)
	{
		printf("命令过短\r\n");
		return 0;
	}
	
	// 命令格式校验
	if(buf[0] == 'c' && buf[1] == 'm' && buf[2] == 'd' && buf[3] == '#')
	{
		// 解析数据获取devicestatus 分隔符为'#'
        j = 0;
        i = 4;
        while(buf[i] != '#' && i < len)
        {
            // 数据超长
            if(j >= 9)
            {
				printf("device超长\r\n");
                return 0;
            }
            device[j++] = buf[i++];
        }
        device[j] = '\0';
		
		j = 0;
        i++;
        while(buf[i] != '#' && i < len)
        {
            // 数据超长
            if(j >= 5)
            {
				printf("status超长\r\n");
                return 0;
            }
            status[j++] = buf[i++];
        }
        status[j] = '\0';
		
		// LED1的命令 ON/OFF
		if(0 == strcmp((char *)device, "LED1"))
        {
			if(0 == strcmp((char *)status, "ON"))
			{
				LED1 = 0;
				printf("LED1打开\r\n");
				return 1;
			}
			else if(0 == strcmp((char *)status, "OFF"))
			{
				LED1 = 1;
				printf("LED1关闭\r\n");
				return 1;
			}
			else
			{
				printf("命令错误\r\n");
				return 0;
			}  
        }
		else if(0 == strcmp((char *)device, "BEEP"))
        {
			if(0 == strcmp((char *)status, "ON"))
			{
				BEEP = 1;
				printf("BEEP打开\r\n");
				return 1;
			}
			else if(0 == strcmp((char *)status, "OFF"))
			{
				BEEP = 0;
				printf("BEEP关闭\r\n");
				return 1;
			}
			else
			{
				printf("命令错误\r\n");
				return 0;
			}  
        }
		else if(0 == strcmp((char *)device, "login"))
        {
			if(0 == strcmp((char *)status, "out"))
			{
				login = 0;
				printf("账号登出\r\n");
				return 1;
			}
			else
			{
				printf("命令错误\r\n");
				return 0;
			}  
        }
		else
		{
			printf("命令错误\r\n");
			return 0;
		}
	}
	else
	{
		printf("命令错误\r\n");
		return 0;
	}
}
