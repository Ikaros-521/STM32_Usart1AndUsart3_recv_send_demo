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

// �����շ����� typeΪ1,����1��,��������3  type��Ϊ1,����3��,��������1
void usart_recv_send(u8 type);
// ����¼ �����յ������ݺ����ݳ��� ���� 0��֤�ɹ���1�˺Ż��������2���ݳ���
u8 check_login(u8* buf, u8 len);
/*
	�������ܣ� �������
	���Σ�     �յ������ݺ����ݳ���
	�����ʽ�� cmd#device#status#
	���أ� 	   0�����Ϲ��� 1�����ɹ�
*/
u8 cmd_analysis(u8* buf, u8 len);

int main(void)
{
    vu8 key = 0;
    delay_init();											 // ��ʱ������ʼ��
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 		 // ����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
    usart1_init(115200);									 // USART1��ʼ�� ������115200 Ĭ������λ8 ֹͣλ1 У��λnone
    usart3_init(115200);									 // USART3��ʼ�� ������115200 Ĭ������λ8 ֹͣλ1 У��λnone
    LED_Init();												 // LED�˿ڳ�ʼ��
    KEY_Init();										  	 	 // ��ʼ���밴�����ӵ�Ӳ���ӿ�
	BEEP_Init();									  	 	 // ��������ʼ��
	LED0 = 1;
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

// �����շ����� typeΪ1,����1��,��������3  type��Ϊ1,����3��,��������1
void usart_recv_send(u8 type)
{
    u8 i = 0;
    u8 tmp_len = 0;
    // ���ݻ���
    static u8 buf[255] = {0};
    //static u8 buf2[255] = {0};
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
        {
            usart1_send_bytes(buf, buf_len);
			printf("\r\n");
            // ���û�е�¼
            if(0 == login)
            {
                // ����¼
                if(0 == check_login(buf, buf_len))
                {
                    login = 1;
                    printf("��¼�ɹ�\r\n");
                }
                else if(2 == check_login(buf, buf_len))
                {
                    printf("�˺Ż����볬��\r\n");
                }
				else if(3 == check_login(buf, buf_len))
                {
                    printf("������̣��뷢������\"login#�˺�#����#\"��¼\r\n");
                }
                else
                {
                    printf("�˺Ż���������뷢������\"login#�˺�#����#\"��¼\r\n");
                }
            }
            // �Ѿ���¼
            else
            {
				cmd_analysis(buf, buf_len);
            }
        }
    }
}

// ����¼ �����յ������� ���� 0��֤�ɹ���1�˺Ż��������2���ݳ�����3���ݹ���
u8 check_login(u8* buf, u8 len)
{
    u8 i = 0, j = 0;
    u8 str_username[21] = {0};
    u8 str_password[21] = {0};
	if(len < 9)
	{
		return 3;
	}
    // ��¼����
    if(buf[0] == 'l' && buf[1] == 'o' && buf[2] == 'g' && buf[3] == 'i' && buf[4] == 'n' && buf[5] == '#')
    {
        // �������ݻ�ȡusername��password �ָ���Ϊ'#'
        j = 0;
        i = 6;
        while(buf[i] != '#' && i < len)
        {
            // ���ݳ���
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
            // ���ݳ���
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
	�������ܣ� �������
	�����ʽ�� cmd#device#status#
	���أ� 	   0�����Ϲ��� 1�����ɹ�
*/
u8 cmd_analysis(u8* buf, u8 len)
{
	u8 device[10] = {0};
	u8 status[6] = {0};
	u8 i = 0, j = 0;
	if(len < 7)
	{
		printf("�������\r\n");
		return 0;
	}
	
	// �����ʽУ��
	if(buf[0] == 'c' && buf[1] == 'm' && buf[2] == 'd' && buf[3] == '#')
	{
		// �������ݻ�ȡdevicestatus �ָ���Ϊ'#'
        j = 0;
        i = 4;
        while(buf[i] != '#' && i < len)
        {
            // ���ݳ���
            if(j >= 9)
            {
				printf("device����\r\n");
                return 0;
            }
            device[j++] = buf[i++];
        }
        device[j] = '\0';
		
		j = 0;
        i++;
        while(buf[i] != '#' && i < len)
        {
            // ���ݳ���
            if(j >= 5)
            {
				printf("status����\r\n");
                return 0;
            }
            status[j++] = buf[i++];
        }
        status[j] = '\0';
		
		// LED1������ ON/OFF
		if(0 == strcmp((char *)device, "LED1"))
        {
			if(0 == strcmp((char *)status, "ON"))
			{
				LED1 = 0;
				printf("LED1��\r\n");
				return 1;
			}
			else if(0 == strcmp((char *)status, "OFF"))
			{
				LED1 = 1;
				printf("LED1�ر�\r\n");
				return 1;
			}
			else
			{
				printf("�������\r\n");
				return 0;
			}  
        }
		else if(0 == strcmp((char *)device, "BEEP"))
        {
			if(0 == strcmp((char *)status, "ON"))
			{
				BEEP = 1;
				printf("BEEP��\r\n");
				return 1;
			}
			else if(0 == strcmp((char *)status, "OFF"))
			{
				BEEP = 0;
				printf("BEEP�ر�\r\n");
				return 1;
			}
			else
			{
				printf("�������\r\n");
				return 0;
			}  
        }
		else if(0 == strcmp((char *)device, "login"))
        {
			if(0 == strcmp((char *)status, "out"))
			{
				login = 0;
				printf("�˺ŵǳ�\r\n");
				return 1;
			}
			else
			{
				printf("�������\r\n");
				return 0;
			}  
        }
		else
		{
			printf("�������\r\n");
			return 0;
		}
	}
	else
	{
		printf("�������\r\n");
		return 0;
	}
}
