#include "usart3.h"
#include "stdio.h"

static uint8_t usart3_buffer[255];
static uint8_t usart3_index;
static uint8_t usart3_count;

// USART3��ʼ�� Ĭ�� ����λ8 ֹͣλ1 У��λnone
void usart3_init(u32 bound)
{
	// GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* config USART3 clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //ʹ��USART3��GPIOBʱ��

    /* USART3 GPIO config */
    /* Configure USART3 Tx (PB.10) as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    /* Configure USART3 Rx (PB.11) as input floating */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;//��ռ���ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//�����ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���

    USART_InitStructure.USART_BaudRate = bound; // ���ڲ�����
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; // �ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1; // һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No; // ����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // ��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // �շ�ģʽ
    USART_Init(USART3, &USART_InitStructure); // ��ʼ������3
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); // �������ڽ����ж�

    USART_Cmd(USART3, ENABLE); // ʹ�ܴ���3 

    usart3_index=0;
    usart3_count=0;
}

// �жϷ�����
void USART3_IRQHandler(void)
{
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        if(usart3_count==usart3_index)
        {
            //�����������ݣ��ص���㿪ʼ�洢
            usart3_count=0;
            usart3_index=0;
        }
        usart3_buffer[usart3_count]=USART3->DR;
        usart3_count++;
        if(usart3_count>=250)
        {
            //û�м�ʱȡ�����ݣ����´洢λ�õ���ĩβ���ص����
            usart3_count=0;
            usart3_index=0;
        }
    }
}

/*���ػ��������ݵĸ���*/
uint8_t usart3_getdata_count(void)
{
    return usart3_count-usart3_index;
}

/*���ػ�������ǰָ����ָ����*/
uint8_t usart3_receive_data(void)
{
    return usart3_buffer[usart3_index++];
}

/*�������ݷ��ͺ���
data_send����������
*/
void usart3_send_byte(uint8_t data_send)
{
    USART_SendData(USART3, data_send);
    while (!(USART3->SR & USART_FLAG_TXE));
}

/*�������ݷ��ͺ���
data_buffer���������ݴ����׵�ַ
length���������ݵĳ���
*/
void usart3_send_bytes(uint8_t* data_buffer,uint8_t length)
{
    uint8_t i;
    for(i=0; i<length; i++)
    {
        usart3_send_byte(data_buffer[i]);
    }
}
