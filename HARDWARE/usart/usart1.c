#include "usart1.h"
#include "stdio.h"

static uint8_t usart1_buffer[255];
static uint8_t usart1_index;
static uint8_t usart1_count;

// USART1��ʼ�� Ĭ������λ8 ֹͣλ1 У��λnone
void usart1_init(u32 bound)
{
    // GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* config USART1 clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); // ʹ��USART1��GPIOAʱ��

    /* USART1 GPIO config */
    /* Configure USART1 Tx (PA.09) as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //�����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure); // ��ʼ��GPIOA.9
    /* Configure USART1 Rx (PA.10) as input floating */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //��������
    GPIO_Init(GPIOA, &GPIO_InitStructure); //��ʼ��GPIOA.10

    // Usart1 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
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
    USART_Init(USART1, &USART_InitStructure); // ��ʼ������1
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // �������ڽ����ж�

    USART_Cmd(USART1, ENABLE); // ʹ�ܴ���1

    usart1_index=0;
    usart1_count=0;
}

// �жϷ�����
void USART1_IRQHandler(void)
{
    // �����ж�
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        if(usart1_count==usart1_index)
        {
            // �����������ݣ��ص���㿪ʼ�洢
            usart1_count=0;
            usart1_index=0;
        }
        usart1_buffer[usart1_count]=USART1->DR;
        usart1_count++;
        if(usart1_count>=250)
        {
            // û�м�ʱȡ�����ݣ����´洢λ�õ���ĩβ���ص����
            usart1_count=0;
            usart1_index=0;
        }
    }
}

/*���ػ��������ݵĸ���*/
uint8_t usart1_getdata_count(void)
{
    return usart1_count-usart1_index;
}

/*���ػ�������ǰָ����ָ����*/
uint8_t usart1_receive_data(void)
{
    return usart1_buffer[usart1_index++];
}

/*�������ݷ��ͺ���
data_send����������
*/
void usart1_send_byte(uint8_t data_send)
{
    USART_SendData(USART1, data_send);
    while (!(USART1->SR & USART_FLAG_TXE));
}

/*�������ݷ��ͺ���
data_buffer���������ݴ����׵�ַ
length���������ݵĳ���
*/
void usart1_send_bytes(uint8_t* data_buffer,uint8_t length)
{
    uint8_t i;
    for(i=0; i<length; i++)
    {
        usart1_send_byte(data_buffer[i]);
    }
}