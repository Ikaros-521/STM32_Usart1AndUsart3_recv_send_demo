#include "usart3.h"
#include "stdio.h"

static uint8_t usart3_buffer[255];
static uint8_t usart3_index;
static uint8_t usart3_count;

// USART3初始化 默认 数据位8 停止位1 校验位none
void usart3_init(u32 bound)
{
	// GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* config USART3 clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //使能USART3，GPIOB时钟

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
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;//抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器

    USART_InitStructure.USART_BaudRate = bound; // 串口波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; // 字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1; // 一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No; // 无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // 收发模式
    USART_Init(USART3, &USART_InitStructure); // 初始化串口3
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); // 开启串口接受中断

    USART_Cmd(USART3, ENABLE); // 使能串口3 

    usart3_index=0;
    usart3_count=0;
}

// 中断服务函数
void USART3_IRQHandler(void)
{
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        if(usart3_count==usart3_index)
        {
            //缓存区无数据，回到起点开始存储
            usart3_count=0;
            usart3_index=0;
        }
        usart3_buffer[usart3_count]=USART3->DR;
        usart3_count++;
        if(usart3_count>=250)
        {
            //没有及时取出数据，导致存储位置到达末尾，回到起点
            usart3_count=0;
            usart3_index=0;
        }
    }
}

/*返回缓存区数据的个数*/
uint8_t usart3_getdata_count(void)
{
    return usart3_count-usart3_index;
}

/*返回缓存区当前指针所指数据*/
uint8_t usart3_receive_data(void)
{
    return usart3_buffer[usart3_index++];
}

/*串口数据发送函数
data_send：发送数据
*/
void usart3_send_byte(uint8_t data_send)
{
    USART_SendData(USART3, data_send);
    while (!(USART3->SR & USART_FLAG_TXE));
}

/*串口数据发送函数
data_buffer：发送数据串的首地址
length：发送数据的长度
*/
void usart3_send_bytes(uint8_t* data_buffer,uint8_t length)
{
    uint8_t i;
    for(i=0; i<length; i++)
    {
        usart3_send_byte(data_buffer[i]);
    }
}
