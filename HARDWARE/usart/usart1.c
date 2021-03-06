#include "usart1.h"
#include "stdio.h"

static uint8_t usart1_buffer[255];
static uint8_t usart1_index;
static uint8_t usart1_count;

//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

// USART1初始化 默认数据位8 停止位1 校验位none
void usart1_init(u32 bound)
{
    // GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* config USART1 clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); // 使能USART1，GPIOA时钟

    /* USART1 GPIO config */
    /* Configure USART1 Tx (PA.09) as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure); // 初始化GPIOA.9
    /* Configure USART1 Rx (PA.10) as input floating */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure); //初始化GPIOA.10

    // Usart1 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
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
    USART_Init(USART1, &USART_InitStructure); // 初始化串口1
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // 开启串口接受中断

    USART_Cmd(USART1, ENABLE); // 使能串口1

    usart1_index=0;
    usart1_count=0;
}

// 中断服务函数
void USART1_IRQHandler(void)
{
    // 接收中断
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        if(usart1_count==usart1_index)
        {
            // 缓存区无数据，回到起点开始存储
            usart1_count=0;
            usart1_index=0;
        }
        usart1_buffer[usart1_count]=USART1->DR;
        usart1_count++;
        if(usart1_count>=250)
        {
            // 没有及时取出数据，导致存储位置到达末尾，回到起点
            usart1_count=0;
            usart1_index=0;
        }
    }
}

/*返回缓存区数据的个数*/
uint8_t usart1_getdata_count(void)
{
    return usart1_count-usart1_index;
}

/*返回缓存区当前指针所指数据*/
uint8_t usart1_receive_data(void)
{
    return usart1_buffer[usart1_index++];
}

/*串口数据发送函数
data_send：发送数据
*/
void usart1_send_byte(uint8_t data_send)
{
    USART_SendData(USART1, data_send);
    while (!(USART1->SR & USART_FLAG_TXE));
}

/*串口数据发送函数
data_buffer：发送数据串的首地址
length：发送数据的长度
*/
void usart1_send_bytes(uint8_t* data_buffer,uint8_t length)
{
    uint8_t i;
    for(i=0; i<length; i++)
    {
        usart1_send_byte(data_buffer[i]);
    }
}
