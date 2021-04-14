
@[TOC](目录)
# 前言
开发板：正点原子 STM32F103 精英版
语言：C语言
开发环境：Keil5
使用了 KEY LED USART USB转TTL模块 智向的蓝牙模块（ps:电脑安装驱动CH340）
**代码下载**：[码云](https://gitee.com/ikaros-521/STM32_Usart1AndUsart3_recv_send_demo)  [GitHub](https://github.com/Ikaros-521/STM32_Usart1AndUsart3_recv_send_demo)
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210414172334869.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)

代码参考：正点原子 源码 串口实验例程
**功能介绍**：
1、LED的**0.2**秒一闪，表示程序正在运行。
2、串口1收到的数据会发给串口3，串口3收到的数据会发给串口1。
3、按键KEY1按下会向串口1发送数据‘1’，按键KEY0按下会向串口3发送数据‘3’。
# 接线
## USB转TTL
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210414173024830.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)

![在这里插入图片描述](https://img-blog.csdnimg.cn/20210414172832226.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210414172918503.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)
## 蓝牙
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210414174608456.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210414174715467.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210414175008697.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)

# 效果图
## USB转TTL
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210414172015329.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)
## 蓝牙
使用的手机软件（安卓）为 BLE调试助手
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210414175051499.jpg)
打开软件、蓝牙、给予权限等
扫描到我们的蓝牙模块，然后连接
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210414182506271.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)
连接成功后
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210414182541621.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)
点击最下面的 Unkonwn Service，展开，有接收 和 发送 按钮
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210414182643669.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)
### 手机收 电脑发
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210414183011609.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210414183043610.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)
### 手机发 电脑收
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210414183059821.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210414183111664.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)
### 蓝牙的连接/断开
蓝牙收到了手机发来的 连接 和 断开 命令
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210414183138426.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)
# 参考用图
## STM32F103
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210414183358864.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210414183457891.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)
![在这里插入图片描述](https://img-blog.csdnimg.cn/2021041418352852.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)

## 蓝牙模块相关
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210414171505592.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210414171523504.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210414171602317.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)

# 核心代码
**完整代码下载**：[码云](https://gitee.com/ikaros-521/STM32_Usart1AndUsart3_recv_send_demo)  [GitHub](https://github.com/Ikaros-521/STM32_Usart1AndUsart3_recv_send_demo)
## main.c

```c
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
    delay_init();												 // 延时函数初始化
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 设置NVIC中断分组2:2位抢占优先级，2位响应优先级
    usart1_init(115200);									 // USART1初始化 波特率115200 默认数据位8 停止位1 校验位none
    usart3_init(115200);									 // USART3初始化 波特率115200 默认数据位8 停止位1 校验位none
    LED_Init();												 // LED端口初始化
    KEY_Init();										  	 	 // 初始化与按键连接的硬件接口
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

```
## usart1.c

```c
#include "usart1.h"
#include "stdio.h"

static uint8_t usart1_buffer[255];
static uint8_t usart1_index;
static uint8_t usart1_count;

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

```
## usart3.c

```c
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

```

