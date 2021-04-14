#ifndef __USART1_H
#define __USART1_H
#include "stm32f10x.h"
#include "delay.h"
extern void usart1_init(u32 bound);
extern uint8_t usart1_getdata_count(void);
extern uint8_t usart1_receive_data(void);
extern void usart1_send_byte(uint8_t data_send);
extern void usart1_send_bytes(uint8_t* data_buffer,uint8_t length);
#endif

