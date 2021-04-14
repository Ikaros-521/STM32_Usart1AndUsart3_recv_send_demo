#ifndef __USART3_H
#define __USART3_H
#include "stm32f10x.h"
#include "delay.h"
extern void usart3_init(u32 bound);
extern uint8_t usart3_getdata_count(void);
extern uint8_t usart3_receive_data(void);
extern void usart3_send_byte(uint8_t data_send);
extern void usart3_send_bytes(uint8_t* data_buffer,uint8_t length);
#endif

