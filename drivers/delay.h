#ifndef __DELAY_H__
#define __DELAY_H__

#include "stm32l1xx.h"

extern void delay_init(void);
extern uint32_t micros(void);
extern uint32_t millis(void);
extern void delay_us(uint32_t us);
extern void delay_ms(uint32_t ms);

#endif
