#ifndef __LED_H__
#define __LED_H__

#include "stm32l1xx.h"

extern void led_init(void);
extern void led_on(void);
extern void led_off(void);
extern void led_toggle(void);

#endif
