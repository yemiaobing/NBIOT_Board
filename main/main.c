#include "stm32l1xx.h"
#include "system_clock.h"
#include "delay.h"
#include "led.h"

void platform_init(void)
{
    HAL_Init();
    SystemClock_Config();
    delay_init();
}

int main(void)
{
    platform_init();
    led_init();
    while(1)
    {
        delay_ms(1000);
        led_toggle();
        delay_ms(1000);
        led_toggle();
    }
    
    return 0;
}
