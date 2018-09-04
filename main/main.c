#include "stm32l1xx.h"
#include "system_clock.h"
#include "delay.h"
#include "led.h"
#include "usart.h"
#include <string.h>

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
    uart_init();
    
    while(1)
    {
        uint8_t read_byte = 0;
        com_get_char(COM1, &read_byte);
        delay_ms(100);
        if (read_byte == 'a')
        {
            com_send_buf(COM1, (uint8_t*)"hello world\n", strlen("hello world\n"));
        }
        else if (read_byte == 'b')
        {
            com_send_buf(COM1, (uint8_t*)"nice to meet you\n", strlen("nice to meet you\n"));
        }
        led_toggle();
    }
    
    return 0;
}
