#include "delay.h"

// cycles per microsecond
static uint32_t usTicks = 0;

// cycles per millisecond
static uint32_t msTicks = 0;

void delay_init(void)
{
    // 1us时钟数
    usTicks = SystemCoreClock / 1000000;
    // 1ms时钟数
    msTicks = SystemCoreClock / 1000;
}

uint32_t micros(void)
{
    register uint32_t ms, cycle_cnt;

    do {
        ms = HAL_GetTick();
        cycle_cnt = SysTick->VAL;
    } while (ms != HAL_GetTick());

    return (ms * 1000) + (msTicks - cycle_cnt) / usTicks;
}

uint32_t millis(void)
{
    return HAL_GetTick();
}

void delay_us(uint32_t us)
{
    uint32_t ticks;
    uint32_t told;
    uint32_t tnow;
    uint32_t tcnt = 0;
    uint32_t reload;
       
	reload = SysTick->LOAD;                
    ticks = us * usTicks;
    
    tcnt = 0;
    told = SysTick->VAL;

    while (1)
    {
        tnow = SysTick->VAL;    
        if (tnow != told)
        {    
            if (tnow < told)
            {
                tcnt += told - tnow;    
            }
            else
            {
                tcnt += reload - tnow + told;    
            }        
            told = tnow;

            if (tcnt >= ticks)
            {
            	break;
            }
        }  
    }
}

void delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}
