#include "bsp_tick.h"
#include "stm32h7xx.h"

static uint32_t HaL_Tick(void)
{
    uint32_t HaL_Tick = 0;
    
	  register uint32_t ms = 0, us= 0;

    ms = HAL_GetTick();

    us = TIM2->CNT;

    HaL_Tick = ms*1000 + us;

    return HaL_Tick;
}

void Delay_us(uint32_t us)
{
    uint32_t Now = HaL_Tick();

    while((HaL_Tick() - Now) < us);
}

void Delay_ms(uint32_t ms)
{
    uint32_t Now = HAL_GetTick();

    while((HAL_GetTick() - Now) < ms);
}