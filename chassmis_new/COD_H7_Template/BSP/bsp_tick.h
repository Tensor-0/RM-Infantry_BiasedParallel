#ifndef BSP_TICK_H
#define BSP_TICK_H

#ifdef __cplusplus
extern "C" {
#endif


#include "stdint.h"

extern void Delay_us(uint32_t us);

extern void Delay_ms(uint32_t ms);
	
	
#endif