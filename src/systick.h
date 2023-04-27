#pragma once

#include <stdint.h>
#include <stdbool.h>

extern bool SysTick_Flag;
extern uint32_t SysTick_Ms;

void SysTick_Init(void);
void delay_us(uint32_t count);
void delay_ms(uint32_t count);
