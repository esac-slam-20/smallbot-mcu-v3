/**
 * @file systick.c
 * @author Jim Jiang (jim@lotlab.org)
 * @brief
 * @version 0.1
 * @date 2023-04-26
 *
 * @copyright Copyright (c) GDUT ESAC 2023
 *
 */

#include "systick.h"
#include "system_ch32v10x.h"
#include "ch32v10x.h"
#include "ch32v10x_it.h"
#include "core_riscv.h"

#include "motor_control.h"

#define SysTick_CNTL ((uint32_t *)((uint32_t)SysTick + 4))
#define SysTick_CNTH ((uint32_t *)((uint32_t)SysTick + 8))

bool SysTick_Flag = false;
uint32_t SysTick_Ms = 0;

static uint8_t p_us = 0;
static uint16_t p_ms = 0;

static void SysTick_SetNext()
{
    SysTick->CTLR = 0;

    uint64_t current = *SysTick_CNTL;
    current += ((uint64_t)*SysTick_CNTH) << 32;

    current += p_ms;

    SysTick->CMPLR0 = ((current >> 0) & 0xFF);
    SysTick->CMPLR1 = ((current >> 8) & 0xFF);
    SysTick->CMPLR2 = ((current >> 16) & 0xFF);
    SysTick->CMPLR3 = ((current >> 24) & 0xFF);
    SysTick->CMPHR0 = ((current >> 32) & 0xFF);
    SysTick->CMPHR1 = ((current >> 40) & 0xFF);
    SysTick->CMPHR2 = ((current >> 48) & 0xFF);
    SysTick->CMPHR3 = ((current >> 56) & 0xFF);

    SysTick->CTLR = 1;
}

void SysTick_Init()
{
    p_us = SystemCoreClock / 8000000;
    p_ms = (uint16_t)p_us * 1000;

    SysTick_SetNext();
    NVIC_EnableIRQ(SysTicK_IRQn);
}


static void delay(uint32_t tick)
{
    uint32_t current = *SysTick_CNTL;
    current += tick;

    while (*SysTick_CNTL < current) {
        __NOP();
    }
}

void delay_us(uint32_t count)
{
    delay(p_us * count);
}

void delay_ms(uint32_t count)
{
    delay(p_ms * count);
}

__attribute__((interrupt(WCH_INTERRUPT_TYPE)))
void SysTick_Handler()
{
    NVIC_ClearPendingIRQ(SysTicK_IRQn);
    SysTick_SetNext();

    // todo: 事件调度
    SysTick_Flag = true;
    SysTick_Ms++;
}