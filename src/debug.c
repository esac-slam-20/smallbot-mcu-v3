#include "debug.h"
#include "ch32v10x_usart.h"
#include "ch32v10x_rcc.h"
#include "gpio.h"
#include <stdio.h>

/**
 * @brief 初始化串口调试用UART
 *
 */
void debug_UARTInit()
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    // 初始化 PIN
    gpio_init_pin(GPIO_PB(10), GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
    gpio_init_pin(GPIO_PB(11), GPIO_Mode_IN_FLOATING, GPIO_Speed_50MHz);

    /* USART configure */
    USART_InitTypeDef USART_InitStructure = {
        .USART_BaudRate = 115200,
        .USART_WordLength = USART_WordLength_8b,
        .USART_StopBits = USART_StopBits_1,
        .USART_Parity = USART_Parity_No,
        .USART_Mode = USART_Mode_Rx | USART_Mode_Tx,
        .USART_HardwareFlowControl = USART_HardwareFlowControl_None,
    };
    USART_Init(USART3, &USART_InitStructure);
    USART_Cmd(USART3, ENABLE);
}

__attribute__((used)) int _write(int fd, char *buf, int size)
{
    for (int i = 0; i < size; i++)
    {
        while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
            ;
        USART_SendData(USART3, *buf++);
    }
    return size;
}

__attribute__((used)) void *_sbrk(ptrdiff_t incr)
{
    extern char _end[];
    extern char _heap_end[];
    static char *curbrk = _end;

    if ((curbrk + incr < _end) || (curbrk + incr > _heap_end))
        return NULL - 1;

    curbrk += incr;
    return curbrk - incr;
}
