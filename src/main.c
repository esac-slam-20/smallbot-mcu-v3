#include "ch32v10x_usart.h"
#include "ch32v10x_rcc.h"
#include "gpio.h"

#include <stdint.h>

#include "communication.h"
#include "config.h"
#include "motor_control.h"
#include "batt.h"

#include "stdio.h"
#include "systick.h"
#include "debug.h"

static void init()
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);

    // 禁用SWJ接口
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);

    SystemInit();
    SysTick_Init();

    // 提前启用所有GPIO的时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);
    
    debug_UARTInit();    // 初始化调试接口
    config_Init();       // 配置初始化
    comm_Init();         // 初始化通信接口
    motor_Init();        // 初始化电机控制
    batt_Init();         // 初始化电量测量
}

int main()
{
    init();
    printf("Hello, world.\r\n");

    gpio_init_pin(GPIO_PC(13), GPIO_Mode_Out_PP, GPIO_Speed_10MHz);
    GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);

    int16_t speeds[] = {150, 150, 150, 0};

    bool led_state = false;
    while (1)
    {
        while (!SysTick_Flag)
        {
            __NOP();
        }
        SysTick_Flag = false;

        if (SysTick_Ms % 500 == 0)
        {
            GPIO_WriteBit(GPIOC, GPIO_Pin_13, led_state ? Bit_SET : Bit_RESET);
            led_state = !led_state;
        }

        // if (SysTick_Ms % 500 == 0)
        // {
        //     motor_SetSpeed(speeds);
        // }

        if (SysTick_Ms % 5 == 0)
        {
            motor_Routine();
        }
    }
}
