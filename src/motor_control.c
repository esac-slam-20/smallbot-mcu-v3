/**
 * @file motor_control.c
 * @author Jim Jiang (jim@lotlab.org)
 * @brief 电机控制相关代码。负责接受速度，控制PWM，并读取编码器信息。
 * @version 0.1
 * @date 2021-07-28
 *
 * @copyright Copyright (c) GDUT ESAC 2021
 *
 */

#include "motor_control.h"
#include "batt.h"
#include "communication.h"
#include "config.h"
#include "debug.h"
#include "gpio.h"
#include "pid.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "ch32v10x_rcc.h"
#include "ch32v10x_tim.h"

// 电机数目
#define MOTOR_COUNT 4
// PWM 分辨率
#define PWM_MAX 1000
// 电机最低控制速度，rps。小于此速度时直接锁电机。
#define MIN_PID_SPEED 1

#define ABS(a) ((a) > 0 ? (a) : -(a))

/**
 * @brief 电机配置
 *
 */
struct Motor motors[MOTOR_COUNT] = {
    {.PWMChannel = 0,
     .OdomChannel = 1},
    {.PWMChannel = 1,
     .OdomChannel = 2},
    {.PWMChannel = 2,
     .OdomChannel = 0},
    {.PWMChannel = 3,
     .OdomChannel = -1},
};

/**
 * @brief 设置指定电机的PWM值
 *
 * @param motor 电机
 * @param cw 正反转
 * @param pwm PWM值
 */
static void motor_setPWM(struct Motor *motor, bool cw, uint16_t pwm)
{
    // todo: 刹车
    // 设置方向和占空比
    switch (motor->PWMChannel)
    {
    case 0:
        // CH1 拉高（禁用），PWM 控制 CH2 有规律的拉低，是正转。
        TIM_CCxCmd(TIM1, TIM_Channel_1, cw ? TIM_CCx_Disable : TIM_CCxN_Enable);
        TIM_CCxNCmd(TIM1, TIM_Channel_1, cw ? TIM_CCxN_Enable : TIM_CCx_Disable);
        TIM_SetCompare1(TIM1, pwm);
        break;
    case 1:
        TIM_CCxCmd(TIM1, TIM_Channel_2, cw ? TIM_CCx_Disable : TIM_CCxN_Enable);
        TIM_CCxNCmd(TIM1, TIM_Channel_2, cw ? TIM_CCxN_Enable : TIM_CCx_Disable);
        TIM_SetCompare2(TIM1, pwm);
        break;
    case 2:
        TIM_CCxCmd(TIM1, TIM_Channel_3, cw ? TIM_CCx_Disable : TIM_CCxN_Enable);
        TIM_CCxNCmd(TIM1, TIM_Channel_3, cw ? TIM_CCxN_Enable : TIM_CCx_Disable);
        TIM_SetCompare3(TIM1, pwm);
        break;
    case 3:
        // CH4 不支持互补输出
        TIM_SetCompare4(TIM1, pwm);
        break;
    default:
        break;
    }
}

/**
 * @brief 设置指定电机的PWM值
 *
 * @param motor 电机序号
 * @param pwm PWM值，范围为[-1, 1]。正数正转，负数反转。
 */
void motor_SetPWM(uint8_t motor, float pwm)
{
    if (motor >= MOTOR_COUNT)
        return;
    motor_setPWM(&motors[motor], pwm > 0, ABS(pwm) * PWM_MAX);
}

/**
 * @brief 初始化电机
 *
 */
static void motor_InitMotor()
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    TIM_TimeBaseInitTypeDef timer_initpara = {
        .TIM_Prescaler = 9,
        .TIM_Period = PWM_MAX,
        .TIM_ClockDivision = TIM_CKD_DIV1,
        .TIM_CounterMode = TIM_CounterMode_Up,
    };

    // 通常情况下，两个信号由对应输出极性控制。当Polar为Low时，说明有效值对应电平为0
    // 当CHx(N)_EN关闭后，对应信号会设置为无效值。
    // 当刹车信号给出后，POEN被强制设为0，此时两个引脚输出其空闲状态的电平。
    //
    // 在这里，我们希望：
    // - 电机正转时，CHx为低，CHxN为高，PWM控制CHx
    // - 电机反转时，CHx为高，CHxN为低，PWM控制CHxN
    // - 刹车时，两者都为高
    TIM_OCInitTypeDef timer_ocinitpara = {
        .TIM_OCMode = TIM_OCMode_PWM2,             // 当计数器超过指定值时变为无效电平
        .TIM_OutputState = TIM_OutputState_Enable, // 设定启用输出
        .TIM_OutputNState = TIM_OutputNState_Enable,
        .TIM_Pulse = 0,
        .TIM_OCPolarity = TIM_OCPolarity_Low,     // 当输出状态时，电平 = 0
        .TIM_OCNPolarity = TIM_OCPolarity_Low,    // 当输出状态时，电平 = 0
        .TIM_OCIdleState = TIM_OCIdleState_Set,   // 空闲状态 = 1
        .TIM_OCNIdleState = TIM_OCNIdleState_Set, // 空闲状态 = 1
    };

    TIM_BDTRInitTypeDef timer_brakeinitpara = {
        .TIM_OSSRState = TIM_OSSRState_Disable,
        .TIM_OSSIState = TIM_OSSIState_Disable,
        .TIM_LOCKLevel = TIM_LOCKLevel_OFF,
        .TIM_DeadTime = 0xFF,
        .TIM_Break = TIM_Break_Disable,
        .TIM_BreakPolarity = TIM_BreakPolarity_High,
        .TIM_AutomaticOutput = TIM_AutomaticOutput_Enable,
    };

    // 初始化Timer
    TIM_TimeBaseInit(TIM1, &timer_initpara);
    TIM_BDTRConfig(TIM1, &timer_brakeinitpara);
    TIM_OC1Init(TIM1, &timer_ocinitpara);
    TIM_OC2Init(TIM1, &timer_ocinitpara);
    TIM_OC3Init(TIM1, &timer_ocinitpara);
    TIM_OC4Init(TIM1, &timer_ocinitpara);

    // GPIO 初始化，设定为AF
    gpio_init_pin(GPIO_PA(8), GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
    gpio_init_pin(GPIO_PA(9), GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
    gpio_init_pin(GPIO_PA(10), GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
    gpio_init_pin(GPIO_PA(11), GPIO_Mode_AF_PP, GPIO_Speed_50MHz);

    gpio_init_pin(GPIO_PB(13), GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
    gpio_init_pin(GPIO_PB(14), GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
    gpio_init_pin(GPIO_PB(15), GPIO_Mode_AF_PP, GPIO_Speed_50MHz);

    GPIO_SetBits(GPIOA, GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11);
    GPIO_SetBits(GPIOB, GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);

    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Disable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
}

/**
 * @brief 初始化光电编码器
 *
 */
static void motor_InitEncoder()
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);

    TIM_ICInitTypeDef timer_icinitpara = {
        .TIM_ICPolarity = TIM_ICPolarity_Rising,
        .TIM_ICSelection = TIM_ICSelection_DirectTI,
        .TIM_ICPrescaler = TIM_ICPSC_DIV1,
        .TIM_ICFilter = 0,
    };

    TIM_TimeBaseInitTypeDef timer_initpara = {
        .TIM_Prescaler = 0, // 分频系数
        .TIM_CounterMode = TIM_CounterMode_Up,
        .TIM_Period = 65535, // 最大周期
        .TIM_ClockDivision = TIM_CKD_DIV1,
        .TIM_RepetitionCounter = 0};

    // 重映射 GPIO
    // TIMER2: A0, A1
    // TIMER3: PB4, PB5
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);
    // TIMER4: PB6, PB7

    // GPIO初始化
    gpio_init_pin(GPIO_PB(4), GPIO_Mode_IPD, GPIO_Speed_50MHz);
    gpio_init_pin(GPIO_PB(5), GPIO_Mode_IPD, GPIO_Speed_50MHz);
    gpio_init_pin(GPIO_PB(6), GPIO_Mode_IPD, GPIO_Speed_50MHz);
    gpio_init_pin(GPIO_PB(7), GPIO_Mode_IPD, GPIO_Speed_50MHz);
    gpio_init_pin(GPIO_PA(0), GPIO_Mode_IPD, GPIO_Speed_50MHz);
    gpio_init_pin(GPIO_PA(1), GPIO_Mode_IPD, GPIO_Speed_50MHz);

    for (size_t i = 0; i < MOTOR_COUNT; i++)
    {
        int8_t ch = motors[i].OdomChannel;
        if (ch < 0)
            continue;

        TIM_TypeDef *timer = (TIM_TypeDef *)((uint32_t)TIM2 + ch * 0x400);
        printf("Initing odom %d in 0x%lx.\r\n", ch, (uint32_t)timer);

        // 初始化外设
        TIM_TimeBaseInit(timer, &timer_initpara);

        // 输入捕获模式
        timer_icinitpara.TIM_Channel = TIM_Channel_1;
        TIM_ICInit(timer, &timer_icinitpara);
        timer_icinitpara.TIM_Channel = TIM_Channel_2;
        TIM_ICInit(timer, &timer_icinitpara);

        TIM_EncoderInterfaceConfig(timer, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge, TIM_ICPolarity_BothEdge);

        // 启用 Timer
        TIM_ARRPreloadConfig(timer, ENABLE);
        TIM_SetCounter(timer, 0);
        TIM_Cmd(timer, ENABLE);
    }
}

/**
 * @brief 初始化所有电机相关外设
 *
 */
void motor_Init()
{
    motor_InitMotor();
    motor_InitEncoder();
}

/**
 * @brief 电机目标速度，rpm
 *
 */
static int16_t motor_targetSpeeds[4];

#define COUNTER_RELOAD 200 // 1s

/**
 * @brief 数据存活计数器
 *
 */
static uint16_t data_valid_counter = 0;

/**
 * @brief 电量测量计数器
 *
 */
static uint16_t batt_counter = 0;

/**
 * @brief 设置电机目标速度
 *
 * @param speeds 电机速度，rpm
 */
void motor_SetSpeed(int16_t speeds[4])
{
    data_valid_counter = COUNTER_RELOAD;
    for (size_t i = 0; i < MOTOR_COUNT; i++)
    {
        motor_targetSpeeds[i] = speeds[i];
    }
    PRINT("Set motor speed to %d %d %d %d.\r\n", speeds[0], speeds[1], speeds[2], speeds[3]);
}

/**
 * @brief 32位编码器值
 *
 */
static int32_t decoder_val[4] = {0};

/**
 * @brief 16位编码器值
 *
 */
static uint16_t decoder_last_val[4] = {0};

/**
 * @brief 电机控制过程，负责读取编码器并调用PID
 *
 */
void motor_Routine()
{
    // 读取编码器数据
    int32_t delta[4] = {0};
    for (size_t i = 0; i < MOTOR_COUNT; i++)
    {
        int8_t ch = motors[i].OdomChannel;
        if (ch < 0)
            continue;

        TIM_TypeDef *timer = (TIM_TypeDef *)((uint32_t)TIM2 + ch * 0x400);

        uint32_t v = TIM_GetCounter(timer);
        delta[i] = v - decoder_last_val[i];
        delta[i] = -delta[i]; // 反转delta

        // 修正Overflow
        if (ABS(delta[i]) > 0x8000)
        {
            delta[i] -= delta[i] > 0 ? 0x10000 : -0x10000;
        }
        // 1信号=4Tick
        decoder_val[i] += delta[i] / 4;

        decoder_last_val[i] = v;
    }

    if (delta[0] != 0 || delta[1] != 0 || delta[2] != 0)
    {
        // printf("Motor: %ld %ld %ld\r\n", decoder_val[0], decoder_val[1], decoder_val[2]);
    }

    // 防止上层控制失效导致速度一直保持
    if (data_valid_counter > 0)
    {
        data_valid_counter--;
        if (data_valid_counter == 0)
        {
            motor_targetSpeeds[0] = 0;
            motor_targetSpeeds[1] = 0;
            motor_targetSpeeds[2] = 0;
            motor_targetSpeeds[3] = 0;
        }
    }

    // PID 控制电机
    for (size_t i = 0; i < MOTOR_COUNT; i++)
    {
        float targetSpeed = motor_targetSpeeds[i] / 60.0f;
        float currentSpeed = delta[i] / 4 / 0.005f / config_EncoderTicks;
        float val = 0;
        // 判断是否可以PID
        if (ABS(targetSpeed) > MIN_PID_SPEED)
        {
            val = pid_DoPID(i, targetSpeed, currentSpeed);
        }
        motor_SetPWM(i, val);
    }

    // 电量测量
    if (batt_counter++ > 1 * 200)
    {
        batt_counter = 0;
        batt_Measure();
    }
}

void motor_SendOdom()
{
    // 上报数据
    comm_SendOdom(decoder_val);
}
