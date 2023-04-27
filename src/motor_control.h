#pragma once

#include <stdint.h>

/**
 * @brief 电机配置
 * 
 */
struct Motor
{
    // PWM通道
    int8_t PWMChannel;
    // 里程计的通道
    int8_t OdomChannel;
};

/**
 * @brief 初始化所有相关
 * 
 */
void motor_Init();

/**
 * @brief 运行电机定时操作
 * 
 */
void motor_Routine();

/**
 * @brief 设置电机目标速度
 * 
 * @param speeds 电机速度，rpm
 */
void motor_SetSpeed(int16_t speeds[4]);

/**
 * @brief 上报Odom数据
 * 
 */
void motor_SendOdom();

void motor_SetPWM(uint8_t motor, float pwm);
