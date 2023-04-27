#include "batt.h"
#include "ch32v10x_adc.h"
#include "ch32v10x_gpio.h"
#include "ch32v10x_rcc.h"
#include "gpio.h"

#include "communication.h"
#include "systick.h"

static uint16_t Calibrattion_Val = 0;

/**
 * @brief 初始化电量测量
 *
 */
void batt_Init()
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    gpio_init_pin(GPIO_PB(0), GPIO_Mode_AIN, GPIO_Speed_50MHz);

    ADC_InitTypeDef init = {
        .ADC_Mode = ADC_Mode_Independent,
        .ADC_DataAlign = ADC_DataAlign_Right,
        .ADC_NbrOfChannel = 1,
        .ADC_ScanConvMode = DISABLE,
        .ADC_ContinuousConvMode = DISABLE,
        .ADC_ExternalTrigConv = ADC_ExternalTrigConv_None,
    };
    ADC_DeInit(ADC1);
    ADC_Init(ADC1, &init);

    ADC_DiscModeChannelCountConfig(ADC1, 1);

    ADC_Cmd(ADC1, ENABLE);
    delay_ms(1);

    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1))
        ;

    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1))
        ;
    Calibrattion_Val = Get_CalibrationValue(ADC1);
}

static volatile uint16_t voltage = 12000;

/**
 * @brief 电量测量
 *
 */
void batt_Measure()
{
    /* ADC regular channel config */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_7Cycles5);
    /* ADC software trigger enable */
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    /* wait the end of conversion flag */
    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC))
        ;
    /* clear the end of conversion flag */
    ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
    /* return regular channel sample value */
    uint16_t raw = ADC_GetConversionValue(ADC1) + Calibrattion_Val;

    // 28k + 100k 电阻分压
    voltage = (uint32_t)raw * 3300 / 4096 * (100 + 28) / 28;
}

/**
 * @brief 发送电压值
 *
 */
void batt_SendVoltage()
{
    comm_SendBatt(voltage);
}
