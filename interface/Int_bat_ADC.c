#include "Int_bat_ADC.h"

/**
 * @brief 初始化电池电压 ADC 采样
 */
void Int_bat_ADC_Init(void)
{
    // 拉低使能脚，打开电池分压采样通道。
    HAL_GPIO_WritePin(BAT_ADC_EN_GPIO_Port, BAT_ADC_EN_Pin, GPIO_PIN_RESET);

    // ADC1 使用连续转换模式，启动后可周期读取转换结果。
    HAL_ADC_Start(&hadc1);
}

/**
 * @brief 读取电池电压
 * 
 * @return float 电池电压，单位 V。
 */
float Int_bat_ADC_Read(void)
{
    // 读取 12 位 ADC 原始值。
    uint32_t adc_value = HAL_ADC_GetValue(&hadc1);

    // 分压电路比例为 1:1，因此实际电压需要乘以 2。
    float voltage = (adc_value * 3.3 / 4095) * 2;
    return voltage;
}
