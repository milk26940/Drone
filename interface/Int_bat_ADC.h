#ifndef __INT_BAT_ADC__
#define __INT_BAT_ADC__

#include "adc.h"

/**
 * @brief 初始化电池电压 ADC 采样
 */
void Int_bat_ADC_Init(void);

/**
 * @brief 读取电池电压
 * 
 * @return float 电池电压，单位 V。
 */
float Int_bat_ADC_Read(void);

#endif // __INT_BAT_ADC__
