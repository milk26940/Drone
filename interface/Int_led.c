/**
 * @file    Int_led.c
 * @brief   LED 控制接口实现
 * @details 提供基础的 LED 引脚翻转功能
 */

#include "Int_led.h"

/**
 * @brief 翻转指定 LED 引脚电平
 * @param GPIOx 目标 GPIO 端口
 * @param GPIO_Pin 目标 GPIO 引脚
 */
void led_toggle(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    HAL_GPIO_TogglePin(GPIOx, GPIO_Pin);
}
