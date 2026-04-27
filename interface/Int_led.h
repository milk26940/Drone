#ifndef INT_LED_H
#define INT_LED_H

#include "main.h"

/* 翻转指定 GPIO 对应的 LED 电平状态 */
void led_toggle(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

#endif // INT_LED_H
