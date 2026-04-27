#include "Com_debug.h"


/**
 * @brief 重定向标准输出到 USART2
 * @param ch 待发送字符
 * @param f 标准流句柄，当前未使用
 * @return 返回已发送字符
 */
int fputc(int ch, FILE *f)
{
    (void)f;
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 1000);
    return ch;
}


