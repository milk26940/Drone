#include "Com_debug.h"


//重定向编写一个函数 fputc，用于将字符发送到串口
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 1000);
    return ch;
}


