#ifndef INT_TP4336_H
#define INT_TP4336_H

#include "main.h"

/* 触发一次 TP4336 按键动作 */
void TP4336_Start(void);
/* 执行 TP4336 关机按键序列 */
void TP4336_Shutdown(void);
/* 周期性执行 TP4336 保活的 FreeRTOS 任务入口 */
void TP4336_Task(void *pvParameters);


#endif // INT_TP4336_H

