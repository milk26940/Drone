#ifndef INT_IP5305T_H
#define INT_IP5305T_H

#include "main.h"

/* 触发一次 IP5305T 按键动作 */
void IP5305T_Start(void);
/* 执行 IP5305T 关机按键序列 */
void IP5305T_Shutdown(void);
/* 周期性执行 IP5305T 保活的 FreeRTOS 任务入口 */
void IP5305T_Task(void *pvParameters);


#endif // INT_IP5305T_H

