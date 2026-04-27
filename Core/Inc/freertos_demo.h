#ifndef __FREERTOS_DEMO_H
#define __FREERTOS_DEMO_H

#include <stdint.h>
#include "../../common/Com_config.h"

/* 遥控连接状态标志，供应用层和 LED 任务共享。 */
extern volatile uint8_t rc_connected;
/* 飞行主状态机当前状态。 */
extern volatile Flight_State flight_state;

/* 创建系统任务并启动调度器。 */
void freertos_start(void);
/* LED 状态指示任务入口。 */
void LED_Task(void *pvParameters);
/* 供其他模块手动更新遥控连接状态。 */
void set_rc_connected(uint8_t connected);

#endif
