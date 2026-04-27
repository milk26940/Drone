/**
 * @file    Int_IP5305T.c
 * @brief   IP5305T 电源管理接口实现
 * @details 通过模拟按键脉冲维持电源芯片工作状态
 */

#include "Int_IP5305T.h"
#include "FreeRTOS.h"
#include "task.h"

/**
 * @brief 触发一次 IP5305T 电源按键
 * @note  通过拉低再拉高按键引脚，模拟人工短按动作
 */
void IP5305T_Start(void)
{
    HAL_GPIO_WritePin(POWER_KEY_GPIO_Port, POWER_KEY_Pin, GPIO_PIN_RESET);
    vTaskDelay(pdMS_TO_TICKS(100));
    HAL_GPIO_WritePin(POWER_KEY_GPIO_Port, POWER_KEY_Pin, GPIO_PIN_SET);
}

/**
 * @brief 执行 IP5305T 关机按键序列
 * @note  通过两次短按模拟关机命令。
 */
void IP5305T_Shutdown(void)
{
    HAL_GPIO_WritePin(POWER_KEY_GPIO_Port, POWER_KEY_Pin, GPIO_PIN_RESET);
    vTaskDelay(pdMS_TO_TICKS(100));
    HAL_GPIO_WritePin(POWER_KEY_GPIO_Port, POWER_KEY_Pin, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(200));

    HAL_GPIO_WritePin(POWER_KEY_GPIO_Port, POWER_KEY_Pin, GPIO_PIN_RESET);
    vTaskDelay(pdMS_TO_TICKS(100));
    HAL_GPIO_WritePin(POWER_KEY_GPIO_Port, POWER_KEY_Pin, GPIO_PIN_SET);
}

/**
 * @brief IP5305T 保活任务
 * @param pvParameters 任务参数，未使用
 * @note  周期性触发按键，避免电源管理芯片自动关断
 */
void IP5305T_Task(void *pvParameters)
{
    (void)pvParameters;

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(10000));
        IP5305T_Start();
    }
}
