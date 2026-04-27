/**
 * @file freertos_demo.c
 * @brief 无人机 FreeRTOS 任务编排
 */

#include "freertos_demo.h"

#include "App_flight.h"
#include "App_receive_data.h"
#include "Com_config.h"
#include "Int_IP5305T.h"
#include "Int_SI24R1.h"
#include "Int_bat_ADC.h"
#include "gpio.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>

/* 电源保活任务栈深度。 */
#define POWER_TASK_STACK_SIZE 128
/* 电源保活任务优先级。 */
#define POWER_TASK_PRIORITY 4
/* 电源保活任务周期，定期模拟按键维持供电。 */
#define POWER_TASK_PERIOD_MS 10000U

/* 飞控姿态解算与电机控制任务栈深度。 */
#define FLIGHT_TASK_STACK_SIZE 128
/* 飞控任务优先级。 */
#define FLIGHT_TASK_PRIORITY 3
/* 飞控控制周期。 */
#define FLIGHT_TASK_PERIOD_MS 6U

/* 状态指示灯任务栈深度。 */
#define LED_TASK_STACK_SIZE 128
/* 状态指示灯任务优先级。 */
#define LED_TASK_PRIORITY 1
/* LED 状态刷新周期。 */
#define LED_TASK_PERIOD_MS 100U

/* 通信任务栈深度。 */
#define COM_TASK_STACK_SIZE 128
/* 通信任务优先级。 */
#define COM_TASK_PRIORITY 4
/* 遥控接收与遥测回传任务周期。 */
#define COM_TASK_PERIOD_MS 10U

/* 供外部读取的遥控连接标志。 */
volatile uint8_t rc_connected = 0U;
/* 飞行主状态机当前状态。 */
volatile Flight_State flight_state = IDLE;
/* 无线回传缓冲区，当前用于发送电池电压。 */
uint8_t back_buff[TX_PLOAD_WIDTH] = {0};

/* 各任务句柄，用于任务间通知或调试。 */
static TaskHandle_t power_task_handle;
static TaskHandle_t flight_task_handle;
static TaskHandle_t led_task_handle;
static TaskHandle_t com_task_handle;

extern Remote_Data remote_data;
extern Remote_State remote_state;

static void Power_Task(void *pvParameters);
static void Flight_Task(void *pvParameters);
static void Com_Task(void *pvParameters);

/**
 * @brief 创建系统任务并启动 FreeRTOS 调度器
 */
void freertos_start(void)
{
    SI24R1_Init();
    SI24R1_RX_Mode();

    xTaskCreate(Power_Task, "power", POWER_TASK_STACK_SIZE, NULL, POWER_TASK_PRIORITY, &power_task_handle);
    xTaskCreate(Flight_Task, "flight", FLIGHT_TASK_STACK_SIZE, NULL, FLIGHT_TASK_PRIORITY, &flight_task_handle);
    xTaskCreate(LED_Task, "led", LED_TASK_STACK_SIZE, NULL, LED_TASK_PRIORITY, &led_task_handle);
    xTaskCreate(Com_Task, "com", COM_TASK_STACK_SIZE, NULL, COM_TASK_PRIORITY, &com_task_handle);

    vTaskStartScheduler();
}

/**
 * @brief 电源管理任务
 * @note 正常情况下周期性给 IP5305T 发送保活脉冲；收到通知后执行关机动作
 */
static void Power_Task(void *pvParameters)
{
    (void)pvParameters;

    while (1)
    {
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(POWER_TASK_PERIOD_MS)) != 0U)
        {
            IP5305T_Shutdown();
        }
        else
        {
            IP5305T_Start();
        }
    }
}

/**
 * @brief 飞控主任务
 * @note 负责姿态解算、姿态 PID、定高 PID 和电机输出控制
 */
static void Flight_Task(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    uint8_t height_count = 0U;

    (void)pvParameters;
    App_flight_init();

    while (1)
    {
        App_flight_get_euler_angle();
        App_flight_pid_process();

        if (flight_state == FIX_HEIGHT)
        {
            height_count++;
            if (height_count >= 4U)
            {
                App_flight_fix_height_pid_process();
                height_count = 0U;
            }
        }
        else
        {
            height_count = 0U;
        }

        App_flight_control_motor();
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(FLIGHT_TASK_PERIOD_MS));
    }
}

/**
 * @brief 状态指示灯任务
 * @note 通过不同闪烁模式反映遥控连接状态和飞行模式
 */
void LED_Task(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    uint8_t count = 0U;

    (void)pvParameters;

    while (1)
    {
        rc_connected = (remote_state == REMOTE_CONNECTED) ? 1U : 0U;

        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, rc_connected ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, rc_connected ? GPIO_PIN_SET : GPIO_PIN_RESET);

        switch (flight_state)
        {
        case IDLE:
            if ((count % 5U) == 0U)
            {
                HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
                HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
            }
            break;

        case NORMAL:
            if ((count % 2U) == 0U)
            {
                HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
                HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
            }
            break;

        case FIX_HEIGHT:
            HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
            break;

        case FAIL:
        default:
            HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
            break;
        }

        count = (count >= 10U) ? 0U : (uint8_t)(count + 1U);
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(LED_TASK_PERIOD_MS));
    }
}

/**
 * @brief 设置遥控连接状态
 * @param connected 1 表示已连接，0 表示未连接
 */
void set_rc_connected(uint8_t connected)
{
    rc_connected = connected ? 1U : 0U;
    remote_state = connected ? REMOTE_CONNECTED : REMOTE_DISCONNECTED;
}

/**
 * @brief 通信任务
 * @note 负责遥控数据接收、掉线判定、飞行状态更新与电池电压回传
 */
static void Com_Task(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    (void)pvParameters;
    Int_bat_ADC_Init();

    while (1)
    {
        uint8_t receive_result = App_ReceiveData();

        App_process_connect_state(receive_result);

        if (remote_data.shutdown == 1U)
        {
            xTaskNotifyGive(power_task_handle);
            remote_data.shutdown = 0U;
        }

        App_process_flight_state();

        snprintf((char *)back_buff, TX_PLOAD_WIDTH, "%.2f", Int_bat_ADC_Read());

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(COM_TASK_PERIOD_MS));
    }
}
