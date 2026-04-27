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

#define POWER_TASK_STACK_SIZE 128
#define POWER_TASK_PRIORITY 4
#define POWER_TASK_PERIOD_MS 10000U

#define FLIGHT_TASK_STACK_SIZE 128
#define FLIGHT_TASK_PRIORITY 3
#define FLIGHT_TASK_PERIOD_MS 6U

#define LED_TASK_STACK_SIZE 128
#define LED_TASK_PRIORITY 1
#define LED_TASK_PERIOD_MS 100U

#define COM_TASK_STACK_SIZE 128
#define COM_TASK_PRIORITY 4
#define COM_TASK_PERIOD_MS 10U

volatile uint8_t rc_connected = 0U;
volatile Flight_State flight_state = IDLE;
uint8_t back_buff[TX_PLOAD_WIDTH] = {0};

static TaskHandle_t power_task_handle;
static TaskHandle_t flight_task_handle;
static TaskHandle_t led_task_handle;
static TaskHandle_t com_task_handle;

extern Remote_Data remote_data;
extern Remote_State remote_state;

static void Power_Task(void *pvParameters);
static void Flight_Task(void *pvParameters);
static void Com_Task(void *pvParameters);

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

void set_rc_connected(uint8_t connected)
{
    rc_connected = connected ? 1U : 0U;
    remote_state = connected ? REMOTE_CONNECTED : REMOTE_DISCONNECTED;
}

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
