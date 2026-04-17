#include "freertos_demo.h"
/* FreeRTOS相关头文件 */
#include "FreeRTOS.h"
#include "task.h"
/* 需要用到的标准库头文件 */

#include "stdio.h"

/* 任务相关定义 */
#define START_TASK_STACK 128
#define START_TASK_PRIORITY 1
TaskHandle_t start_task_handle;
void start_task(void *pvParameters);

/* 任务1定义 */
#define TASK1_STACK 128
#define TASK1_PRIORITY 2
TaskHandle_t task1_handle;
void task1(void *pvParameters);

/* 任务2定义 */
#define TASK2_STACK 128
#define TASK2_PRIORITY 3
TaskHandle_t task2_handle;
void task2(void *pvParameters);

/* 任务3定义 */
#define TASK3_STACK 128
#define TASK3_PRIORITY 4
TaskHandle_t task3_handle;
void task3(void *pvParameters);

/**
 * @description: 创建FreeRTOS任务
 * @return {*}
 */
void freertos_start(void)
{
    /* 1.创建一个启动任务 */
    xTaskCreate((TaskFunction_t)start_task,               // 任务函数地址
                (char *)"start_task",                     // 任务名称字符串
                (configSTACK_DEPTH_TYPE)START_TASK_STACK, // 任务堆栈大小，默认单位4字节
                (void *)NULL,                             // 传给任务的参数
                (UBaseType_t)START_TASK_PRIORITY,         // 任务优先级
                (TaskHandle_t *)&start_task_handle);      // 任务句柄地址

    /* 2.启动任务调度器：开始多任务调度 */
    vTaskStartScheduler();
}

/**
 * @description: 启动任务，用于创建其他任务
 * @param {void} *pvParameters
 * @return {*}
 */
void start_task(void *pvParameters)
{
    /* 临界区：在临界区内的代码不会被打断 */
    taskENTER_CRITICAL();

    /* 创建3个任务 */
    xTaskCreate((TaskFunction_t)task1,
                (char *)"task1",
                (configSTACK_DEPTH_TYPE)TASK1_STACK,
                (void *)NULL,
                (UBaseType_t)TASK1_PRIORITY,
                (TaskHandle_t *)&task1_handle);
    xTaskCreate((TaskFunction_t)task2,
                (char *)"task2",
                (configSTACK_DEPTH_TYPE)TASK2_STACK,
                (void *)NULL,
                (UBaseType_t)TASK2_PRIORITY,
                (TaskHandle_t *)&task2_handle);
    xTaskCreate((TaskFunction_t)task3,
                (char *)"task3",
                (configSTACK_DEPTH_TYPE)TASK3_STACK,
                (void *)NULL,
                (UBaseType_t)TASK3_PRIORITY,
                (TaskHandle_t *)&task3_handle);
    /* 这个任务只需要执行一次即可，执行完删除自己 */
    vTaskDelete(NULL);

    /* 退出临界区 */
    taskEXIT_CRITICAL();
}

/**
 * @description: 任务1，
 * @param {void} *pvParameters
 * @return {*}
 */
void task1(void *pvParameters)
{

    while (1)
    {
        printf("task1运行...\r\n");

        vTaskDelay(500);
    }
}

/**
 * @description: 任务2，
 * @param {void} *pvParameters
 * @return {*}
 */
void task2(void *pvParameters)
{
    while (1)
    {
        printf("task2运行...\r\n");

        vTaskDelay(500);
    }
}

/**
 * @description: 任务3，
 * @param {void} *pvParameters
 * @return {*}
 */
void task3(void *pvParameters)
{

    vTaskDelay(500);
    // HAL_Delay(500); // 使用delay会导致当前任务阻塞，失去CPU使用权
}
