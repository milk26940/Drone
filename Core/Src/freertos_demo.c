/**
 * @file freertos_demo.c
 * @brief 无人机 FreeRTOS 任务编排
 */

#include "freertos_demo.h"

#include "App_flight.h"
#include "App_receive_data.h"
#include "Com_config.h"
#include "Int_TP4336.h"
#include "Int_SI24R1.h"
#include "Int_bat_ADC.h"
#include "gpio.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>

/* 电源保活任务栈深度。
 * 该任务逻辑很轻，只需完成任务通知等待和 TP4336 保活/关机控制，
 * 因此使用较小栈空间即可。
 */
#define POWER_TASK_STACK_SIZE 128
/* 电源保活任务优先级。
 * 需要及时响应关机请求，并周期性执行电源保活，所以优先级设置较高。
 */
#define POWER_TASK_PRIORITY 4
/* 电源保活任务周期，定期模拟按键维持供电。
 * 若电源芯片长时间未检测到“按键”动作，可能自动关闭输出，
 * 因此需要固定周期执行一次保活动作。
 */
#define POWER_TASK_PERIOD_MS 10000U

/* 飞控姿态解算与电机控制任务栈深度。
 * 该任务会调用姿态解算、PID 控制和电机输出等函数，
 * 计算量相对更大，但当前实现仍控制在较小栈占用范围内。
 */
#define FLIGHT_TASK_STACK_SIZE 128
/* 飞控任务优先级。
 * 飞控回路直接影响姿态稳定性，因此优先级高于 LED 等非关键任务。
 */
#define FLIGHT_TASK_PRIORITY 3
/* 飞控控制周期。
 * 6ms 对应约 166Hz 控制频率，用于姿态更新和电机控制输出。
 */
#define FLIGHT_TASK_PERIOD_MS 6U

/* 状态指示灯任务栈深度。
 * LED 任务只执行简单 GPIO 操作，栈需求最低。
 */
#define LED_TASK_STACK_SIZE 128
/* 状态指示灯任务优先级。
 * LED 仅用于状态显示，不参与控制闭环，因此优先级最低。
 */
#define LED_TASK_PRIORITY 1
/* LED 状态刷新周期。
 * 100ms 的节拍便于通过不同闪烁频率区分系统状态。
 */
#define LED_TASK_PERIOD_MS 100U

/* 通信任务栈深度。
 * 通信任务负责收发数据、状态处理和电压采样回传，
 * 栈需求介于飞控任务和 LED 任务之间。
 */
#define COM_TASK_STACK_SIZE 128
/* 通信任务优先级。
 * 遥控通信需要较好的实时性，尤其是掉线检测和关机命令响应，
 * 因此与电源任务同为较高优先级。
 */
#define COM_TASK_PRIORITY 4
/* 遥控接收与遥测回传任务周期。
 * 10ms 对应 100Hz 通信处理频率，可兼顾实时性和 CPU 占用。
 */
#define COM_TASK_PERIOD_MS 10U

/* 供外部读取的遥控连接标志。
 * 该变量用于向其他模块暴露“遥控器当前是否已连接”的简化状态。
 */
volatile uint8_t rc_connected = 0U;
/* 飞行主状态机当前状态。
 * 任务间通过该状态决定控制策略和指示灯显示模式。
 */
volatile Flight_State flight_state = IDLE;
/* 无线回传缓冲区，当前用于发送电池电压。
 * 缓冲区长度由无线芯片有效载荷宽度决定，使用字符串格式发送。
 */
uint8_t back_buff[TX_PLOAD_WIDTH] = {0};

/* 各任务句柄，用于任务间通知或调试。
 * 当前主要使用 power_task_handle 对电源任务发送通知，实现遥控关机。
 */
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
 * @note 启动顺序为：
 *       1. 初始化无线模块；
 *       2. 切换到接收模式；
 *       3. 创建各业务任务；
 *       4. 启动调度器，由 FreeRTOS 接管系统运行。
 */
void freertos_start(void)
{
    /* 初始化 SI24R1 无线模块，为后续遥控接收和数据回传做准备。 */
    SI24R1_Init();
    /* 默认进入接收模式，系统上电后优先等待遥控器数据。 */
    SI24R1_RX_Mode();

    /* 创建电源管理任务，负责保活和关机。 */
    xTaskCreate(Power_Task, "power", POWER_TASK_STACK_SIZE, NULL, POWER_TASK_PRIORITY, &power_task_handle);
    /* 创建飞控主任务，负责姿态解算和电机控制。 */
    xTaskCreate(Flight_Task, "flight", FLIGHT_TASK_STACK_SIZE, NULL, FLIGHT_TASK_PRIORITY, &flight_task_handle);
    /* 创建 LED 指示任务，用于显示当前工作状态。 */
    xTaskCreate(LED_Task, "led", LED_TASK_STACK_SIZE, NULL, LED_TASK_PRIORITY, &led_task_handle);
    /* 创建通信任务，负责遥控收发、掉线判断和电压回传。 */
    xTaskCreate(Com_Task, "com", COM_TASK_STACK_SIZE, NULL, COM_TASK_PRIORITY, &com_task_handle);

    /* 启动调度器后，程序正常情况下不会再返回到此处。 */
    vTaskStartScheduler();
}

/**
 * @brief 电源管理任务
 * @note 正常情况下周期性给 TP4336 发送保活脉冲；收到通知后执行关机动作
 * @details
 * 该任务利用“任务通知 + 超时等待”机制复用两种行为：
 * 1. 若等待超时，说明当前没有关机请求，则执行一次保活；
 * 2. 若等待期间收到通知，说明外部请求关机，则立即调用关机接口。
 */
static void Power_Task(void *pvParameters)
{
    (void)pvParameters;

    while (1)
    {
        /* 等待关机通知。
         * - 若返回值非 0，表示等待期间收到了通知；
         * - 若返回值为 0，表示等待超时。
         */
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(POWER_TASK_PERIOD_MS)) != 0U)
        {
            /* 收到关机请求后，通知电源芯片执行关机。 */
            TP4336_Shutdown();
        }
        else
        {
            /* 周期性执行保活，防止移动电源自动断电。 */
            TP4336_Start();
        }
    }
}

/**
 * @brief 飞控主任务
 * @note 负责姿态解算、姿态 PID、定高 PID 和电机输出控制
 * @details
 * 这是系统的核心控制任务，使用固定周期调度：
 * 1. 更新姿态角；
 * 2. 执行姿态控制 PID；
 * 3. 在定高模式下按分频节奏执行定高 PID；
 * 4. 根据控制结果更新电机输出。
 */
static void Flight_Task(void *pvParameters)
{
    /* 使用 vTaskDelayUntil 实现严格周期调度，避免任务执行时间抖动累积。 */
    TickType_t last_wake_time = xTaskGetTickCount();
    /* 定高 PID 的分频计数器。
     * 姿态环频率更高，定高环频率较低，因此通过计数实现“慢环”控制。
     */
    uint8_t height_count = 0U;

    (void)pvParameters;
    /* 飞控相关模块初始化，例如传感器、控制参数或滤波状态。 */
    App_flight_init();

    while (1)
    {
        /* 读取并更新当前欧拉角，作为控制输入。 */
        App_flight_get_euler_angle();
        /* 执行姿态闭环控制，计算横滚/俯仰/偏航控制量。 */
        App_flight_pid_process();

        /* 只有在定高模式下才运行定高控制环。 */
        if (flight_state == FIX_HEIGHT)
        {
            height_count++;
            /* 通过 4 次姿态环触发 1 次定高环，降低高度环执行频率。 */
            if (height_count >= 4U)
            {
                /* 执行定高 PID，修正油门或高度控制输出。 */
                App_flight_fix_height_pid_process();
                height_count = 0U;
            }
        }
        else
        {
            /* 退出定高模式时清零计数，避免重新进入时节拍异常。 */
            height_count = 0U;
        }

        /* 将各控制量映射为最终电机输出。 */
        App_flight_control_motor();
        /* 按固定周期运行飞控任务，维持稳定控制频率。 */
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(FLIGHT_TASK_PERIOD_MS));
    }
}

/**
 * @brief 状态指示灯任务
 * @note 通过不同闪烁模式反映遥控连接状态和飞行模式
 * @details
 * LED1/LED2：用于显示遥控连接状态；
 * LED3/LED4：用于显示飞行状态机状态，不同模式对应不同闪烁或常亮方式。
 */
void LED_Task(void *pvParameters)
{
    /* LED 任务同样使用固定周期运行，保证闪烁节奏稳定。 */
    TickType_t last_wake_time = xTaskGetTickCount();
    /* 软计数器，用于构造不同频率的闪烁模式。 */
    uint8_t count = 0U;

    (void)pvParameters;

    while (1)
    {
        /* 将外部遥控状态同步到简化连接标志，便于其他模块直接读取。 */
        rc_connected = (remote_state == REMOTE_CONNECTED) ? 1U : 0U;

        /* LED1/LED2 直接反映遥控连接状态：连接则点亮，断开则熄灭。 */
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, rc_connected ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, rc_connected ? GPIO_PIN_SET : GPIO_PIN_RESET);

        /* LED3/LED4 根据飞行状态显示不同模式。 */
        switch (flight_state)
        {
        case IDLE:
            /* 空闲模式下慢闪，表示系统空闲但正常运行。 */
            if ((count % 5U) == 0U)
            {
                HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
                HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
            }
            break;

        case NORMAL:
            /* 普通飞行模式下较快闪烁，用于区别于 IDLE。 */
            if ((count % 2U) == 0U)
            {
                HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
                HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
            }
            break;

        case FIX_HEIGHT:
            /* 定高模式下常亮，便于快速识别当前已进入辅助控制模式。 */
            HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
            break;

        case FAIL:
        default:
            /* 故障或未知状态下熄灭，作为异常提示。 */
            HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
            break;
        }

        /* 计数器循环递增，避免无界增长。 */
        count = (count >= 10U) ? 0U : (uint8_t)(count + 1U);
        /* 按设定节拍更新 LED 显示。 */
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(LED_TASK_PERIOD_MS));
    }
}

/**
 * @brief 通信任务
 * @note 负责遥控数据接收、掉线判定、飞行状态更新与电池电压回传
 * @details
 * 该任务承担“外部输入 -> 状态处理 -> 遥测输出”的链路：
 * 1. 接收遥控器发来的控制数据；
 * 2. 根据接收结果更新连接状态；
 * 3. 处理关机命令；
 * 4. 更新飞行状态机；
 * 5. 回传当前电池电压。
 */
static void Com_Task(void *pvParameters)
{
    /* 通信任务也使用固定周期执行，确保连接检测与回传节拍稳定。 */
    TickType_t last_wake_time = xTaskGetTickCount();

    (void)pvParameters;
    /* 初始化电池电压 ADC，供后续遥测读取。 */
    Int_bat_ADC_Init();

    while (1)
    {
        /* 接收遥控数据。
         * 返回值通常表示本周期是否成功收到有效数据，
         * 后续将基于该结果进行连接状态判定。
         */
        uint8_t receive_result = App_ReceiveData();

        /* 根据本次接收结果更新连接状态，例如正常连接、超时掉线等。 */
        App_process_connect_state(receive_result);

        /* 若遥控端下发关机命令，则通知电源任务执行关机流程。 */
        if (remote_data.shutdown == 1U)
        {
            xTaskNotifyGive(power_task_handle);
            /* 命令消费后立即清零，避免重复触发关机。 */
            remote_data.shutdown = 0U;
        }

        /* 根据最新遥控输入刷新飞行模式状态机。 */
        App_process_flight_state();

        /* 将当前电池电压格式化到回传缓冲区。
         * 例如生成 "3.85" 这样的字符串供无线模块发送。
         */
        snprintf((char *)back_buff, TX_PLOAD_WIDTH, "%.2f", Int_bat_ADC_Read());

        /* 固定周期运行通信逻辑。 */
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(COM_TASK_PERIOD_MS));
    }
}
