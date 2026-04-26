/**
 * @file freertos_demo.c
 * @brief FreeRTOS 任务管理模块 - 无人机核心任务处理
 * @details 本文件实现了无人机的三个主要FreeRTOS任务：
 *          1. IP5305T电源管理任务 - 监测和管理电池状态
 *          2. 电机控制任务 - 控制四个电机的转速
 *          3. LED指示任务 - 控制LED灯状态指示
 * @version 1.0
 */

#include "freertos_demo.h"
#include "../../interface/Int_IP5305T.h"     /* IP5305T电源管理芯片接口 */
#include "../../interface/Int_Motor.h"       /* 电机驱动接口 */
#include "../../interface/Int_led.h"         /* LED控制接口 */
#include "../../interface/Int_SI24R1.h"      /* SI24R1无线射频模块接口 */
#include "../../common/Com_config.h"         /* 通用配置 */

/* ==================== FreeRTOS相关头文件 ==================== */
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

/* ==================== STM32 HAL驱动头文件 ==================== */
#include "tim.h"                             /* 定时器配置 */
#include "gpio.h"                            /* GPIO配置 */

/* ==================== 任务相关宏定义 ==================== */
/** IP5305T电源管理任务 */
#define IP5305T_TASK_STACK 128              /* 任务栈大小 (字数) */
#define IP5305T_TASK_PRIORITY 2             /* 任务优先级 */

/** 电机控制任务 */
#define MOTOR_TASK_STACK 128                /* 任务栈大小 (字数) */
#define MOTOR_TASK_PRIORITY 2               /* 任务优先级 */

/** LED指示任务 */
#define LED_TASK_STACK 128                  /* 任务栈大小 (字数) */
#define LED_TASK_PRIORITY 1                 /* 任务优先级 (最低) */

/** SI24R1接收任务 */
#define SI24R1_RX_TASK_STACK 128            /* 任务栈大小 (字数) */
#define SI24R1_RX_TASK_PRIORITY 3           /* 任务优先级 */
#define SI24R1_RX_TASK_PERIOD_MS 10U        /* 接收轮询周期 */
#define SI24R1_RX_LOST_TIMEOUT_MS 1000U     /* 接收丢失判定时间 */

/** SI24R1发送任务 */
#define SI24R1_TX_TASK_STACK 128            /* 任务栈大小 (字数) */
#define SI24R1_TX_TASK_PRIORITY 2           /* 任务优先级 */
#define SI24R1_TX_TASK_PERIOD_MS 1000U      /* 测试发送周期 */

/* ==================== 电机控制模块变量 ==================== */
/** 
 * 四个电机的控制结构体实例
 * - motor_left_0/1: 左侧两个电机 (TIM3/TIM4驱动)
 * - motor_right_0/1: 右侧两个电机 (TIM2/TIM1驱动)
 * 每个电机配置有定时器通道和初始速度参数
 */
static MotorControl_t motor_left_0 = {LEFT_0, &htim3, TIM_CHANNEL_1, 200};
static MotorControl_t motor_left_1 = {LEFT_1, &htim4, TIM_CHANNEL_4, 200};
static MotorControl_t motor_right_0 = {RIGHT_0, &htim2, TIM_CHANNEL_2, 200};
static MotorControl_t motor_right_1 = {RIGHT_1, &htim1, TIM_CHANNEL_3, 200};

volatile uint8_t rc_connected = 0;
volatile Flight_State flight_state = IDLE;


/* ==================== 任务句柄 ==================== */
/** 存储各任务的句柄，用于后续任务管理 */
TaskHandle_t ip5305t_task_handle;           /* IP5305T任务句柄 */
TaskHandle_t motor_task_handle;             /* 电机控制任务句柄 */
TaskHandle_t led_task_handle;               /* LED指示任务句柄 */
TaskHandle_t si24r1_rx_task_handle;         /* SI24R1接收任务句柄 */
TaskHandle_t si24r1_tx_task_handle;         /* SI24R1发送任务句柄 */
static SemaphoreHandle_t si24r1_lock;       /* SI24R1收发访问锁 */

volatile u8 si24r1_rx_buffer[TX_PLOAD_WIDTH];
volatile u8 si24r1_tx_result = 0U;

/* ==================== 任务函数前向声明 ==================== */
void IP5305T_Task(void *pvParameters);
void Motor_Task(void *pvParameters);
void LED_Task(void *pvParameters);
static void SI24R1_Rx_Task(void *pvParameters);
static void SI24R1_Tx_Task(void *pvParameters);

/**
 * @brief FreeRTOS 系统初始化和启动函数
 * @details 本函数完成以下操作：
 *          1. 初始化四个电机的PWM驱动
 *          2. 创建IP5305T电源管理任务
 *          3. 创建电机控制任务
 *          4. 创建LED指示任务
 *          5. 启动FreeRTOS任务调度器
 * @return 无
 * @note 该函数在 main() 中调用，一旦调用则进入 FreeRTOS 调度环境
 */
void freertos_start(void)
{
    /* -------- 1. 初始化四个电机的PWM输出 -------- */
    motor_start(&motor_left_0);
    motor_start(&motor_left_1);
    motor_start(&motor_right_0);
    motor_start(&motor_right_1);

    /* -------- 2. 初始化SI24R1无线模块和访问锁 -------- */
    SI24R1_Init();
    SI24R1_RX_Mode();
    si24r1_lock = xSemaphoreCreateBinary();
    if (si24r1_lock == NULL)
    {
        Error_Handler();
    }
    xSemaphoreGive(si24r1_lock);

    /* -------- 3. 创建IP5305T电源管理任务 -------- 
     * 功能：定期读取电池状态信息
     * 周期：10秒
     * 优先级：中等
     */
    xTaskCreate((TaskFunction_t)IP5305T_Task,
                (char *)"ip5305t",
                (configSTACK_DEPTH_TYPE)IP5305T_TASK_STACK,
                (void *)NULL,
                (UBaseType_t)IP5305T_TASK_PRIORITY,
                (TaskHandle_t *)&ip5305t_task_handle);

    /* -------- 4. 创建电机控制任务 --------
     * 功能：根据设定的速度值控制四个电机
     * 周期：1秒
     * 优先级：中等
     */
    xTaskCreate((TaskFunction_t)Motor_Task,
                (char *)"motor",
                (configSTACK_DEPTH_TYPE)MOTOR_TASK_STACK,
                (void *)NULL,
                (UBaseType_t)MOTOR_TASK_PRIORITY,
                (TaskHandle_t *)&motor_task_handle);

    /* -------- 5. 创建LED指示任务 --------
     * 功能：周期性闪烁LED指示灯
     * 周期：1秒 (闪烁周期)
     * 优先级：最低
     */
    xTaskCreate((TaskFunction_t)LED_Task,
                (char *)"led",
                (configSTACK_DEPTH_TYPE)LED_TASK_STACK,
                (void *)NULL,
                (UBaseType_t)LED_TASK_PRIORITY,
                (TaskHandle_t *)&led_task_handle);

    /* -------- 6. 创建SI24R1接收任务 --------
     * 功能：轮询无线接收FIFO，收到数据后保存到si24r1_rx_buffer
     * 周期：10ms
     * 优先级：较高
     */
    xTaskCreate((TaskFunction_t)SI24R1_Rx_Task,
                (char *)"si24_rx",
                (configSTACK_DEPTH_TYPE)SI24R1_RX_TASK_STACK,
                (void *)NULL,
                (UBaseType_t)SI24R1_RX_TASK_PRIORITY,
                (TaskHandle_t *)&si24r1_rx_task_handle);

    /* -------- 7. 创建SI24R1发送任务 --------
     * 功能：周期发送32字节测试数据包
     * 周期：1秒
     * 优先级：中等
     */
    xTaskCreate((TaskFunction_t)SI24R1_Tx_Task,
                (char *)"si24_tx",
                (configSTACK_DEPTH_TYPE)SI24R1_TX_TASK_STACK,
                (void *)NULL,
                (UBaseType_t)SI24R1_TX_TASK_PRIORITY,
                (TaskHandle_t *)&si24r1_tx_task_handle);

    /* -------- 8. 启动FreeRTOS任务调度器 --------
     * 该函数启动任务调度，不会返回（除非发生错误）
     */
    vTaskStartScheduler();
}

/**
 * @brief LED指示灯控制任务
 * @details 
 * @param pvParameters FreeRTOS任务参数 (未使用)
 * @return 无 (任务无限循环运行)
 * @note 任务周期：1000ms
 */
void LED_Task(void *pvParameters)
{
    (void)pvParameters;
    uint8_t rc_blink = 0;
    uint8_t flight_blink = 0;

    while (1)
    {
        if (rc_connected)
        {
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
        }
        else
        {
            rc_blink ^= 1U;
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, rc_blink ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, rc_blink ? GPIO_PIN_SET : GPIO_PIN_RESET);
        }

        switch (flight_state)
        {
            case IDLE:
                flight_blink ^= 1U;
                HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, flight_blink ? GPIO_PIN_SET : GPIO_PIN_RESET);
                HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, flight_blink ? GPIO_PIN_SET : GPIO_PIN_RESET);
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;

            case NORMAL:
                flight_blink ^= 1U;
                HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, flight_blink ? GPIO_PIN_SET : GPIO_PIN_RESET);
                HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, flight_blink ? GPIO_PIN_SET : GPIO_PIN_RESET);
                vTaskDelay(pdMS_TO_TICKS(500));
                break;

            case FIX_HEIGHT:
                HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;

            case FAIL:
            default:
                HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;
        }
    }
}

void set_rc_connected(uint8_t connected)
{
    rc_connected = connected ? 1U : 0U;
}

/**
 * @brief SI24R1无线接收任务
 * @details 周期性读取无线数据包，收到有效数据后缓存并标记遥控连接
 * @param pvParameters FreeRTOS任务参数 (未使用)
 * @return 无 (任务无限循环运行)
 */
static void SI24R1_Rx_Task(void *pvParameters)
{
    u8 rxbuf[TX_PLOAD_WIDTH];
    u8 i;
    TickType_t last_rx_tick;

    (void)pvParameters;
    last_rx_tick = xTaskGetTickCount();

    while (1)
    {
        if (xSemaphoreTake(si24r1_lock, pdMS_TO_TICKS(5)) == pdTRUE)
        {
            if (SI24R1_RxPacket(rxbuf) == 0U)
            {
                for (i = 0; i < TX_PLOAD_WIDTH; i++)
                {
                    si24r1_rx_buffer[i] = rxbuf[i];
                }
                set_rc_connected(1U);
                last_rx_tick = xTaskGetTickCount();
            }
            xSemaphoreGive(si24r1_lock);
        }

        if ((xTaskGetTickCount() - last_rx_tick) >= pdMS_TO_TICKS(SI24R1_RX_LOST_TIMEOUT_MS))
        {
            set_rc_connected(0U);
        }

        vTaskDelay(pdMS_TO_TICKS(SI24R1_RX_TASK_PERIOD_MS));
    }
}

/**
 * @brief SI24R1无线发送任务
 * @details 周期性发送测试数据，发送后恢复到接收模式
 * @param pvParameters FreeRTOS任务参数 (未使用)
 * @return 无 (任务无限循环运行)
 */
static void SI24R1_Tx_Task(void *pvParameters)
{
    u8 txbuf[TX_PLOAD_WIDTH] = {0};
    u8 sequence = 0U;
    u8 i;

    (void)pvParameters;

    while (1)
    {
        txbuf[0] = 0xA5;
        txbuf[1] = sequence++;
        for (i = 2; i < TX_PLOAD_WIDTH; i++)
        {
            txbuf[i] = i;
        }

        if (xSemaphoreTake(si24r1_lock, pdMS_TO_TICKS(20)) == pdTRUE)
        {
            SI24R1_TX_Mode();
            si24r1_tx_result = SI24R1_TxPacket(txbuf);
            SI24R1_RX_Mode();
            xSemaphoreGive(si24r1_lock);
        }

        vTaskDelay(pdMS_TO_TICKS(SI24R1_TX_TASK_PERIOD_MS));
    }
}

/**
 * @brief 电机控制任务
 * @details 该任务周期性地控制四个电机的转速
 *          通过PWM信号调节电机速度，实现无人机的姿态控制和飞行
 * @param pvParameters FreeRTOS任务参数 (未使用)
 * @return 无 (任务无限循环运行)
 * @note 任务周期：1000ms (1秒)
 *       当前所有电机设置为固定速度200，实际应用中应从控制算法或遥控器获取
 * @warning 注意电机转速的同步性，避免无人机翻滚或倾斜
 */
void Motor_Task(void *pvParameters)
{
    (void)pvParameters;     /* 消除未使用参数警告 */

    while (1)
    {
        /* 
         * 控制四个电机的转速
         * 参数说明：
         * - 第一参数：电机结构体指针
         * - 第二参数：目标转速值 (0-255或0-100%)
         */
        motor_speed_control(&motor_left_0, 200);    /* 左侧电机0转速控制 */
        motor_speed_control(&motor_left_1, 200);    /* 左侧电机1转速控制 */
        motor_speed_control(&motor_right_0, 200);   /* 右侧电机0转速控制 */
        motor_speed_control(&motor_right_1, 200);   /* 右侧电机1转速控制 */
        
        /* 延迟1秒后再次执行转速控制 */
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
