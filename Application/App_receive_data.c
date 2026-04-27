#include "App_receive_data.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

/* 当前解析后的遥控器输入数据。 */
Remote_Data remote_data = {.thr = 0, .yaw = 500, .pit = 500, .rol = 500, .fix_height = 0, .shutdown = 0};
/* 遥控链路连接状态。 */
Remote_State remote_state = REMOTE_DISCONNECTED;
/* 无线模块接收缓冲区。 */
uint8_t rx_buff[TX_PLOAD_WIDTH] = {0};
/* 连续接收失败计数，用于判定掉线。 */
uint8_t retry_count = 0;
/* 油门解锁状态机当前状态。 */
Thr_state thr_state = FREE;
/* 油门首次进入高位时的时间戳。 */
uint32_t max_enter_time = 0;
/* 油门首次进入低位时的时间戳。 */
uint32_t min_enter_time = 0;

extern volatile Flight_State flight_state;
extern uint16_t fix_height;
extern uint8_t back_buff[TX_PLOAD_WIDTH];

/**
 * @brief 接收并解析一帧遥控器数据
 * @retval 0 接收并校验成功
 * @retval 1 接收失败或校验失败
 * @note 成功接收后会立刻回传一帧遥测数据，再切回接收模式
 */
uint8_t App_ReceiveData(void)
{
    memset(rx_buff, 0, TX_PLOAD_WIDTH);

    if (SI24R1_RxPacket(rx_buff) != 0)
    {
        return 1;
    }

    /* 收到遥控数据后立即回传遥测数据，然后切回接收模式。 */
    SI24R1_TX_Mode();
    for (uint8_t i = 0; i < 5; i++)
    {
        if (SI24R1_TxPacket(back_buff) == 0)
        {
            break;
        }
    }
    SI24R1_RX_Mode();

    // 1. 帧头校验
    if (rx_buff[0] != FRAME_HEAD_CHECK_1 || rx_buff[1] != FRAME_HEAD_CHECK_2 || rx_buff[2] != FRAME_HEAD_CHECK_3)
    {
        return 1;
    }

    // 2. 校验和校验：前13字节求和，13~16字节为接收到的校验和，高位在前
    uint32_t sum = 0;
    uint32_t sum_receive = 0;

    for (uint8_t i = 0; i < 13; i++)
    {
        sum += rx_buff[i];
    }

    sum_receive = ((uint32_t)rx_buff[13] << 24) | ((uint32_t)rx_buff[14] << 16) | ((uint32_t)rx_buff[15] << 8) | rx_buff[16];

    if (sum != sum_receive)
    {
        return 1;
    }

    // 3. 数据解析
    remote_data.thr = (rx_buff[3] << 8) | rx_buff[4];
    remote_data.yaw = (rx_buff[5] << 8) | rx_buff[6];
    remote_data.pit = (rx_buff[7] << 8) | rx_buff[8];
    remote_data.rol = (rx_buff[9] << 8) | rx_buff[10];
    remote_data.shutdown = rx_buff[11];
    remote_data.fix_height = rx_buff[12];

    return 0;
}

/**
 * @brief App_ReceiveData 的兼容封装
 * @retval 0 接收成功
 * @retval 1 接收失败
 */
uint8_t App_receive_data(void)
{
    return App_ReceiveData();
}

/**
 * @brief 根据本次接收结果更新遥控连接状态
 * @param res 本次接收结果，0 表示成功，非 0 表示失败
 * @note 连续失败达到阈值后才判定为掉线，避免偶发丢包导致误判
 */
void App_process_connect_state(uint8_t res)
{
    if (res == 0)
    {
        remote_state = REMOTE_CONNECTED;
        retry_count = 0;
    }
    else
    {
        retry_count++;
        if (retry_count >= MAX_RETRY_TIMES)
        {
            remote_state = REMOTE_DISCONNECTED;
            retry_count = 0;
        }
    }
}

/**
 * @brief 油门解锁状态机
 * @retval 0 已完成解锁
 * @retval 1 尚未解锁
 * @note 解锁手势为油门持续高位后切到低位，并分别保持一定时间
 */
static uint8_t App_process_unlock(void)
{
    switch (thr_state)
    {
    case FREE:
        if (remote_data.thr >= 900)
        {
            thr_state = MAX;
            max_enter_time = xTaskGetTickCount();
        }
        break;

    case MAX:
        if (remote_data.thr < 900)
        {
            if ((xTaskGetTickCount() - max_enter_time) >= pdMS_TO_TICKS(1000))
            {
                thr_state = LEAVE_MAX;
            }
            else
            {
                thr_state = FREE;
            }
        }
        break;

    case LEAVE_MAX:
        if (remote_data.thr <= 100)
        {
            thr_state = MIN;
            min_enter_time = xTaskGetTickCount();
        }
        break;

    case MIN:
        if ((xTaskGetTickCount() - min_enter_time) <= pdMS_TO_TICKS(1000))
        {
            if (remote_data.thr > 100)
            {
                thr_state = FREE;
            }
        }
        else
        {
            thr_state = UNLOCK;
        }
        break;

    case UNLOCK:
    default:
        break;
    }

    return (thr_state == UNLOCK) ? 0U : 1U;
}

/**
 * @brief 根据遥控状态和飞行条件更新飞行状态机
 * @note 负责处理待机、普通飞行、定高和失控保护之间的切换
 */
void App_process_flight_state(void)
{
    switch (flight_state)
    {
    case IDLE:
        if (App_process_unlock() == 0)
        {
            flight_state = NORMAL;
            thr_state = FREE;
        }
        break;

    case NORMAL:
        if (remote_data.fix_height == 1)
        {
            flight_state = FIX_HEIGHT;
            remote_data.fix_height = 0;
            fix_height = Int_VL53L1X_GetDistance();
        }
        if (remote_state == REMOTE_DISCONNECTED)
        {
            flight_state = FAIL;
        }
        break;

    case FIX_HEIGHT:
        if (remote_data.fix_height == 1)
        {
            flight_state = NORMAL;
            remote_data.fix_height = 0;
        }
        if (remote_state == REMOTE_DISCONNECTED)
        {
            flight_state = FAIL;
        }
        break;

    case FAIL:
        if (remote_state == REMOTE_CONNECTED && remote_data.thr <= 100)
        {
            flight_state = IDLE;
        }
        break;

    default:
        flight_state = IDLE;
        break;
    }
}




