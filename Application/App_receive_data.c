#include "App_receive_data.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

Remote_Data remote_data = {.thr = 0, .yaw = 500, .pit = 500, .rol = 500, .fix_height = 0, .shutdown = 0};
Remote_State remote_state = REMOTE_DISCONNECTED;
uint8_t rx_buff[TX_PLOAD_WIDTH] = {0};
uint8_t retry_count = 0;
Thr_state thr_state = FREE;
uint32_t max_enter_time = 0;
uint32_t min_enter_time = 0;

extern volatile Flight_State flight_state;
extern uint16_t fix_height;
extern uint8_t back_buff[TX_PLOAD_WIDTH];

//接收数据并解析为结构体
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

uint8_t App_receive_data(void)
{
    return App_ReceiveData();
}

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




