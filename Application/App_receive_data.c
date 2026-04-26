#include "app_receive_data.h"
#include <string.h>

Remote_Data remote_data = {.thr = 0, .yaw = 500, .pit = 500, .rol = 500, .fix_height = 0, .shutdown = 0};
Remote_State remote_state = REMOTE_DISCONNECTED;
uint8_t rx_buff[TX_PLOAD_WIDTH] = {0};
uint8_t retry_count = 0;

//接收数据并解析为结构体
uint8_t App_ReceiveData(void)
{
    memset(rx_buff, 0, TX_PLOAD_WIDTH);

    if (SI24R1_RxPacket(rx_buff) != 0)
    {
        return 1;
    }

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





