#include "App_receive_data.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

/* 遥控数据帧格式：
 * Byte0~2   : 固定帧头 's' 'g' 'g'
 * Byte3~4   : 油门 thr，高字节在前
 * Byte5~6   : 偏航 yaw，高字节在前
 * Byte7~8   : 俯仰 pit，高字节在前
 * Byte9~10  : 横滚 rol，高字节在前
 * Byte11    : 关机命令
 * Byte12    : 定高切换命令
 * Byte13~16 : 前 13 字节累加和，高字节在前
 */

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
    /* 每次接收前先清空缓冲区，避免上一次残留数据干扰校验。 */
    memset(rx_buff, 0, TX_PLOAD_WIDTH);

    /* 无线模块未收到完整数据包时，直接返回失败，由上层统计掉线。 */
    if (SI24R1_RxPacket(rx_buff) != 0)
    {
        return 1;
    }

    /* 收到遥控数据后立即回传遥测数据，然后切回接收模式。 */
    SI24R1_TX_Mode();
    for (uint8_t i = 0; i < 5; i++)
    {
        if (SI24R1_TxPacket(back_buff) == TX_DS)
        {
            break;
        }
    }
    SI24R1_RX_Mode();

    /* 1. 帧头校验：快速过滤掉无效包或错位数据。 */
    if (rx_buff[0] != FRAME_HEAD_CHECK_1 || rx_buff[1] != FRAME_HEAD_CHECK_2 || rx_buff[2] != FRAME_HEAD_CHECK_3)
    {
        return 1;
    }

    /* 2. 累加和校验：前 13 字节求和，与帧尾 4 字节校验值比较。 */
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
	
    /* 3. 数据解析：将原始字节流拆成遥控量与功能开关。 */
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
    /* 只要本次成功收到有效包，就立即恢复连接状态并清零失败计数。 */
    if (res == 0)
    {
        remote_state = REMOTE_CONNECTED;
        retry_count = 0;
    }
    else
    {
        /* 单次失败不立刻判定掉线，避免偶发丢包导致误触发保护。 */
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
    /* 解锁手势步骤：
     * 1. 油门拉到高位并保持至少 1s；
     * 2. 再将油门拉到低位并保持至少 1s；
     * 3. 满足后进入 UNLOCK，允许从 IDLE 切到 NORMAL。
     */
    switch (thr_state)
    {
    case FREE:
        /* 等待用户先将油门推到高位，作为解锁手势起点。 */
        if (remote_data.thr >= 900)
        {
            thr_state = MAX;
            max_enter_time = xTaskGetTickCount();
        }
        break;

    case MAX:
        /* 只有高位保持足够时长后离开高位，才进入下一阶段。 */
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
        /* 已经满足高位停留要求，接下来等待油门拉回低位。 */
        if (remote_data.thr <= 100)
        {
            thr_state = MIN;
            min_enter_time = xTaskGetTickCount();
        }
        break;

    case MIN:
        /* 低位保持不足 1s 时，如果用户又抬高油门，视为手势中断。 */
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
        /* 已解锁后保持状态，等待上层消费这次解锁结果。 */
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
    /* 状态机职责：
     * IDLE       : 等待完成解锁手势
     * NORMAL     : 普通姿态控制
     * FIX_HEIGHT : 在普通姿态控制基础上叠加定高
     * FAIL       : 遥控失联后的保护降落/停机过渡状态
     */
    switch (flight_state)
    {
    case IDLE:
        /* 只有完成解锁动作后，才允许从待机进入正常飞行。 */
        if (App_process_unlock() == 0)
        {
            flight_state = NORMAL;
            thr_state = FREE;
        }
        break;

    case NORMAL:
        /* 收到定高切换命令后，记录当前高度作为定高目标。 */
        if (remote_data.fix_height == 1)
        {
            flight_state = FIX_HEIGHT;
            remote_data.fix_height = 0;
            fix_height = Int_VL53L1X_GetDistance();
        }
        /* 遥控失联时立即进入失效保护。 */
        if (remote_state == REMOTE_DISCONNECTED)
        {
            flight_state = FAIL;
        }
        break;

    case FIX_HEIGHT:
        /* 再次收到定高命令时，退出定高并回到普通模式。 */
        if (remote_data.fix_height == 1)
        {
            flight_state = NORMAL;
            remote_data.fix_height = 0;
        }
        /* 定高模式同样受失联保护约束。 */
        if (remote_state == REMOTE_DISCONNECTED)
        {
            flight_state = FAIL;
        }
        break;

    case FAIL:
        /* 失联恢复后，要求油门回到底位，才允许退回待机态。 */
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




