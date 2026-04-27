#ifndef __COM_CONFIG_H
#define __COM_CONFIG_H

#include <stdint.h>

/* 遥控链路连接状态。 */
typedef enum
{
    REMOTE_CONNECTED = 0,
    REMOTE_DISCONNECTED
} Remote_State;

/* 飞行模式状态机。 */
typedef enum
{
    IDLE = 0,
    NORMAL,
    FIX_HEIGHT,
    FAIL
} Flight_State;

/* 油门解锁动作的中间状态。 */
typedef enum
{
    FREE = 0,
    MAX,
    LEAVE_MAX,
    MIN,
    UNLOCK
} Thr_state;

/* 单帧遥控器数据内容。 */
typedef struct
{
    /* 油门通道。 */
    int16_t thr;
    /* 偏航通道。 */
    int16_t yaw;
    /* 俯仰通道。 */
    int16_t pit;
    /* 横滚通道。 */
    int16_t rol;
    /* 关机命令标志。 */
    uint8_t shutdown;
    /* 定高模式切换命令标志。 */
    uint8_t fix_height;
} Remote_Data;

#endif /* __COM_CONFIG_H */
