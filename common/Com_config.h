#ifndef __COM_CONFIG_H
#define __COM_CONFIG_H

#include <stdint.h>

typedef enum
{
    REMOTE_CONNECTED = 0,
    REMOTE_DISCONNECTED
} Remote_State;

/** 飞行状态 */
typedef enum
{
    IDLE = 0,
    NORMAL,
    FIX_HEIGHT,
    FAIL
} Flight_State;

/** 油门解锁状态 */
typedef enum
{
    FREE = 0,
    MAX,
    LEAVE_MAX,
    MIN,
    UNLOCK
} Thr_state;

/** 遥控器数据 */
typedef struct
{
    int16_t thr;
    int16_t yaw;
    int16_t pit;
    int16_t rol;
    uint8_t shutdown;
    uint8_t fix_height;
} Remote_Data;

#endif /* __COM_CONFIG_H */
