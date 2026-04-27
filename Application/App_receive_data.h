#ifndef APP_RECEIVE_DATA_H
#define APP_RECEIVE_DATA_H

#include "Com_config.h"
#include "Int_SI24R1.h"
#include "Int_VL53L1X.h"

/* 遥控数据帧头固定标识。 */
#define FRAME_HEAD_CHECK_1 's'
#define FRAME_HEAD_CHECK_2 'g'
#define FRAME_HEAD_CHECK_3 'g'

/* 判定遥控掉线前允许的最大连续失败次数。 */
#define MAX_RETRY_TIMES 10

/* 接收并解析一帧遥控数据。 */
uint8_t App_ReceiveData(void);
/* App_ReceiveData 的兼容封装接口。 */
uint8_t App_receive_data(void);

/* 根据接收结果更新遥控连接状态。 */
void App_process_connect_state(uint8_t res);
/* 根据当前输入与连接状态推进飞行状态机。 */
void App_process_flight_state(void);

#endif // APP_RECEIVE_DATA_H
