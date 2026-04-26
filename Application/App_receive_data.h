#ifndef APP_RECEIVE_DATA_H
#define APP_RECEIVE_DATA_H

#include "Com_config.h"
#include "Int_SI24R1.h"

// 接收数据帧头校验值
#define FRAME_HEAD_CHECK_1 's'
#define FRAME_HEAD_CHECK_2 'g'
#define FRAME_HEAD_CHECK_3 'g'

// 最大重试次数
#define MAX_RETRY_TIMES 10

// Function prototypes
uint8_t App_ReceiveData(void);


void App_process_connect_state(uint8_t res);

#endif // APP_RECEIVE_DATA_H
