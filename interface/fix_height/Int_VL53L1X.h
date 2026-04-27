#ifndef __INT_VL53L1X__
#define __INT_VL53L1X__

#include "vl53l1_platform.h"
#include "VL53L1X_api.h"
#include "VL53L1X_calibration.h"
#include "FreeRTOS.h"
#include "task.h"

/**
 * @brief 初始化 VL53L1X 测距模块
 * @note 完成模块复位、参数配置并启动连续测距
 */
void Int_VL53L1X_Init(void);

/**
 * @brief 读取 VL53L1X 当前距离
 * @retval 当前距离值，单位为 mm
 */
uint16_t Int_VL53L1X_GetDistance(void);

#endif // __INT_VL53L1X__
