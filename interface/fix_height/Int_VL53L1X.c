#include "Int_VL53L1X.h"
#include <stdio.h>

/* VL53L1X 默认 8 位 I2C 设备地址。 */
uint16_t dev = 0x52;

/**
 * @brief 初始化 VL53L1X 测距模块
 * @note 复位模块后读取基础识别信息，并按当前应用需求配置测距参数
 */
void Int_VL53L1X_Init(void)
{
    // 通过 XSHUT 引脚复位传感器，确保寄存器状态干净。
    HAL_GPIO_WritePin(VX_XSHUT_GPIO_Port, VX_XSHUT_Pin, GPIO_PIN_RESET);
    vTaskDelay(2);
    HAL_GPIO_WritePin(VX_XSHUT_GPIO_Port, VX_XSHUT_Pin, GPIO_PIN_SET);
    vTaskDelay(2);

    uint8_t byteData = 0;
    uint16_t wordData = 0;
    uint8_t sensorState = 0;
    /* 读取基础识别寄存器，便于确认 I2C 通信和模块型号正常。 */
    VL53L1_RdByte(dev, 0x010F, &byteData);
    printf("VL53L1X Model_ID: %X\n", byteData);
    VL53L1_RdByte(dev, 0x0110, &byteData);
    printf("VL53L1X Module_Type: %X\n", byteData);
    VL53L1_RdWord(dev, 0x010F, &wordData);
    printf("VL53L1X: %X\n", wordData);
    while (sensorState == 0)
    {
        VL53L1X_BootState(dev, &sensorState);
        vTaskDelay(2);
    }
    printf("Chip booted\n");

    /* 使用官方默认参数完成内部初始化。 */
    VL53L1X_SensorInit(dev);
    /* 按当前飞控需求配置长距离模式和 20ms 测距周期。 */
    VL53L1X_SetDistanceMode(dev, 2);          /* 1=short, 2=long */
    VL53L1X_SetTimingBudgetInMs(dev, 20);     /* in ms possible values [20, 50, 100, 200, 500] */
    VL53L1X_SetInterMeasurementInMs(dev, 20); /* in ms, IM must be > = TB */
                                              //  status = VL53L1X_SetOffset(dev,20); /* offset compensation in mm */
                                              //  status = VL53L1X_SetROI(dev, 16, 16); /* minimum ROI 4,4 */
                                              //	status = VL53L1X_CalibrateOffset(dev, 140, &offset); /* may take few second to perform the offset cal*/
                                              //	status = VL53L1X_CalibrateXtalk(dev, 1000, &xtalk); /* may take few second to perform the xtalk cal */
    printf("VL53L1X Ultra Lite Driver Example running ...\n");
    VL53L1X_StartRanging(dev); /* This function has to be called to enable the ranging */
}

/**
 * @brief 读取 VL53L1X 当前距离
 * @retval 当前测得距离，单位为 mm
 */
uint16_t Int_VL53L1X_GetDistance(void)
{
    uint16_t Distance = 0;
    VL53L1X_GetDistance(dev, &Distance);
    return Distance;
}
