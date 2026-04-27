#ifndef __INT_MPU6050_H
#define __INT_MPU6050_H

#include "main.h"
#include "i2c.h"

/* MPU6050 器件地址与寄存器地址定义 */
#define MPU6050_ADDRESS        (0x68U << 1)
#define MPU6050_SMPLRT_DIV     0x19U
#define MPU6050_CONFIG         0x1AU
#define MPU6050_GYRO_CONFIG    0x1BU
#define MPU6050_ACCEL_CONFIG   0x1CU
#define MPU6050_WHO_AM_I       0x75U
#define MPU6050_PWR_MGMT_1     0x6BU
#define MPU6050_PWR_MGMT_2     0x6CU
#define MPU6050_ACCEL_XOUT_H   0x3BU
#define MPU6050_ACCEL_XOUT_L   0x3CU
#define MPU6050_ACCEL_YOUT_H   0x3DU
#define MPU6050_ACCEL_YOUT_L   0x3EU
#define MPU6050_ACCEL_ZOUT_H   0x3FU
#define MPU6050_ACCEL_ZOUT_L   0x40U
#define MPU6050_GYRO_XOUT_H    0x43U
#define MPU6050_GYRO_XOUT_L    0x44U
#define MPU6050_GYRO_YOUT_H    0x45U
#define MPU6050_GYRO_YOUT_L    0x46U
#define MPU6050_GYRO_ZOUT_H    0x47U
#define MPU6050_GYRO_ZOUT_L    0x48U

/* 三轴加速度原始数据 */
typedef struct
{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
} Accel_struct;

/* 三轴角速度原始数据 */
typedef struct
{
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} Gyro_struct;

/* 组合后的六轴原始数据 */
typedef struct
{
    Accel_struct accel;
    Gyro_struct gyro;
} Gyro_Accel_Struct;

/* 零偏补偿量，由校准函数计算后更新 */
extern int16_t acc_x_offset;
extern int16_t acc_y_offset;
extern int16_t acc_z_offset;
extern int16_t gyro_x_offset;
extern int16_t gyro_y_offset;
extern int16_t gyro_z_offset;

/* 兼容保留接口 */
void MPU6050_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT);
/* 寄存器单字节写入 */
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);
/* 寄存器单字节读取 */
uint8_t MPU6050_ReadReg(uint8_t RegAddress);
/* 初始化传感器 */
void MPU6050_Init(void);
/* 读取传感器 ID */
uint8_t MPU6050_GetID(void);
/* 读取原始六轴数据 */
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);
/* 仅读取加速度数据 */
void Int_MPU6050_Get_Acc(Accel_struct *accel);
/* 读取并补偿后的六轴数据 */
void Int_MPU6050_Get_Data(Gyro_Accel_Struct *data);
/* 执行静止零偏校准 */
void Int_MPU6050_calculate_offset(void);

#endif /* __INT_MPU6050_H */
