#ifndef __COMMON_IMU_H
#define __COMMON_IMU_H

#include "Com_debug.h"
#include "Com_Config.h"
#include "Int_mpu6050.h"
#include "math.h"

/* 表示四元数的结构体 */
typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
} Quaternion_Struct;

/* 解算得到的欧拉角 */
typedef struct
{
    float yaw;
    float pitch;
    float roll;
} Euler_struct;

/* 弧度转角度系数。 */
extern float RtA;
/* 原始陀螺仪数据转换为角速度后的比例系数，单位为度每秒。 */
extern float Gyro_G;
/* 原始陀螺仪数据转换为角速度后的比例系数，单位为弧度每秒。 */
extern float Gyro_Gr;

/* 根据六轴原始数据解算欧拉角姿态。 */
void Common_IMU_GetEulerAngle(Gyro_Accel_Struct *gyroAccel,
                              Euler_struct *eulerAngle,
                              float dt);
/* 获取归一化后的 Z 轴加速度分量。 */
float Common_IMU_GetNormAccZ(void);

#endif
