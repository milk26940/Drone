#ifndef INT_MOTOR_H
#define INT_MOTOR_H

#include "main.h"

/**
 * @brief 电机编号枚举
 * @note  用于区分四路电机控制对象
 */
typedef enum
{
    LEFT_0 = 0,
    LEFT_1,
    RIGHT_0,
    RIGHT_1
} MotorId_t;

/**
 * @brief 电机控制结构体
 * @details 保存电机对应的定时器、通道及当前速度
 */
typedef struct
{
    MotorId_t id;           // 电机编号
    TIM_HandleTypeDef *htim; // 定时器句柄
    uint16_t channel;        // 定时器通道
    int16_t speed;           // 电机速度，范围 0 到 1000
} MotorControl_t;

/* 启动电机 PWM 输出 */
void motor_start(MotorControl_t *motor);
/* 停止电机 PWM 输出 */
void motor_stop(MotorControl_t *motor);
/* 设置电机速度 */
void motor_speed_control(MotorControl_t *motor, int16_t speed);


#endif // INT_MOTOR_H
