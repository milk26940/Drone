#ifndef INT_MOTOR_H
#define INT_MOTOR_H

#include "main.h"

typedef enum
{
    LEFT_0 = 0,
    LEFT_1,
    RIGHT_0,
    RIGHT_1
} MotorId_t;

typedef struct
{
    MotorId_t id;           // 电机编号
    TIM_HandleTypeDef *htim; // 定时器句柄
    uint16_t channel;        // 定时器通道
    int16_t speed;           // 电机速度，范围 0 到 1000
} MotorControl_t;

void motor_start(MotorControl_t *motor);
void motor_stop(MotorControl_t *motor);
void motor_speed_control(MotorControl_t *motor, int16_t speed);


#endif // INT_MOTOR_H
