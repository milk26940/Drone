#include "Int_Motor.h"

void motor_start(MotorControl_t *motor)
{
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, 200); // 设置初始速度为200
    HAL_TIM_PWM_Start(motor->htim, motor->channel);
}

void motor_stop(MotorControl_t *motor)
{
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, 0);
    HAL_TIM_PWM_Stop(motor->htim, motor->channel);
}

void motor_speed_control(MotorControl_t *motor, int16_t speed)
{
    if (speed < 0)
    {
        speed = 0;
    }
    else if (speed > 1000)
    {
        speed = 1000;
    }

    motor->speed = speed;
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, (uint16_t)speed);
}

