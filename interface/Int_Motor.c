/**
 * @file    Int_Motor.c
 * @brief   电机控制接口实现
 * @details 封装电机 PWM 启停与占空比调节逻辑
 */

#include "Int_Motor.h"

/**
 * @brief 启动指定电机
 * @param motor 电机控制对象指针
 * @note  启动时先写入一个初始占空比，避免电机完全无响应
 */
void motor_start(MotorControl_t *motor)
{
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, 200); // 设置初始速度为200
    HAL_TIM_PWM_Start(motor->htim, motor->channel);
}

/**
 * @brief 停止指定电机
 * @param motor 电机控制对象指针
 * @note  先将比较值清零，再关闭 PWM 输出
 */
void motor_stop(MotorControl_t *motor)
{
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, 0);
    HAL_TIM_PWM_Stop(motor->htim, motor->channel);
}

/**
 * @brief 控制电机速度
 * @param motor 电机控制对象指针
 * @param speed 目标速度，范围限制为 0~1000
 * @note  超出范围的输入会被自动钳制
 */
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

