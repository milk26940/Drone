#ifndef APP_FLIGHT_H
#define APP_FLIGHT_H

#include "Com_config.h"
#include "Int_Motor.h"
#include "Int_mpu6050.h"

extern uint16_t fix_height;

// 飞控任务初始化，完成MPU6050校准和电机启动。
void App_flight_init(void);

// 根据MPU6050六轴数据解算欧拉角。
void App_flight_get_euler_angle(void);

// 根据姿态角和遥控器目标值计算串级PID。
void App_flight_pid_process(void);

// 根据飞行状态和PID输出控制四路电机。
void App_flight_control_motor(void);

// 定高模式下计算高度PID。
void App_flight_fix_height_pid_process(void);

#endif // APP_FLIGHT_H
