#ifndef __COM_PID__
#define __COM_PID__

#include "main.h"
/* PID 计算周期，需与飞控控制周期保持一致。 */
#define PID_PERIOD 0.006

/* PID 控制器运行时参数与状态。 */
typedef struct
{
    float kp;       // 比例部分  值越大响应速度越快
    float ki;       // 积分部分  解决稳态误差  无人机控制中  积分项一般不使用
    float kd;       // 微分部分  值越大 抑制效果越好 解决过调震荡
    float err;      // 误差值
    float desire;   // 目标值
    float measure;  // 测量值
    float last_err; // 上一次的误差
    float integral; // 积分累积
    float output;   // 输出结果
} PID_Struct;

/* 执行一次位置式 PID 计算。 */
void Com_PID_Calc(PID_Struct *pid);

/* 依次执行外环和内环的串级 PID 计算。 */
void Com_PID_Calc_Chain(PID_Struct *out_pid, PID_Struct *in_pid);

/**
 * @brief 限制数值在正常的范围内
 *
 * @param speed
 * @param max_speed
 * @param min_speed
 * @return int16_t
 */
int16_t Com_limit(int16_t speed, int16_t max_speed, int16_t min_speed);

#endif // __COM_PID__
