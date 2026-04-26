#include "App_flight.h"

#include "tim.h"

#include <math.h>
#include <stdint.h>

#define APP_FLIGHT_PID_PERIOD 0.006f
#define APP_FLIGHT_LOW_PASS_ALPHA 0.15f
#define APP_FLIGHT_GYRO_DPS_SCALE (4000.0f / 65536.0f)
#define APP_FLIGHT_GYRO_RAD_SCALE (APP_FLIGHT_GYRO_DPS_SCALE / 180.0f * 3.1415926f)
#define APP_FLIGHT_RAD_TO_DEG 57.2957795f
#define APP_FLIGHT_MOTOR_MAX_SPEED 700
#define APP_FLIGHT_MOTOR_MIN_SPEED 0
#define APP_FLIGHT_SAFE_THR 50

typedef struct
{
    float yaw;
    float pitch;
    float roll;
} Euler_struct;

typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
} AppFlightQuaternion_t;

typedef struct
{
    float last_p;
    float now_p;
    float out;
    float kg;
    float q;
    float r;
} AppFlightKalman_t;

typedef struct
{
    float kp;
    float ki;
    float kd;
    float err;
    float desire;
    float measure;
    float last_err;
    float integral;
    float output;
} AppFlightPid_t;

extern Remote_Data remote_data;
extern volatile Flight_State flight_state;

static Gyro_Accel_Struct gyro_accel_data = {0};
static Euler_struct euler_angle = {0};
static Gyro_struct last_gyro = {0};
uint16_t fix_height = 0U;

static MotorControl_t left_top_motor = {LEFT_0, &htim3, TIM_CHANNEL_1, 0};
static MotorControl_t left_bottom_motor = {LEFT_1, &htim4, TIM_CHANNEL_4, 0};
static MotorControl_t right_top_motor = {RIGHT_0, &htim2, TIM_CHANNEL_2, 0};
static MotorControl_t right_bottom_motor = {RIGHT_1, &htim1, TIM_CHANNEL_3, 0};

// PID调参时先调内环角速度，再调外环姿态角。
static AppFlightPid_t pitch_pid = {.kp = -7.00f, .ki = 0.00f, .kd = 0.00f};
static AppFlightPid_t gyro_y_pid = {.kp = 3.00f, .ki = 0.00f, .kd = 0.50f};
static AppFlightPid_t roll_pid = {.kp = -7.00f, .ki = 0.00f, .kd = 0.00f};
static AppFlightPid_t gyro_x_pid = {.kp = 3.00f, .ki = 0.00f, .kd = 0.50f};
static AppFlightPid_t yaw_pid = {.kp = -3.00f, .ki = 0.00f, .kd = 0.00f};
static AppFlightPid_t gyro_z_pid = {.kp = -5.00f, .ki = 0.00f, .kd = 0.00f};
static AppFlightPid_t height_pid = {.kp = -0.60f, .ki = 0.00f, .kd = -0.20f};

static AppFlightKalman_t accel_kfs[3] = {
    {0.02f, 0.0f, 0.0f, 0.0f, 0.001f, 0.543f},
    {0.02f, 0.0f, 0.0f, 0.0f, 0.001f, 0.543f},
    {0.02f, 0.0f, 0.0f, 0.0f, 0.001f, 0.543f},
};

static int16_t App_Flight_LowPass(int16_t new_value, int16_t prev_value);
static double App_Flight_KalmanFilter(AppFlightKalman_t *kf, double input);
static void App_Flight_CalcPid(AppFlightPid_t *pid);
static void App_Flight_CalcPidChain(AppFlightPid_t *out_pid, AppFlightPid_t *in_pid);
static int16_t App_Flight_LimitFloat(float value, int16_t max_value, int16_t min_value);
static void App_Flight_GetEulerAngle(Gyro_Accel_Struct *gyro_accel, Euler_struct *euler, float dt);
static uint16_t App_Flight_ReadHeight(void);
static void App_Flight_SetMotorSpeed(MotorControl_t *motor, int16_t speed);
static float App_Flight_Qrsqrt(float number);

void App_flight_init(void)
{
    MPU6050_Init();
    Int_MPU6050_calculate_offset();

    motor_start(&left_top_motor);
    motor_start(&left_bottom_motor);
    motor_start(&right_top_motor);
    motor_start(&right_bottom_motor);

    fix_height = 0U;
}

void App_flight_get_euler_angle(void)
{
    Int_MPU6050_Get_Data(&gyro_accel_data);

    gyro_accel_data.gyro.gyro_x = App_Flight_LowPass(gyro_accel_data.gyro.gyro_x, last_gyro.gyro_x);
    gyro_accel_data.gyro.gyro_y = App_Flight_LowPass(gyro_accel_data.gyro.gyro_y, last_gyro.gyro_y);
    gyro_accel_data.gyro.gyro_z = App_Flight_LowPass(gyro_accel_data.gyro.gyro_z, last_gyro.gyro_z);
    last_gyro = gyro_accel_data.gyro;

    gyro_accel_data.accel.accel_x = (int16_t)App_Flight_KalmanFilter(&accel_kfs[0], gyro_accel_data.accel.accel_x);
    gyro_accel_data.accel.accel_y = (int16_t)App_Flight_KalmanFilter(&accel_kfs[1], gyro_accel_data.accel.accel_y);
    gyro_accel_data.accel.accel_z = (int16_t)App_Flight_KalmanFilter(&accel_kfs[2], gyro_accel_data.accel.accel_z);

    App_Flight_GetEulerAngle(&gyro_accel_data, &euler_angle, APP_FLIGHT_PID_PERIOD);
}

void App_flight_pid_process(void)
{
    pitch_pid.desire = (remote_data.pit - 500) / 50.0f;
    pitch_pid.measure = euler_angle.pitch;
    gyro_y_pid.measure = gyro_accel_data.gyro.gyro_y * APP_FLIGHT_GYRO_DPS_SCALE;
    App_Flight_CalcPidChain(&pitch_pid, &gyro_y_pid);

    roll_pid.desire = (remote_data.rol - 500) / 50.0f;
    roll_pid.measure = euler_angle.roll;
    gyro_x_pid.measure = gyro_accel_data.gyro.gyro_x * APP_FLIGHT_GYRO_DPS_SCALE;
    App_Flight_CalcPidChain(&roll_pid, &gyro_x_pid);

    yaw_pid.desire = (remote_data.yaw - 500) / 50.0f;
    yaw_pid.measure = euler_angle.yaw;
    gyro_z_pid.measure = gyro_accel_data.gyro.gyro_z * APP_FLIGHT_GYRO_DPS_SCALE;
    App_Flight_CalcPidChain(&yaw_pid, &gyro_z_pid);
}

void App_flight_control_motor(void)
{
    const int16_t yaw_output = App_Flight_LimitFloat(gyro_z_pid.output, 100, -100);

    switch (flight_state)
    {
    case IDLE:
        left_top_motor.speed = 0;
        left_bottom_motor.speed = 0;
        right_top_motor.speed = 0;
        right_bottom_motor.speed = 0;
        break;

    case NORMAL:
        left_top_motor.speed = App_Flight_LimitFloat(remote_data.thr + gyro_y_pid.output - gyro_x_pid.output + yaw_output,
                                                     APP_FLIGHT_MOTOR_MAX_SPEED, APP_FLIGHT_MOTOR_MIN_SPEED);
        left_bottom_motor.speed = App_Flight_LimitFloat(remote_data.thr - gyro_y_pid.output - gyro_x_pid.output - yaw_output,
                                                        APP_FLIGHT_MOTOR_MAX_SPEED, APP_FLIGHT_MOTOR_MIN_SPEED);
        right_top_motor.speed = App_Flight_LimitFloat(remote_data.thr + gyro_y_pid.output + gyro_x_pid.output - yaw_output,
                                                      APP_FLIGHT_MOTOR_MAX_SPEED, APP_FLIGHT_MOTOR_MIN_SPEED);
        right_bottom_motor.speed = App_Flight_LimitFloat(remote_data.thr - gyro_y_pid.output + gyro_x_pid.output + yaw_output,
                                                         APP_FLIGHT_MOTOR_MAX_SPEED, APP_FLIGHT_MOTOR_MIN_SPEED);
        break;

    case FIX_HEIGHT:
        left_top_motor.speed = App_Flight_LimitFloat(remote_data.thr + gyro_y_pid.output - gyro_x_pid.output + yaw_output + height_pid.output,
                                                     APP_FLIGHT_MOTOR_MAX_SPEED, APP_FLIGHT_MOTOR_MIN_SPEED);
        left_bottom_motor.speed = App_Flight_LimitFloat(remote_data.thr - gyro_y_pid.output - gyro_x_pid.output - yaw_output + height_pid.output,
                                                        APP_FLIGHT_MOTOR_MAX_SPEED, APP_FLIGHT_MOTOR_MIN_SPEED);
        right_top_motor.speed = App_Flight_LimitFloat(remote_data.thr + gyro_y_pid.output + gyro_x_pid.output - yaw_output + height_pid.output,
                                                      APP_FLIGHT_MOTOR_MAX_SPEED, APP_FLIGHT_MOTOR_MIN_SPEED);
        right_bottom_motor.speed = App_Flight_LimitFloat(remote_data.thr - gyro_y_pid.output + gyro_x_pid.output + yaw_output + height_pid.output,
                                                         APP_FLIGHT_MOTOR_MAX_SPEED, APP_FLIGHT_MOTOR_MIN_SPEED);
        break;

    case FAIL:
        left_top_motor.speed = App_Flight_LimitFloat(left_top_motor.speed - 2, APP_FLIGHT_MOTOR_MAX_SPEED, APP_FLIGHT_MOTOR_MIN_SPEED);
        left_bottom_motor.speed = App_Flight_LimitFloat(left_bottom_motor.speed - 2, APP_FLIGHT_MOTOR_MAX_SPEED, APP_FLIGHT_MOTOR_MIN_SPEED);
        right_top_motor.speed = App_Flight_LimitFloat(right_top_motor.speed - 2, APP_FLIGHT_MOTOR_MAX_SPEED, APP_FLIGHT_MOTOR_MIN_SPEED);
        right_bottom_motor.speed = App_Flight_LimitFloat(right_bottom_motor.speed - 2, APP_FLIGHT_MOTOR_MAX_SPEED, APP_FLIGHT_MOTOR_MIN_SPEED);
        if (left_top_motor.speed == 0 && left_bottom_motor.speed == 0 &&
            right_top_motor.speed == 0 && right_bottom_motor.speed == 0)
        {
            flight_state = IDLE;
        }
        break;

    default:
        break;
    }

    // 油门过低时强制关停电机，避免误转。
    if (remote_data.thr < APP_FLIGHT_SAFE_THR)
    {
        left_top_motor.speed = 0;
        left_bottom_motor.speed = 0;
        right_top_motor.speed = 0;
        right_bottom_motor.speed = 0;
    }

    App_Flight_SetMotorSpeed(&left_top_motor, left_top_motor.speed);
    App_Flight_SetMotorSpeed(&left_bottom_motor, left_bottom_motor.speed);
    App_Flight_SetMotorSpeed(&right_top_motor, right_top_motor.speed);
    App_Flight_SetMotorSpeed(&right_bottom_motor, right_bottom_motor.speed);
}

void App_flight_fix_height_pid_process(void)
{
    height_pid.desire = fix_height;
    height_pid.measure = App_Flight_ReadHeight();
    App_Flight_CalcPid(&height_pid);
}

static int16_t App_Flight_LowPass(int16_t new_value, int16_t prev_value)
{
    return (int16_t)(APP_FLIGHT_LOW_PASS_ALPHA * new_value + (1.0f - APP_FLIGHT_LOW_PASS_ALPHA) * prev_value);
}

static double App_Flight_KalmanFilter(AppFlightKalman_t *kf, double input)
{
    kf->now_p = kf->last_p + kf->q;
    kf->kg = kf->now_p / (kf->now_p + kf->r);
    kf->out = kf->out + kf->kg * (input - kf->out);
    kf->last_p = (1.0f - kf->kg) * kf->now_p;
    return kf->out;
}

static void App_Flight_CalcPid(AppFlightPid_t *pid)
{
    float derivative;

    pid->err = pid->measure - pid->desire;
    pid->integral += pid->err;

    if (pid->last_err == 0.0f)
    {
        pid->last_err = pid->err;
    }

    derivative = pid->err - pid->last_err;
    pid->output = pid->kp * pid->err +
                  pid->ki * pid->integral * APP_FLIGHT_PID_PERIOD +
                  pid->kd * derivative / APP_FLIGHT_PID_PERIOD;
    pid->last_err = pid->err;
}

static void App_Flight_CalcPidChain(AppFlightPid_t *out_pid, AppFlightPid_t *in_pid)
{
    App_Flight_CalcPid(out_pid);
    in_pid->desire = out_pid->output;
    App_Flight_CalcPid(in_pid);
}

static int16_t App_Flight_LimitFloat(float value, int16_t max_value, int16_t min_value)
{
    if (value > max_value)
    {
        return max_value;
    }
    if (value < min_value)
    {
        return min_value;
    }
    return (int16_t)value;
}

static void App_Flight_GetEulerAngle(Gyro_Accel_Struct *gyro_accel, Euler_struct *euler, float dt)
{
    typedef struct
    {
        float x;
        float y;
        float z;
    } Vector3f_t;

    static Vector3f_t gyro_integral_error = {0.0f, 0.0f, 0.0f};
    static AppFlightQuaternion_t q = {1.0f, 0.0f, 0.0f, 0.0f};
    const float kp = 0.8f;
    const float ki = 0.0003f;
    const float half_time = dt * 0.5f;
    Vector3f_t gravity;
    Vector3f_t acc;
    Vector3f_t gyro;
    Vector3f_t acc_gravity;
    float norm;
    float q0_delta;
    float q1_delta;
    float q2_delta;
    float q3_delta;
    float vec_x_z;
    float vec_y_z;
    float vec_z_z;
    float yaw_rate;

    gravity.x = 2.0f * (q.q1 * q.q3 - q.q0 * q.q2);
    gravity.y = 2.0f * (q.q0 * q.q1 + q.q2 * q.q3);
    gravity.z = 1.0f - 2.0f * (q.q1 * q.q1 + q.q2 * q.q2);

    norm = App_Flight_Qrsqrt((float)gyro_accel->accel.accel_x * gyro_accel->accel.accel_x +
                             (float)gyro_accel->accel.accel_y * gyro_accel->accel.accel_y +
                             (float)gyro_accel->accel.accel_z * gyro_accel->accel.accel_z);
    if (norm <= 0.0f)
    {
        return;
    }

    acc.x = gyro_accel->accel.accel_x * norm;
    acc.y = gyro_accel->accel.accel_y * norm;
    acc.z = gyro_accel->accel.accel_z * norm;

    acc_gravity.x = acc.y * gravity.z - acc.z * gravity.y;
    acc_gravity.y = acc.z * gravity.x - acc.x * gravity.z;
    acc_gravity.z = acc.x * gravity.y - acc.y * gravity.x;

    gyro_integral_error.x += acc_gravity.x * ki;
    gyro_integral_error.y += acc_gravity.y * ki;
    gyro_integral_error.z += acc_gravity.z * ki;

    gyro.x = gyro_accel->gyro.gyro_x * APP_FLIGHT_GYRO_RAD_SCALE + kp * acc_gravity.x + gyro_integral_error.x;
    gyro.y = gyro_accel->gyro.gyro_y * APP_FLIGHT_GYRO_RAD_SCALE + kp * acc_gravity.y + gyro_integral_error.y;
    gyro.z = gyro_accel->gyro.gyro_z * APP_FLIGHT_GYRO_RAD_SCALE + kp * acc_gravity.z + gyro_integral_error.z;

    q0_delta = (-q.q1 * gyro.x - q.q2 * gyro.y - q.q3 * gyro.z) * half_time;
    q1_delta = (q.q0 * gyro.x - q.q3 * gyro.y + q.q2 * gyro.z) * half_time;
    q2_delta = (q.q3 * gyro.x + q.q0 * gyro.y - q.q1 * gyro.z) * half_time;
    q3_delta = (-q.q2 * gyro.x + q.q1 * gyro.y + q.q0 * gyro.z) * half_time;

    q.q0 += q0_delta;
    q.q1 += q1_delta;
    q.q2 += q2_delta;
    q.q3 += q3_delta;

    norm = App_Flight_Qrsqrt(q.q0 * q.q0 + q.q1 * q.q1 + q.q2 * q.q2 + q.q3 * q.q3);
    if (norm <= 0.0f)
    {
        return;
    }
    q.q0 *= norm;
    q.q1 *= norm;
    q.q2 *= norm;
    q.q3 *= norm;

    vec_x_z = 2.0f * q.q0 * q.q2 - 2.0f * q.q1 * q.q3;
    vec_y_z = 2.0f * q.q2 * q.q3 + 2.0f * q.q0 * q.q1;
    vec_z_z = 1.0f - 2.0f * q.q1 * q.q1 - 2.0f * q.q2 * q.q2;

    yaw_rate = gyro_accel->gyro.gyro_z * APP_FLIGHT_GYRO_DPS_SCALE;
    if (yaw_rate > 0.5f || yaw_rate < -0.5f)
    {
        euler->yaw += yaw_rate * dt;
    }

    euler->pitch = asinf(vec_x_z) * APP_FLIGHT_RAD_TO_DEG;
    euler->roll = atan2f(vec_y_z, vec_z_z) * APP_FLIGHT_RAD_TO_DEG;
}

static uint16_t App_Flight_ReadHeight(void)
{
    // 当前项目还没有接入VL53L1X测距模块，保留定高接口默认返回0。
    return 0U;
}

static void App_Flight_SetMotorSpeed(MotorControl_t *motor, int16_t speed)
{
    motor_speed_control(motor, App_Flight_LimitFloat(speed, APP_FLIGHT_MOTOR_MAX_SPEED, APP_FLIGHT_MOTOR_MIN_SPEED));
}

static float App_Flight_Qrsqrt(float number)
{
    union
    {
        float f;
        uint32_t u;
    } conv;
    const float three_halfs = 1.5f;
    float x2;

    if (number <= 0.0f)
    {
        return 0.0f;
    }

    x2 = number * 0.5f;
    conv.f = number;
    conv.u = 0x5f3759dfU - (conv.u >> 1);
    conv.f = conv.f * (three_halfs - (x2 * conv.f * conv.f));
    return conv.f;
}
