/*
 * pid.h
 *
 *  Created on: Apr 5, 2025
 *      Author: fabbo
 */

#ifndef __PID_H
#define __PID_H

typedef struct {
    struct {
        float x;
        float y;
        float z;
    } accel;     // 加速度数据
    struct {
        float x;
        float y;
        float z;
    } gyro;      // 角速度数据
    struct {
        float roll;
        float pitch;
        float yaw;
    } angle;     // 姿态角数据
} SensorData;

typedef struct {
    float wheel_L;  // 左前轮速度
    float wheel_R;  // 右前轮速度
} WheelSpeeds;

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float error;
    float error_last;
    float error_sum;
    float error_sum_max;
} PID_t;

// Robot physical parameters
#define WHEEL_DISTANCE 0.16f
#define WHEEL_RADIUS  0.035f
#define MAX_SPEED 750.0f
#define DEAD (1000.f-MAX_SPEED)

// PID Constants
#define ANGLE_PID_KP    25.0f
#define ANGLE_PID_KI    0.00f
#define ANGLE_PID_KD    0.0f

#define SPEED_PID_KP    0.0f
#define SPEED_PID_KI    0.0f
#define SPEED_PID_KD    0.0f

#define TURN_PID_KP     0.0f
#define TURN_PID_KI     0.00f
#define TURN_PID_KD     0.0f

// Wheel PID Constants 
#define WHEEL_PID_KP    0.5f
#define WHEEL_PID_KI    0.01f
#define WHEEL_PID_KD    0.001f
#define WHEEL_TIME_CONSTANT 0.1f
#define WHEEL_I_MAX ((MAX_SPEED * WHEEL_TIME_CONSTANT) / WHEEL_PID_KI)

// Time constants
#define ANGLE_TIME_CONSTANT 0.1f
#define SPEED_TIME_CONSTANT 0.1f
#define TURN_TIME_CONSTANT  0.1f

// Integral limits
#define ANGLE_I_MAX 1//((MAX_SPEED * ANGLE_TIME_CONSTANT) / ANGLE_PID_KI)
#define SPEED_I_MAX ((MAX_SPEED * SPEED_TIME_CONSTANT) / SPEED_PID_KI)
#define TURN_I_MAX  ((MAX_SPEED * TURN_TIME_CONSTANT) / TURN_PID_KI)

// Function declaration
WheelSpeeds balance(SensorData imu, WheelSpeeds current_speed, 
        float target_speed, float target_angular_velocity,
        PID_t *angle_pid, PID_t *speed_pid, PID_t *turn_pid, PID_t *wheel_pid_L, PID_t *wheel_pid_R);

// Global PID controllers
extern PID_t angle_pid;
extern PID_t speed_pid;
extern PID_t turn_pid;
extern PID_t wheel_pid_L;
extern PID_t wheel_pid_R;

#endif // __PID_H
