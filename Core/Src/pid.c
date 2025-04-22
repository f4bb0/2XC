#include "pid.h"
#include <math.h>

// Initialize PID parameters
PID_t angle_pid = {
    .Kp = ANGLE_PID_KP, 
    .Ki = ANGLE_PID_KI, 
    .Kd = ANGLE_PID_KD,
    .error_sum_max = ANGLE_I_MAX
};

PID_t speed_pid = {
    .Kp = SPEED_PID_KP, 
    .Ki = SPEED_PID_KI, 
    .Kd = SPEED_PID_KD,
    .error_sum_max = SPEED_I_MAX
};

PID_t turn_pid = {
    .Kp = TURN_PID_KP, 
    .Ki = TURN_PID_KI, 
    .Kd = TURN_PID_KD,
    .error_sum_max = TURN_I_MAX
};

//轮速PID控制
PID_t wheel_pid_L = {
    .Kp = WHEEL_PID_KP,
    .Ki = WHEEL_PID_KI,
    .Kd = WHEEL_PID_KD,
    .error_sum_max = WHEEL_I_MAX
};
PID_t wheel_pid_R = {
    .Kp = WHEEL_PID_KP,
    .Ki = WHEEL_PID_KI,
    .Kd = WHEEL_PID_KD,
    .error_sum_max = WHEEL_I_MAX
};

WheelSpeeds balance(SensorData imu, WheelSpeeds current_speed, 
                   float target_speed, float target_angular_velocity,
                   PID_t *angle_pid, PID_t *speed_pid, PID_t *turn_pid, PID_t *wheel_pid_L, PID_t *wheel_pid_R) {
    // 1. 直立环控制（角度环）
    float angle_error = -(imu.angle.pitch - (-1.8f));  // 调整目标角度为 1.0 度
    angle_pid->error = angle_error;
    float angle_output = angle_pid->Kp * angle_pid->error + 
                         angle_pid->Ki * angle_pid->error_sum +
                         angle_pid->Kd * (angle_pid->error - angle_pid->error_last);
    angle_pid->error_last = angle_pid->error;
    angle_pid->error_sum += angle_pid->error;
    angle_pid->error_sum = fminf(fmaxf(angle_pid->error_sum, -angle_pid->error_sum_max), angle_pid->error_sum_max);
    
    // 2. 速度环控制
    float current_speed_avg = (current_speed.wheel_L + current_speed.wheel_R) / 2.0f;
    speed_pid->error = target_speed - current_speed_avg;
    float speed_output = speed_pid->Kp * speed_pid->error +
                        speed_pid->Ki * speed_pid->error_sum +
                        speed_pid->Kd * (speed_pid->error - speed_pid->error_last);
    speed_pid->error_last = speed_pid->error;
    speed_pid->error_sum += speed_pid->error;
    speed_pid->error_sum = fminf(fmaxf(speed_pid->error_sum, -speed_pid->error_sum_max), speed_pid->error_sum_max);
    
    // 3. 转向环控制
    float current_angular_velocity = (current_speed.wheel_R - current_speed.wheel_L)
                                   * WHEEL_RADIUS / WHEEL_DISTANCE;

    turn_pid->error = target_angular_velocity - current_angular_velocity;
    float turn_output = turn_pid->Kp * turn_pid->error +
                       turn_pid->Ki * turn_pid->error_sum +
                       turn_pid->Kd * (turn_pid->error - turn_pid->error_last);
    turn_pid->error_last = turn_pid->error;
    turn_pid->error_sum += turn_pid->error;
    turn_pid->error_sum = fminf(fmaxf(turn_pid->error_sum, -turn_pid->error_sum_max), turn_pid->error_sum_max);

    // 将转向输出转换为轮子速度差
    float speed_diff = turn_output * WHEEL_DISTANCE;

    // 4. 综合输出计算
    float tilt_compensation = 0.1f * speed_pid->error;
    float balance_output = angle_output + speed_output + tilt_compensation;

    // 计算目标轮速
    WheelSpeeds target = {
        .wheel_L = balance_output - speed_diff * 0.5f,
        .wheel_R = balance_output + speed_diff * 0.5f
    };
//
//    // 左轮PID
//    wheel_pid_L->error = target.wheel_L - current_speed.wheel_L;
//    float wheel_L_output = wheel_pid_L->Kp * wheel_pid_L->error +
//                          wheel_pid_L->Ki * wheel_pid_L->error_sum +
//                          wheel_pid_L->Kd * (wheel_pid_L->error - wheel_pid_L->error_last);
//    wheel_pid_L->error_last = wheel_pid_L->error;
//    wheel_pid_L->error_sum += wheel_pid_L->error;
//    wheel_pid_L->error_sum = fminf(fmaxf(wheel_pid_L->error_sum, -wheel_pid_L->error_sum_max), wheel_pid_L->error_sum_max);
//
//    // 右轮PID
//    wheel_pid_R->error = target.wheel_R - current_speed.wheel_R;
//    float wheel_R_output = wheel_pid_R->Kp * wheel_pid_R->error +
//                          wheel_pid_R->Ki * wheel_pid_R->error_sum +
//                          wheel_pid_R->Kd * (wheel_pid_R->error - wheel_pid_R->error_last);
//    wheel_pid_R->error_last = wheel_pid_R->error;
//    wheel_pid_R->error_sum += wheel_pid_R->error;
//    wheel_pid_R->error_sum = fminf(fmaxf(wheel_pid_R->error_sum, -wheel_pid_R->error_sum_max), wheel_pid_R->error_sum_max);
//
//    // 最终输出
//    WheelSpeeds output = {
//        .wheel_L = wheel_L_output,
//        .wheel_R = wheel_R_output
//    };
//
//    // 限幅保护 - 确保正负速度范围对称 (-MAX_SPEED to +MAX_SPEED)
//    output.wheel_L = fminf(fmaxf(output.wheel_L, -MAX_SPEED), MAX_SPEED);
//    output.wheel_R = fminf(fmaxf(output.wheel_R, -MAX_SPEED), MAX_SPEED);

    // 返回输出结构体
    return target;
}
