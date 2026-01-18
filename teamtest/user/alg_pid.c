/*
 *  Project      : CAN_demo
 * 
 *  FilePath     : alg_pid.c
 *  Description  : PID控制器算法实现
 *  LastEditors  : Lee
 *  Date         : 2024年11月15日
 */

#include "alg_pid.h"
#include <math.h>

/**
 * @brief      初始化PID控制器
 * @param      pid: PID控制器结构体指针
 * @param      kp: 比例系数
 * @param      ki: 积分系数
 * @param      kd: 微分系数
 * @param      integral_limit: 积分限幅
 * @param      output_limit: 输出限幅
 * @retval     None
 */
void PID_Init(PID_TypeDef* pid, float kp, float ki, float kd, float integral_limit, float output_limit) {
    if (pid == NULL) return;
    
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
    
    pid->target = 0.0f;
    pid->current = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->output = 0.0f;
    pid->last_time = HAL_GetTick();
}

/**
 * @brief      计算PID输出
 * @param      pid: PID控制器结构体指针
 * @param      target: 目标值
 * @param      current: 当前值
 * @retval     PID输出值
 */
float PID_Calculate(PID_TypeDef* pid, float target, float current) {
    if (pid == NULL) return 0.0f;
    
    uint32_t current_time = HAL_GetTick();
    float dt = (float)(current_time - pid->last_time) / 1000.0f; // 转换为秒
    
    // 防止dt为0或过大（避免系统异常）
    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.001f; // 默认1ms
    }
    
    pid->target = target;
    pid->current = current;
    pid->error = target - current;
    
    // 比例项
    float p_out = pid->kp * pid->error;
    
    // 积分项（带积分限幅）
    pid->integral += pid->error * dt;
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }
    float i_out = pid->ki * pid->integral;
    
    // 微分项
    pid->derivative = (pid->error - pid->last_error) / dt;
    float d_out = pid->kd * pid->derivative;
    
    // PID输出
    pid->output = p_out + i_out + d_out;
    
    // 输出限幅
    if (pid->output > pid->output_limit) {
        pid->output = pid->output_limit;
    } else if (pid->output < -pid->output_limit) {
        pid->output = -pid->output_limit;
    }
    
    // 更新上次的值
    pid->last_error = pid->error;
    pid->last_time = current_time;
    
    return pid->output;
}

/**
 * @brief      重置PID控制器（清零积分项和误差）
 * @param      pid: PID控制器结构体指针
 * @retval     None
 */
void PID_Reset(PID_TypeDef* pid) {
    if (pid == NULL) return;
    
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->output = 0.0f;
    pid->last_time = HAL_GetTick();
}

/**
 * @brief      设置PID参数
 * @param      pid: PID控制器结构体指针
 * @param      kp: 比例系数
 * @param      ki: 积分系数
 * @param      kd: 微分系数
 * @retval     None
 */
void PID_SetParams(PID_TypeDef* pid, float kp, float ki, float kd) {
    if (pid == NULL) return;
    
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

