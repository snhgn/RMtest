/*
 *  Project      : CAN_demo
 * 
 *  FilePath     : alg_pid.h
 *  Description  : PID控制器算法头文件
 *  LastEditors  : Lee
 *  Date         : 2024年11月15日
 */

#ifndef ALG_PID_H
#define ALG_PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

// PID控制器结构体
typedef struct {
    float kp;              // 比例系数
    float ki;              // 积分系数
    float kd;              // 微分系数
    
    float target;          // 目标值
    float current;         // 当前值
    float error;           // 当前误差
    float last_error;      // 上次误差
    float integral;        // 积分项
    float derivative;      // 微分项
    float output;          // 输出值
    
    float integral_limit;  // 积分限幅
    float output_limit;    // 输出限幅
    
    uint32_t last_time;    // 上次更新时间
} PID_TypeDef;

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
void PID_Init(PID_TypeDef* pid, float kp, float ki, float kd, float integral_limit, float output_limit);

/**
 * @brief      计算PID输出
 * @param      pid: PID控制器结构体指针
 * @param      target: 目标值
 * @param      current: 当前值
 * @retval     PID输出值
 */
float PID_Calculate(PID_TypeDef* pid, float target, float current);

/**
 * @brief      重置PID控制器（清零积分项和误差）
 * @param      pid: PID控制器结构体指针
 * @retval     None
 */
void PID_Reset(PID_TypeDef* pid);

/**
 * @brief      设置PID参数
 * @param      pid: PID控制器结构体指针
 * @param      kp: 比例系数
 * @param      ki: 积分系数
 * @param      kd: 微分系数
 * @retval     None
 */
void PID_SetParams(PID_TypeDef* pid, float kp, float ki, float kd);

#ifdef __cplusplus
}
#endif

#endif /* ALG_PID_H */

