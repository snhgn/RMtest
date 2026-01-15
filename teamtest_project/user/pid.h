#ifndef __PID_H
#define __PID_H

#include "main.h"

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    
    float max_out;  // 最大输出限幅 (例如 16384对应20A)
    float max_iout; // 积分限幅
    
    float set;      // 目标值
    float fdb;      // 反馈值
    
    float out;      // 计算出的输出值
    float Pout;
    float Iout;
    float Dout;
    
    float err[3];   // [0]本次误差 [1]上次误差 [2]上上次误差
    
} PID_TypeDef;

void PID_Init(PID_TypeDef *pid, float p, float i, float d, float max_out, float max_iout);
float PID_Calc(PID_TypeDef *pid, float target, float feedback);

#endif
