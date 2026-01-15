#include "pid.h"

void PID_Init(PID_TypeDef *pid, float p, float i, float d, float max_out, float max_iout)
{
    pid->Kp = p;
    pid->Ki = i;
    pid->Kd = d;
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->err[0] = pid->err[1] = pid->err[2] = 0.0f;
    pid->out = pid->Iout = 0.0f;
}

float PID_Calc(PID_TypeDef *pid, float target, float feedback)
{
    pid->set = target;
    pid->fdb = feedback;
    pid->err[0] = target - feedback;

    pid->Pout = pid->Kp * pid->err[0];
    pid->Iout += pid->Ki * pid->err[0];
    pid->Dout = pid->Kd * (pid->err[0] - pid->err[1]);

    // 积分限幅
    if (pid->Iout > pid->max_iout) pid->Iout = pid->max_iout;
    else if (pid->Iout < -pid->max_iout) pid->Iout = -pid->max_iout;

    pid->out = pid->Pout + pid->Iout + pid->Dout;

    // 总输出限幅
    if (pid->out > pid->max_out) pid->out = pid->max_out;
    else if (pid->out < -pid->max_out) pid->out = -pid->max_out;

    pid->err[2] = pid->err[1];
    pid->err[1] = pid->err[0];

    return pid->out;
}