#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "main.h"

// 简单的死区限制，防止摇杆轻微漂移导致车乱动
#define RC_DEADZONE 10 

void Chassis_Init(void);
void Chassis_Loop_Handler(void);

#endif