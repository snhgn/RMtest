#include "bsp_can.h"
#include "motor_alter.h"
#include "alg_pid.h"
#include "remote.h"
#include "work.h"
#include "main.h"
#include "math.h"
#include <stdlib.h>

extern CAN_HandleTypeDef hcan1;
extern Motor_MotorTypeDef Motor[4];
extern Rc_Data rc;

static PID_TypeDef motor_pids[4];    
static uint8_t is_work_initialized = 0; 

#define DEAD_ZONE 20       
#define MAX_RPM 5000.0f     
#define RC_MAX 660.0f 

float clip(float value, float min, float max) {
    if (value > max) return max;
    if (value < min) return min;
    return value;
}


void Work(){
       // [0. 初始化阶段] 
    // 只在第一次运行时执行，用于设置 PID 参数
    if (is_work_initialized == 0) {
        for (int i = 0; i < 4; i++) {
            PID_Init(&motor_pids[i], 0.0f, 0.0f, 0.0f, 3000.0f, 16384.0f);
        }
        is_work_initialized = 1;
    } 
    // [1. 处理遥控器数据]
    // 假设: ch1=前进后退, ch0=左右平移, ch2=旋转 (具体看您的遥控器模式)
    
    float vx = (abs(rc.ch[1]) < DEAD_ZONE) ? 0 : (float)rc.ch[1]; // 前进速度 (前后推杆)
    float vy = (abs(rc.ch[0]) < DEAD_ZONE) ? 0 : (float)rc.ch[0]; // 平移速度 (左右推杆)
    float vw = (abs(rc.ch[2]) < DEAD_ZONE) ? 0 : (float)rc.ch[2]; // 旋转速度 (旋转拨杆)

    // 归一化并映射到目标转速
    vx = (vx / RC_MAX) * MAX_RPM;
    vy = (vy / RC_MAX) * MAX_RPM;
    vw = (vw / RC_MAX) * MAX_RPM;

    // [2. 麦克纳姆轮运动学解算] 
    float target_speeds[4];

    // 假设顺序: 0(左前), 1(右前), 2(右后), 3(左后)
    // 或者: 0(右前), 1(左前) ... 请根据 Motor_InitAllMotor 中的 ID 顺序确认
    // 这里的 ID 顺序依据 Motor_InitAllMotors: 0->201, 1->202, 2->203, 3->204
    
    // 典型 O型 或 X型 分配公式：
    target_speeds[0] =  vx + vy + vw; // Motor 0 (ID:201)
    target_speeds[1] = -vx + vy + vw; // Motor 1 (ID:202) (右侧电机通常安装方向相反，所以Vx取反)
    target_speeds[2] = -vx - vy + vw; // Motor 2 (ID:203) (右侧)
    target_speeds[3] =  vx - vy + vw; // Motor 3 (ID:204)

    // [3. PID计算]
    int16_t current_outputs[4];
    
    for (int i = 0; i < 4; i++) {
        // 计算 PID: 目标转速 vs 实际反馈转速
        float output = PID_Calculate(&motor_pids[i], target_speeds[i], Motor[i].encoder.speed);
        current_outputs[i] = (int16_t)output;
    }

    // [4. CAN 发送]
    // 配置发送头，控制 ID 1-4 的电机，标识符为 0x200
    CAN_TxHeaderTypeDef tx_header;
    Can_InitTxHeader(&tx_header, 0x200, 0, 8);

    uint8_t tx_data[8];
    tx_data[0] = (uint8_t)(current_outputs[0] >> 8);
    tx_data[1] = (uint8_t)(current_outputs[0]);
    tx_data[2] = (uint8_t)(current_outputs[1] >> 8);
    tx_data[3] = (uint8_t)(current_outputs[1]);
    tx_data[4] = (uint8_t)(current_outputs[2] >> 8);
    tx_data[5] = (uint8_t)(current_outputs[2]);
    tx_data[6] = (uint8_t)(current_outputs[3] >> 8);
    tx_data[7] = (uint8_t)(current_outputs[3]);

    Can_SendMessage(&hcan1, &tx_header, tx_data);

}
