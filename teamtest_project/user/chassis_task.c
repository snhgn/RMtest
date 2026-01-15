#include "chassis_task.h"
#include "remote.h"   // 引用遥控器变量 rc
#include "drv_can.h"  // 引用CAN发送缓冲区和电机反馈数据
#include "pid.h"      // 引用PID算法

// 引用外部定义的全局变量
extern Rc_Data rc;
extern uint8_t CAN1_0x200_Tx_Data[8];
extern Motor_Measure_t motor_chassis[4]; // 引用电机反馈数据 [0]:ID1, [1]:ID2...

// 定义4个电机的PID结构体
PID_TypeDef pid_chassis[4];

// PID参数 (需要根据实际电机型号C610/C620/M3508进行调整)
// 这里给出一组典型的M3508/C620速度环参数作为初始值
#define CHASSIS_KP 5.0f
#define CHASSIS_KI 0.0f
#define CHASSIS_KD 0.0f
#define CHASSIS_MAX_OUT 16000.0f  // C620/M3508最大电流约为16384
#define CHASSIS_MAX_IOUT 5000.0f

// 遥控器数值转目标转速的比例系数
// 遥控器最大值660, 假设最大期望转速 6000 RPM (减速后轮子转速需除以减速比)
// 660 * 9 ≈ 6000
#define RC_TO_SPEED_GAIN 9.0f 

void Chassis_Init(void)
{
    // 初始化4个电机的PID
    for(int i=0; i<4; i++) {
        PID_Init(&pid_chassis[i], CHASSIS_KP, CHASSIS_KI, CHASSIS_KD, CHASSIS_MAX_OUT, CHASSIS_MAX_IOUT);
    }
}

/**
 * @brief 将int16类型的数值填入CAN发送缓冲区
 * @param buffer 缓冲区指针
 * @param index 电机索引 (0-3)
 * @param value 电流值/电压值
 */
static void Set_Motor_Tx_Data(uint8_t *buffer, uint8_t index, int16_t value)
{
    // CAN协议规定：高8位在前，低8位在后
    buffer[index * 2]     = (value >> 8);
    buffer[index * 2 + 1] = value;
}

/**
 * @brief 底盘控制主循环
 * 建议在 main while(1) 或 定时器中断 中调用
 */
void Chassis_Loop_Handler(void)
{
    // 1. 安全保护：如果遥控器掉线(数据全0) 或 拨杆处于下档(假设下档是2)，则停止
    // rc.s[1] 的值：1(上), 3(中), 2(下) - 大疆遥控器通常逻辑
    if (rc.s[1] == 2) 
    {
        for (int i = 0; i < 4; i++) {
            Set_Motor_Tx_Data(CAN1_0x200_Tx_Data, i, 0);
            // 重置PID积分项，防止恢复时瞬间冲出
            pid_chassis[i].Iout = 0;
            pid_chassis[i].out = 0;
        }
        return;
    }

    // 2. 读取遥控器数值 & 死区处理
    // 目标转速计算
    float vx = (abs(rc.ch[0]) < RC_DEADZONE) ? 0 : rc.ch[0]; // 左右平移
    float vy = (abs(rc.ch[1]) < RC_DEADZONE) ? 0 : rc.ch[1]; // 前进后退
    float wz = (abs(rc.ch[2]) < RC_DEADZONE) ? 0 : rc.ch[2]; // 旋转

    // 3. 麦克纳姆轮运动学解算 (O型安装)
    // 根据你的电机安装实际ID顺序，可能需要调整正负号
    // 定义目标转速 (Target RPM)
    float target_speed[4];
    
    // 假设：Motor1:右前, Motor2:左前, Motor3:左后, Motor4:右后
    // 运动学分解公式
    target_speed[0] = (vy - vx - wz) * RC_TO_SPEED_GAIN; // 右前 (ID 1)
    target_speed[1] = (vy + vx + wz) * RC_TO_SPEED_GAIN; // 左前 (ID 2)
    target_speed[2] = (vy - vx + wz) * RC_TO_SPEED_GAIN; // 左后 (ID 3)
    target_speed[3] = (vy + vx - wz) * RC_TO_SPEED_GAIN; // 右后 (ID 4)

    // 4. PID 计算与输出填充
    for (int i = 0; i < 4; i++)
    {
        // 获取真实转速 (来自CAN回调更新的 motor_chassis 数组)
        float real_speed = motor_chassis[i].speed_rpm;

        // 计算PID，得到目标电流值
        float out_current = PID_Calc(&pid_chassis[i], target_speed[i], real_speed);

        // 填入CAN发送缓冲区
        Set_Motor_Tx_Data(CAN1_0x200_Tx_Data, i, (int16_t)out_current);
    }
    
    // 注意：这里只更新了数据缓冲区 CAN1_0x200_Tx_Data
    // 你需要确保在其他地方（如定时器中断）调用了实际的 CAN 发送函数，
    // 或者在这里直接调用发送函数，例如：
    // HAL_CAN_AddTxMessage(&hcan1, ...); 
    // 或者使用 drv_can.c 中可能存在的发送接口。
}