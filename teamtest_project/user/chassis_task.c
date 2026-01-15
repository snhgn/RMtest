#include "chassis_task.h"
#include "remote.h"   // 引用遥控器变量 rc
#include "drv_can.h"  // 引用CAN发送缓冲区

// 引用外部定义的全局变量
extern Rc_Data rc;
extern uint8_t CAN1_0x200_Tx_Data[];

// 电机目标值
int16_t motor_speed_target[4];

void Chassis_Init(void)
{
    // 如果有底盘特定的初始化放在这里
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
        for (int i = 0; i < 4; i++) Set_Motor_Tx_Data(CAN1_0x200_Tx_Data, i, 0);
        return;
    }

    // 2. 读取遥控器数值 & 死区处理
    int16_t vx = (abs(rc.ch[0]) < RC_DEADZONE) ? 0 : rc.ch[0]; // 左右平移
    int16_t vy = (abs(rc.ch[1]) < RC_DEADZONE) ? 0 : rc.ch[1]; // 前进后退
    int16_t wz = (abs(rc.ch[2]) < RC_DEADZONE) ? 0 : rc.ch[2]; // 旋转

    // 3. 麦克纳姆轮运动学解算 (O型安装)
    // 根据你的电机安装实际ID顺序，可能需要调整正负号
    // 假设：Motor1:右前, Motor2:左前, Motor3:左后, Motor4:右后
    
    // 增加一个增益系数，把遥控器数值(max 660)放大到电机电流数值(max 10000左右)
    float gain = 10.0f; 

    int16_t wheel_1 = (vy - vx - wz) * gain; // 右前
    int16_t wheel_2 = (vy + vx + wz) * gain; // 左前
    int16_t wheel_3 = (vy - vx + wz) * gain; // 左后
    int16_t wheel_4 = (vy + vx - wz) * gain; // 右后

    // 4. 将计算结果填入CAN发送缓冲区
    // 注意：这里是开环控制，直接把遥控器量映射为电流。
    // 如果想要精准速度控制，这里应该计算出"目标转速"，然后通过PID算法算出电流。
    
    Set_Motor_Tx_Data(CAN1_0x200_Tx_Data, 0, wheel_1); // ID 1
    Set_Motor_Tx_Data(CAN1_0x200_Tx_Data, 1, wheel_2); // ID 2
    Set_Motor_Tx_Data(CAN1_0x200_Tx_Data, 2, wheel_3); // ID 3
    Set_Motor_Tx_Data(CAN1_0x200_Tx_Data, 3, wheel_4); // ID 4
    
    // 注意：数据填入缓冲区后，实际的发送是在 drv_can.c 的定时器中断里进行的
}