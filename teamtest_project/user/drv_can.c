#include "drv_can.h"

Struct_CAN_Manage_Object CAN1_Manage_Object = {0};
uint8_t CAN1_0x200_Tx_Data[8];
Motor_Measure_t motor_chassis[4]; // 电机反馈数据

void CAN_Init(CAN_HandleTypeDef *hcan, CAN_Call_Back Callback_Function)
{
    HAL_CAN_Start(hcan);
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    // FIFO1中断不需要开启，因为Filter全部映射到了FIFO0
    
    if (hcan->Instance == CAN1)
    {
        CAN1_Manage_Object.CAN_Handler = hcan;
        CAN1_Manage_Object.Callback_Function = Callback_Function;
    }
}

void CAN_Filter_Mask_Config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID)
{
    CAN_FilterTypeDef can_filter_init_structure;
    if (Object_Para & 0x01) return;
    if ((Object_Para & 0x02) >> 1) return;

    can_filter_init_structure.FilterIdHigh = (ID & 0x7FF) << 5;
    can_filter_init_structure.FilterIdLow = 0x0000;
    can_filter_init_structure.FilterMaskIdHigh = (Mask_ID & 0x7FF) << 5;
    can_filter_init_structure.FilterMaskIdLow = 0x0000;
    can_filter_init_structure.FilterBank = (Object_Para >> 3) & 0x1F;
    can_filter_init_structure.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_init_structure.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_init_structure.FilterActivation = ENABLE;
    can_filter_init_structure.SlaveStartFilterBank = 14;
    can_filter_init_structure.FilterFIFOAssignment = (Object_Para >> 2) & 0x01;

    HAL_CAN_ConfigFilter(hcan, &can_filter_init_structure);
}

uint8_t CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t used_mailbox;
    assert_param(hcan != NULL);
    tx_header.StdId = ID;
    tx_header.ExtId = 0;
    tx_header.IDE = 0;
    tx_header.RTR = 0;
    tx_header.DLC = Length;
    return (HAL_CAN_AddTxMessage(hcan, &tx_header, Data, &used_mailbox));
}

void TIM_CAN_PeriodElapsedCallback()
{
    // 你的 main.c 开启了 TIM6，并在 stm32f4xx_it.c 中调用了此函数，这里是正确的发送逻辑
    CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);
}

/**
 * @brief HAL库CAN接收FIFO0中断
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    // 获取数据
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    // 1. 处理电机反馈 (DJI电机 ID通常为 0x201-0x204)
    if (rx_header.StdId >= 0x201 && rx_header.StdId <= 0x204)
    {
        uint8_t index = rx_header.StdId - 0x201;
        // 解析转速 (高8位在前)
        motor_chassis[index].speed_rpm = (int16_t)(rx_data[2] << 8 | rx_data[3]);
        motor_chassis[index].real_current = (int16_t)(rx_data[4] << 8 | rx_data[5]);
        // 如果需要位置环，还需要处理 total_angle
    }

    // 2. 如果有其他回调函数 (保留原逻辑)
    if (hcan->Instance == CAN1 && CAN1_Manage_Object.Callback_Function != NULL)
    {
        // 这里的结构体转换和回调是为了兼容你原本的架构
        // 但注意：Struct_CAN_Rx_Buffer 需要在头文件中定义完整
        static Struct_CAN_Rx_Buffer can_rx_buffer;
        can_rx_buffer.Header = rx_header;
        for(int i=0; i<8; i++) can_rx_buffer.Data[i] = rx_data[i];
        CAN1_Manage_Object.Callback_Function(&can_rx_buffer);
    }
}
