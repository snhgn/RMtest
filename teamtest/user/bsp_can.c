/*
 *  Project      : CAN_demo
 * 
 *  FilePath     : bsp_can.c
 *  Description  : 控制CAN通信
 *  LastEditors  : 李宪
 *  Date         : 2024年8月19日20:04:21
 *  LastEditTime : 2024年11月15日22:14:14
 */


#include "bsp_can.h"
#include "motor_alter.h"

CAN_RxHeaderTypeDef Can_RxHeader;
uint8_t Can_RxData[Const_Can_RX_BUFF_LEN];


/**
* @brief        	CAN通信遇到问题的时候报错，使用串口把信息打印到电脑上，暂时没写串口，不用管
 * @param         [uint32_t] ret
 * @return        [type]
 */
void Can_ErrorHandler(uint32_t ret) {
    //Log_DebugPrintf("Error: CAN Error!\n");
    while (1) {
        return;
    }
}


/**
 * @brief         初始化CAN的发送器，包括ID，DLC等数据
 * @param         [CAN_TxHeaderTypeDef] *pheader : 指向 CAN 发送头结构体的指针
 * @param         [uint32_t] stdid : 标准标识符 (Standard Identifier)
 * @param         [uint32_t] extid : 扩展标识符 (Extended Identifier)
 * @param         [uint32_t] dlc : 数据长度码 (Data Length Code)
 * @return        [type]
 */
void Can_InitTxHeader(CAN_TxHeaderTypeDef *pheader, uint32_t stdid, uint32_t extid, uint32_t dlc) {
    pheader->StdId = stdid;
    pheader->ExtId = extid;
    pheader->RTR = CAN_RTR_DATA;
    pheader->IDE = CAN_ID_STD;
    pheader->DLC = dlc;
    pheader->TransmitGlobalTime = DISABLE;
}


/**
 * @brief        	初始化CAN通信
 * @param         [CAN_HandleTypeDef*] CAN 句柄的指针，用于标识哪个 CAN 接口将被配置和启用
 * @return        [type]
 */
void Can_InitFilterAndStart(CAN_HandleTypeDef* phcan) {
    CAN_FilterTypeDef sFilterConfig;
		
		// 根据传入的CAN句柄决定使用哪个过滤器组
    if (phcan == &hcan1)
        sFilterConfig.FilterBank = 0;				
    else
        sFilterConfig.FilterBank = 14;
    
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;			// 设置过滤器模式为标识符掩码模式（ID Mask）
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;		// 设置过滤器的位宽为32位
    sFilterConfig.FilterIdHigh = 0x0000;									// 设置过滤器ID的高16位为0x0000，接收所有ID的消息（因为掩码为0不进行过滤）
    sFilterConfig.FilterIdLow = 0x0000;										// 设置过滤器ID的低16位为0x0000
    sFilterConfig.FilterMaskIdHigh = 0x0000;							// 设置过滤器掩码ID的高16位为0x0000
    sFilterConfig.FilterMaskIdLow = 0x0000;								// 设置过滤器掩码ID的低16位为0x0000
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;		// 将接收到的CAN消息分配到FIFO0
    sFilterConfig.FilterActivation = ENABLE;							// 启用过滤器
    sFilterConfig.SlaveStartFilterBank = 14;							// 设置从过滤器的起始组号为14（通常用于多CAN配置中的从属过滤器）

    uint32_t ret = HAL_CAN_ConfigFilter(phcan, &sFilterConfig);// 配置CAN过滤器，传入CAN句柄和过滤器配置结构体
    if (ret != HAL_OK) {
        Can_ErrorHandler(ret);
    }
    
    ret = HAL_CAN_Start(phcan);	// 启动CAN模块，使其进入工作状态
    if (ret != HAL_OK) {
        Can_ErrorHandler(ret);
    }
    
    ret = HAL_CAN_ActivateNotification(phcan, CAN_IT_RX_FIFO0_MSG_PENDING);// 激活CAN接收中断，当FIFO0中有新消息时触发中断
    if (ret != HAL_OK) {
        Can_ErrorHandler(ret);
    }   
}


/**
 * @brief        	函数用于将信息发送到 CAN 总线上
 * @param         [CAN_HandleTypeDef*] phcan	CAN 句柄的指针，用于指定哪个 CAN 接口将用于发送消息。
 * @param         [CAN_TxHeaderTypeDef*] pheader	指向 CAN 发送消息头部结构体的指针，包含了发送消息的相关配置信息（如标识符、帧格式等）。
 * @param         [uint8_t] txdata		要发送的数据的数组
 * @return        [type]
 */
void Can_SendMessage(CAN_HandleTypeDef* phcan, CAN_TxHeaderTypeDef* pheader, uint8_t txdata[]) {
    uint32_t mailbox;
    /* 开始发送 */
    uint32_t ret = HAL_CAN_AddTxMessage(phcan, pheader, txdata, &mailbox);
    if (ret != HAL_OK) {
        /* 报错 */
        Can_ErrorHandler(ret);
    }
}


/**
 * @brief        	这个函数用在CAN的中断回调中，用于对接收到的数据进行处理
 * @param         [CAN_HandleTypeDef*] phcan				指向 CAN 句柄的指针，用于标识具体的 CAN 外设实例
 * @param         [CAN_RxHeaderTypeDef*] rxheader 	结构体的指针，包含接收到的 CAN 消息的头部信息（如标准标识符 StdId，数据长度码 DLC 等）
 * @param         [uint8_t] rxdata									存储接收到的 CAN 消息的数据
 * @return        [type]
 */
void Can_RxMessageCallback(CAN_HandleTypeDef* phcan, CAN_RxHeaderTypeDef* rxheader, uint8_t rxdata[]) {
    if (phcan == &hcan1) {
        Motor_EncoderDecodeCallback(phcan, rxheader -> StdId, rxdata, rxheader -> DLC);
    }
}

