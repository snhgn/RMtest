/*
 *  Project      : CAN_demo
 * 
 *  FilePath     : bsp_can.h
 *  Description  : 控制CAN通信
 *  LastEditors  : 李宪
 *  Date         : 2024年8月19日20:03:58
 *  LastEditTime : 2024年11月15日22:14:18
 */


#ifndef CAN_UTIL_H
#define CAN_UTIL_H

#ifdef __cplusplus
#endif 

#include "can.h"

/*	以下全局变量只在stm32xx_it.c文件里面调用，其他地方没有使用到，移植的时候需要注意	*/
#define Const_Can_RX_BUFF_LEN  200
extern uint8_t Can_RxData[Const_Can_RX_BUFF_LEN];
extern	CAN_RxHeaderTypeDef Can_RxHeader;

void Can_ErrorHandler(uint32_t ret);
void Can_InitTxHeader(CAN_TxHeaderTypeDef *pheader, uint32_t stdid, uint32_t extid, uint32_t dlc);
void Can_InitFilterAndStart(CAN_HandleTypeDef* phcan);
void Can_SendMessage(CAN_HandleTypeDef* phcan, CAN_TxHeaderTypeDef* pheader, uint8_t txdata[]);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* phcan);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *phcan);
void Can_RxMessageCallback(CAN_HandleTypeDef* phcan, CAN_RxHeaderTypeDef* rxheader, uint8_t rxdata[]);

#endif

#ifdef __cplusplus
#endif
