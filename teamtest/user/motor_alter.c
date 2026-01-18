/*
 *  Project      :  
 *  
 *  FilePath     : drv_motor.c
 *  Description  : 这个文件是控制电机相关的驱动
 *  LastEditors  : 李宪
 *  Date         : 2024年8月19日19:18:51
 *  LastEditTime : 
 */
 
#include <stdio.h>
#include <stdlib.h>
#include "motor_alter.h"
#include "bsp_can.h"


const uint32_t Const_Motor_MOTOR_OFFLINE_TIME = 200;		//电机离线时间的常量值，单位毫秒
const uint32_t Const_Motor_MOTOR_TX_EXTID = 0x01;				//电机发送数据的扩展ID，0x01。
const uint32_t Const_Motor_MOTOR_TX_DLC = 8;						//数据长度，8字节
const uint32_t Const_Motor_MOTOR_RX_DLC = 8;



/*	创建储存电机数据的结构体	*/
Motor_MotorTypeDef Motor[4];



/**
	* @brief      初始化各个组中的电机
  * @param      NULL
  * @retval     使用本函数需要包含一下“can.h”
  */
void Motor_InitAllMotors() {

	  Motor_InitMotor(&Motor[0],0x201,rm3508_encoder_callback);
	  Motor_InitMotor(&Motor[1],0x202,rm3508_encoder_callback);	 
	  Motor_InitMotor(&Motor[2],0x203,rm3508_encoder_callback);
	  Motor_InitMotor(&Motor[3],0x204,rm3508_encoder_callback);		
}


/**
  * @brief      电机初始化
	* @param      pmotor: 指向电机结构体的指针
	* @param      callback: 电机所对应的编码器解码函数
  * @retval     NULL
  */
void Motor_InitMotor(Motor_MotorTypeDef* pmotor,  uint16_t id, 
                     Motor_EncoderCallbackFuncTypeDef callback) {
    if (pmotor == NULL) return;
    pmotor->last_update_time 	= 0;
    pmotor->id 								= id;			
    pmotor->init	 						= 0;			
    pmotor->is_online 				= 0;
    pmotor->output 						= 0;
    pmotor->callback 					= callback;
}


/********** 编码器的回调函数 **********/
/**
  * @brief      用于处理电机编码器解码的回调函数
  * @param      phcan: 	指向 CAN 句柄的指针，表示哪个 CAN 接口被使用
  * @param      stdid: 	CAN 标识符，用于区分不同的 CAN 消息
  * @param      rxdata: 接收数据的缓冲区，包含了从 CAN 总线接收到的数据
  * @param      len:  	接收到的数据长度
  * @retval     NULL
  */
void Motor_EncoderDecodeCallback(CAN_HandleTypeDef* phcan, uint32_t stdid, uint8_t rxdata[], uint32_t len) {
    if (phcan == NULL) {
        return; // 如果 phcan 为 NULL，直接返回，避免空指针问题
    }
		for (int j = 0; j < 4; j++) {
				Motor_MotorTypeDef* motor = &Motor[j]; // 获取当前电机的句柄
				// 检查 CAN 标识符是否与当前电机匹配
				if (stdid == motor->id) {
						// 调用电机的回调函数进行数据处理，注意，这里的回调是个空的函数指针，具体函数要在初始化中设置
						motor->callback(motor, rxdata, len);
						return; // 找到匹配的电机后立即返回
				}
		}
}


/********** 编码器解析函数，以下函数都是在初始化中赋值到结构体就可以了 **********/


/**
  * @brief      rm3508 电机编码器回调
  * @param      pmotor: 指向Motor_MotorTypeDef句柄的指针
  * @retval     NULL
  */
void rm3508_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len) {
    if (pmotor == NULL) return;
    // 从接收到的数据中解析当前速度和电流
    pmotor->encoder.speed   = (float)((int16_t)((uint16_t)rxbuff[2] << 8 | (uint16_t)rxbuff[3])) / 19.0f;
    pmotor->encoder.current = (float)((int16_t)((uint16_t)rxbuff[4] << 8 | (uint16_t)rxbuff[5]));
		// 更新电机状态的最后更新时间戳
    pmotor->last_update_time = HAL_GetTick(); 
}
