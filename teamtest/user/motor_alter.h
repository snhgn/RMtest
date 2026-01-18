#ifndef DRV_MOTORY
#define DRV_MOTORY

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
	
	
#define 	MOTOR_GROUP_NUM 			

typedef struct {
		// 电机的反馈值
    float speed;
    float current;
    uint8_t error_code;			
	} Motor_EncoderTypeDef;			//电机编码器数据
	
	
typedef struct _motor_type {
    Motor_EncoderTypeDef encoder;

    float output;
    uint32_t id;
    uint8_t is_online;							// 电机是否在线的状态标识，1 表示在线，0 表示离线
    uint32_t last_update_time;
    uint8_t init;										// 标识电机是否已初始化，1 表示已初始化，0 表示未初始化
    void (*callback)(struct _motor_type*, uint8_t rxbuff[], uint32_t len);
	} Motor_MotorTypeDef;							//控制电机的相关数据
	
typedef struct {
    uint8_t motor_num;
    Motor_MotorTypeDef* motor_handle[4];
    CAN_HandleTypeDef* can_handle;
    CAN_TxHeaderTypeDef can_header;
	} Motor_MotorGroupTypeDef;					//电机的分组，包含了ID等CAN通信所需要的数据

	/*函数指针*/
typedef void (*Motor_EncoderCallbackFuncTypeDef)(Motor_MotorTypeDef*, uint8_t[], uint32_t);	

// 外部声明Motor数组，供其他文件访问
extern Motor_MotorTypeDef Motor[4];

void Motor_InitAllMotors();
	
void Motor_InitMotorGroup(Motor_MotorGroupTypeDef* pgroup,  uint8_t motor_num, CAN_HandleTypeDef* phcan, uint16_t stdid);	
void Motor_InitMotor(Motor_MotorTypeDef* pmotor,  uint16_t id, Motor_EncoderCallbackFuncTypeDef callback);
	
void Motor_EncoderDecodeCallback(CAN_HandleTypeDef* phcan, uint32_t stdid, uint8_t rxdata[], uint32_t len);

void rm3508_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len);	

	
#ifdef __cplusplus
}
#endif

#endif
