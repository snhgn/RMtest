#include "remote.h"
#include "usart.h"
#include "string.h"

//extern UART_HandleTypeDef huart3;
//extern DMA_HandleTypeDef hdma_usart3_rx;

uint8_t rxBuff[54];
uint8_t* pData = rxBuff;

Rc_Data rc;

void rc_init()
{
    //memset(&rc,0,sizeof(rc));
   
    for(int i = 0;i<4;++i)
    {
        rc.ch[i] = 0;
    }
    rc.s[0] = 0;
    rc.s[1] = 0;
    //开启串口空闲中断
    __HAL_UART_CLEAR_IDLEFLAG(&huart3);
    __HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);
    //启动串口
    if(huart3.RxState == HAL_UART_STATE_READY)
    {
        huart3.pRxBuffPtr = pData;
        huart3.RxXferSize = 54;
        huart3.ErrorCode = HAL_UART_ERROR_NONE;
        //配置DMA
        HAL_DMA_Start(huart3.hdmarx,(uint32_t)&huart3.Instance -> DR,(uint32_t)rxBuff,54);
        //使能串口DMA接受
        SET_BIT(huart3.Instance -> CR3,USART_CR3_DMAR);
    }
}

void rc_processdata(uint8_t* rxBuff)
{
    rc.ch[0] = offset(((int16_t)rxBuff[0] | ((int16_t)rxBuff[1] << 8)) & 0x07FF);
    rc.ch[1] = offset((((int16_t)rxBuff[1] >> 3) | ((int16_t)rxBuff[2] << 5)) & 0x07FF);
    rc.ch[2] = offset((((int16_t)rxBuff[2] >> 6) | ((int16_t)rxBuff[3] << 2) | ((int16_t)rxBuff[4] << 10)) & 0x07FF);
    rc.ch[3] = offset((((int16_t)rxBuff[4] >> 1) | ((int16_t)rxBuff[5]<<7)) & 0x07FF); 

    rc.s[0] = (rxBuff[5] >> 6) & 0x03;
    rc.s[1] =  (rxBuff[5] >> 4) & 0x03;
}

int16_t offset(int16_t rc)
{
    int16_t temp;
   temp = rc-1024;
    return temp;
}
