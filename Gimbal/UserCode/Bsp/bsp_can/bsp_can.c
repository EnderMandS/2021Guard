/*
 * @Descripttion: 对CAN通信初始化以及CAN接收发送的回调函数
 * @version: V1.0
 * @Author: Xpj
 * @LastEditors: Xpj
 * @Date: 2020-10-18 13:22:54
 */

#include "bsp_can.h"
#include "can.h"
#include "main.h"

/**
 * @brief: CAN滤波器初始化
 * @param {void} 
 * @retval: none
 * @attention: 
 */
void CAN_Filter_Init(void)
{
  CAN_FilterTypeDef CAN_Filter_STM;
	CAN_Filter_STM.FilterActivation=ENABLE;
	CAN_Filter_STM.FilterMode=CAN_FILTERMODE_IDMASK;
	CAN_Filter_STM.FilterScale = CAN_FILTERSCALE_32BIT;
  CAN_Filter_STM.FilterIdHigh = 0x0000;
  CAN_Filter_STM.FilterIdLow = 0x0000;
  CAN_Filter_STM.FilterMaskIdHigh = 0x0000;
  CAN_Filter_STM.FilterMaskIdLow = 0x0000;
  CAN_Filter_STM.FilterBank = 0;
  CAN_Filter_STM.FilterFIFOAssignment = CAN_RX_FIFO0;
  HAL_CAN_ConfigFilter(&hcan1, &CAN_Filter_STM);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


//  CAN_Filter_STM.SlaveStartFilterBank = 14;
//  CAN_Filter_STM.FilterBank = 14;
//  HAL_CAN_ConfigFilter(&hcan2, &CAN_Filter_STM);
//  HAL_CAN_Start(&hcan2);
//  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}
