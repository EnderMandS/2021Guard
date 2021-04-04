#ifndef __BSP_USART_H__
#define __BSP_USART_H__

#include "main.h"

HAL_StatusTypeDef Bsp_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
void Dma_UsartIdleHanlder(UART_HandleTypeDef *huart,uint16_t Size);

#endif
