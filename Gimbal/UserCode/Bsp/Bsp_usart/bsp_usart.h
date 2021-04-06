#ifndef _BSP_USART_H
#define _BSP_USART_H
#include "stdint.h"
#include "main.h"

void Dma_UsartIdleHanlder(UART_HandleTypeDef *huart,uint16_t Size);
HAL_StatusTypeDef HAL_UART_Receive_IT_IDLE(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef Bsp_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef Bsp_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
void HAL_UART_IDLE_IRQHandler(UART_HandleTypeDef *huart);

#endif
