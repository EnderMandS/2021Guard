#include "bsp_usart.h"
#include "usart.h"
#include "usartinfo.h"
#include "string.h"

__weak void UART_IdleRxCallback(UART_HandleTypeDef *huart){}

/**
  * @brief  通过非阻塞方式接收数据，数据长度有最大限度，但是在最大限度之内可以接受任意长度的数据 
  * @param  huart: 指向UART_HandleTypeDef结构体的指针，该指针包含了UART的配置信息
  * @param  Size: 可接收数据的最大长度
  * @retval None
  */
void Dma_UsartIdleHanlder(UART_HandleTypeDef *huart,uint16_t Size)
{
	uint32_t DMA_FLAGS;//根据串口的不同来选择清除不同的DMA标志位
//  uint32_t tmp;
	
	if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
  {
				//关闭dma
	  __HAL_DMA_DISABLE(huart->hdmarx);
		
		//及时更新
		UART_IdleRxCallback(huart);
	if(huart == &huart3)
	{
		memset(&UART_Buffer,0,36);
	}		
	else if(huart == &huart1)
	{
		memset(&View_Buf,0,VIEW_BUF_LEN); //清除
	}
	else if(huart == &huart6)
	{
		memset(&Groy_Data_Buf,0,GROY_DATA_BUF_LEN); //串口陀螺仪，清除
	}
		DMA_FLAGS = __HAL_DMA_GET_TC_FLAG_INDEX(huart->hdmarx);	
		__HAL_DMA_CLEAR_FLAG(huart->hdmarx,DMA_FLAGS);
	 
		__HAL_DMA_SET_COUNTER(huart->hdmarx,Size);
		
		__HAL_DMA_ENABLE(huart->hdmarx);

		
		/*清除IDLE标志位*/
    __HAL_UART_CLEAR_IDLEFLAG(huart);
		
	}
}






/**
  * @brief  为串口开启没有中断的DMA传输，为了减少中断次数为其他中断空出资源。
  *         代替HAL库的函数(此处在main函数中调用)
  * @param  hdma: 指向DMA_HandleTypeDef结构体的指针，这个结构体包含了DMA流的配置信息.  
  * @retval HAL status
  */
HAL_StatusTypeDef Bsp_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
  uint32_t *tmp;
  uint32_t tmp1 = 0;
  
  tmp1 = huart->gState;
  if((tmp1 == HAL_UART_STATE_READY) || (tmp1 == HAL_UART_STATE_BUSY_TX))
  {
    if((pData == NULL ) || (Size == 0)) 
    {
      return HAL_ERROR;
    }
    
    /* Process Locked */
    __HAL_LOCK(huart);
    
    huart->pRxBuffPtr = pData;
    huart->RxXferSize = Size;
    
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    /* Check if a transmit process is ongoing or not */
    if(huart->gState == HAL_UART_STATE_BUSY_TX)
    {
      huart->gState = HAL_UART_STATE_BUSY_TX_RX;
    }
    else
    {
      huart->gState = HAL_UART_STATE_BUSY_RX;
    }
    
    /* Enable the DMA Stream */
    tmp = (uint32_t*)&pData;
    HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, *(uint32_t*)tmp, Size);
    
    /* Enable the DMA transfer for the receiver request by setting the DMAR bit 
    in the UART CR3 register */
    huart->Instance->CR3 |= USART_CR3_DMAR;
    
    /* Process Unlocked */
    __HAL_UNLOCK(huart);
    
    return HAL_OK;
  }
  else
  {
    return HAL_BUSY; 
  }
}


/**
  * @brief  通过非阻塞方式接收数据，数据长度有最大限度，但是在最大限度之内可以接受任意长度的数据 DMADMA
  * @param  huart: 指向UART_HandleTypeDef结构体的指针，该指针包含了UART的配置信息
  * @param  pData: 指向接受数据缓冲区的指针
  * @param  Size: 可接收数据的最大长度
  * @retval HAL status
  */
HAL_StatusTypeDef Bsp_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
	Bsp_UART_Receive_DMA(huart,pData,Size);//利用DMA接受数据
	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);//开启串口空闲中断
	return HAL_OK;
}

