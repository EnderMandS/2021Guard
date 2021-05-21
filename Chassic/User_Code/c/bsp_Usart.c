#include "bsp_usart.h"
#include "judge.h"
#include "usart.h"

uint8_t rx_judge_buf[200];

/**
  * @brief  Ϊ���ڿ���û���жϵ�DMA���䣬Ϊ�˼����жϴ���Ϊ�����жϿճ���Դ��
  *         ����HAL��ĺ���(�˴���main�����е���)
  * @param  hdma: ָ��DMA_HandleTypeDef�ṹ���ָ�룬����ṹ�������DMA����������Ϣ.  
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
  * @brief  ͨ����������ʽ�������ݣ����ݳ���������޶ȣ�����������޶�֮�ڿ��Խ������ⳤ�ȵ����� DMADMA
  * @param  huart: ָ��UART_HandleTypeDef�ṹ���ָ�룬��ָ�������UART��������Ϣ
  * @param  pData: ָ��������ݻ�������ָ��
  * @param  Size: �ɽ������ݵ���󳤶�
  * @retval HAL status
  */
HAL_StatusTypeDef Bsp_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
	Bsp_UART_Receive_DMA(huart,pData,Size);//����DMA��������
	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);//�������ڿ����ж�
	return HAL_OK;
}

/**
  * @brief  ���ڿ����ж�DMA���ջص�����
  * @param  ����ͨ����ַ UART_HandleTypeDef *
  * @retval None
  */
void UART_IdleRxCallback(UART_HandleTypeDef *huart)
{
	//����ϵͳ����
	if (huart == &huart6)
	{
		memcpy(&rx_judge_buf, Judgement_Buf, 200); //���ݳ���
		Judge_Read_Data(Judgement_Buf);
	}
}

/**
  * @brief  ͨ����������ʽ�������ݣ����ݳ���������޶ȣ�����������޶�֮�ڿ��Խ������ⳤ�ȵ����� 
  * @param  huart: ָ��UART_HandleTypeDef�ṹ���ָ�룬��ָ�������UART��������Ϣ
  * @param  Size: �ɽ������ݵ���󳤶�
  * @retval None
  */
void Dma_UsartIdleHanlder(UART_HandleTypeDef *huart,uint16_t Size)
{
	uint32_t DMA_FLAGS;//���ݴ��ڵĲ�ͬ��ѡ�������ͬ��DMA��־λ
//  uint32_t tmp;
	
	if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
  {
		//�ر�dma
	  __HAL_DMA_DISABLE(huart->hdmarx);
		//��ʱ����
		UART_IdleRxCallback(huart);
		if(huart == &huart6)
		{
			memset(&Judgement_Buf,0,JUDGEMENT_BUF_LEN); //���
		}
		DMA_FLAGS = __HAL_DMA_GET_TC_FLAG_INDEX(huart->hdmarx);	
		__HAL_DMA_CLEAR_FLAG(huart->hdmarx,DMA_FLAGS);
		__HAL_DMA_SET_COUNTER(huart->hdmarx,Size);
		__HAL_DMA_ENABLE(huart->hdmarx);
		/*���IDLE��־λ*/
    __HAL_UART_CLEAR_IDLEFLAG(huart);
	}
}
