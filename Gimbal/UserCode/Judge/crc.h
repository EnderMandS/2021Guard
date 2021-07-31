#ifndef __CRC_H_
#define __CRC_H_

#include "stdint.h"
#include "stdint.h"
#include "stdio.h"
#include "stdbool.h"
#include "string.h"

// CRC8
void Append_CRC8_Check_Sum( uint8_t *pchMessage, uint16_t dwLength);
uint32_t Verify_CRC8_Check_Sum( uint8_t *pchMessage, uint16_t dwLength);
uint8_t Get_CRC8_Check_Sum( uint8_t *pchMessage, uint16_t dwLength, uint8_t ucCRC8 );

// CRC16
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
#define    FALSE    0
#define    TRUE     1

extern uint16_t CRC_INIT;

#endif
