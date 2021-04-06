/*
 * @Descripttion: 串口数据处理
 * @version: V1.0
 * @Author: Xpj
 * @LastEditors: Xpj
 * @Date: 2020-10-26 18:04:29
 */
#ifndef _USARTINFO_H
#define _USARTINFO_H

#include "main.h"
#define JUDGEMENT_BUF_LEN      200
#define VIEW_BUF_LEN          100
#define GROY_DATA_BUF_LEN   200

extern float eular[3]; //欧拉角 eular[0]==Pitch eular[1]==Roll eular[2]==Yaw 
extern int16_t gyo[3];
extern uint8_t Groy_Data_Buf[];
extern uint8_t Judgement_Buf[];
extern uint8_t View_Buf[];
extern uint8_t UART_Buffer[36];
extern uint8_t view_shoot_mode ;
extern uint8_t view_send_state;
extern uint8_t Hi229_Update;

typedef struct
{
	float pitch;
	float yaw;
	int16_t frame_id;

} frame_judge;
typedef union
{
 int16_t d;
 uint8_t c[2];
} int162uchar;
typedef union
{
 float f;
 uint8_t c[4];
} float2uchar;

extern uint8_t extern_view_send_state;
extern float2uchar send_pitch;
extern float2uchar send_yaw;
extern uint8_t Aimming;

void Uart1_TransmissionT_Data(uint8_t *p_data, uint32_t size);
void Uart6_TransmissionT_Data(uint8_t *p_data, uint32_t size);
#endif
