/**
  ******************************************************************************
  * @file    Remote_Control.h
  * @author  Ginger
  * @version V1.0.0
  * @date    2015/11/14
  * @brief   
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
	
#ifndef __RC__
#define __RC__
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#define RC_Frame_Lentgh		18

#define Key_W ((uint16_t)1 << 0)
#define Key_S ((uint16_t)1 << 1)
#define Key_A ((uint16_t)1 << 2)
#define Key_D ((uint16_t)1 << 3)
#define Key_SHIFT ((uint16_t)1 << 4)
#define Key_CTRL ((uint16_t)1 << 5)
#define Key_Q ((uint16_t)1 << 6)
#define Key_E ((uint16_t)1 << 7)
#define Key_R ((uint16_t)1 << 8)
#define Key_F ((uint16_t)1 << 9)
#define Key_G ((uint16_t)1 << 10)
#define Key_Z ((uint16_t)1 << 11)
#define Key_X ((uint16_t)1 << 12)
#define Key_C ((uint16_t)1 << 13)
#define Key_V ((uint16_t)1 << 14)
#define Key_B ((uint16_t)1 << 15)


typedef struct {
	int16_t ch1;	//each ch value from -364 -- +364
	int16_t ch2;
	int16_t ch3;
	int16_t ch4;
	
	uint8_t switch_left;	//3 value
	uint8_t switch_right;
	
	struct {
		int16_t x;
		int16_t y;
		int16_t z;
	
		uint8_t press_left;
		uint8_t press_right;
	}mouse;
	
	struct {
		uint16_t key_code;
/**********************************************************************************
   * ¼üÅÌÍ¨µÀ:15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
   *          V    C    X	  Z    G    F    R   E   Q  CTRL  SHIFT  D   A   S   W
************************************************************************************/

	}keyBoard;
	

}RC_Type;



enum{
	Switch_Up = 1,
	Switch_Middle = 3,
	Switch_Down = 2,
};
enum{
	W = 1,
	S = 2,
	A = 3,
	D =	4,
	SHIFT =5,
	CTRL =6,
	Q =7,
	E =8,
	R =9,
	F =10,
	G =11,
	Z =12,
	X =13,
	C =14,
	V =15,
	B =16	
};

extern RC_Type remote_control;
extern uint32_t  Latest_Remote_Control_Pack_Time ;
void Callback_RC_Handle(RC_Type* rc, uint8_t* buff);
#endif


