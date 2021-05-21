#ifndef __BUZZER_H__
#define __BUZZER_H__

#include "main.h"
#include <stdbool.h>

#define Buzzer_Break_Time 100

enum{
	Short_Time=100,
	Middle_Time=500,
	Long_Time=1000
};

extern uint32_t Buzzer_cnt;
extern uint32_t Buzzer_On_Time;
extern uint32_t Buzzer_Off_Time;
extern bool Buzzer_Busy;
extern bool Buzzer_Working;

void Buzzer_Init(void);
void Buzzer_ON(void);
void Buzzer_OFF(void);
void Buzzer_ms(uint16_t cnt, uint16_t On_Time, uint16_t Off_Time);

#endif
