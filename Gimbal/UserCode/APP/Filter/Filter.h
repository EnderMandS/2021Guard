#ifndef __FILTER_H__
#define __FILTER_H__

extern float first_order_filter_X_cali(float input);
extern float first_order_filter_Y_cali(float input);
void Data_Common(int* Data_Tab,int p,int *Idx,int *Sum,int MagBARO_TAB_SIZE);
void Data_Float_Common(float* Data_Tab,float p,int *Idx,float *Sum,int MagBARO_TAB_SIZE);

#endif
