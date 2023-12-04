/*************** (C) COPYRIGHT 2012 Kingrobot manipulator Team ****************
* File Name          : Delay.c
* Author             : Fenkella Zhou
* Version            : V1.0.0
* Date               : 20/06/2012
* Description        : This file delimits all the delay functions.
******************************************************************************/
#ifndef __delay_h_
#define __delay_h_	

#include "stm32f4xx.h"

void delay_init(u8 SYSCLK);//��ʱ������ʼ��
void delay_ms(u32 nms);//ms����ʱ����
void delay_us(u32 nus);//us����ʱ����

void DelayNus(vu32);
void DelayNms(vu32);	
void Delay(u32);
void TimeInit(void);


#endif

/******************* (C) COPYRIGHT 2012 Kingrobot manipulator Team *****END OF FILE****/
