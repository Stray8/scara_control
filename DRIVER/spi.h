/*************** (C) COPYRIGHT 2019 Kingrobot manipulator Team ****************
* File Name          : w25qxx.c
* Author             : Fenkella Zhou
* Version            : V1.0.0
* Date               : 06/08/2019
* Description        : This file is complete the SPI settings.
******************************************************************************/

#ifndef  SPIINT_H
#define  SPIINT_H

#include "stm32f4xx.h"
//#include "stm32f10x_lib.h"		
#include "Delay.h"
#include "out.h" 
#include "sys.h" 
 				  	    													  
void SPI3_Init(void);			 //��ʼ��SPI��
u8 SPI3_ReadWriteByte(u8 TxData);//SPI���߶�дһ���ֽ�
 
#endif


