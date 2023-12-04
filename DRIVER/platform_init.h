/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/

#ifndef __platform_init_h_
#define __platform_init_h_

void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void TimeInit(void);
void USART1_Configuration(void);
void UART4_Configuration(void);
//void PVD_Init(void);
void Power_EXTI_Init(void);
void Limit_EXTI_Init(void);


#endif

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team *****END OF FILE****/

