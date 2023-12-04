/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ****************
* File Name          : out.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This file provides all the GPIO output functions.
******************************************************************************/
#include "stm32f4xx.h"
#include "out.h"
#include "Delay.h"
#include "Manual.h"	
#include "Parameter.h"

u8 Output_Status[5] = {0xff, 0xff, 0xff , 0xff, 0xff};//用于保存输出口状态
u8 IO_NumberSet[3] = {0x00, 0x00, 0x00};//用于保存输出口状态
/**************************************************************************************************
**  函数名：  OutputInit()
**	输入参数：  
**	输出参数：
**	函数功能：输出口初始化
**	备注：	  
**  作者：    
**  开发日期：
***************************************************************************************************/
void OutputInit(void)
{
	u16 i = 0;
	GPIO_InitTypeDef GPIO_InitStructureOut;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG | RCC_AHB1Periph_GPIOF | RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOD | \
								RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOB| RCC_AHB1Periph_GPIOA, ENABLE);
	
	/***************485控制信号输出端口配置*****************/
	GPIO_InitStructureOut.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructureOut.GPIO_OType = GPIO_OType_PP;    	//推挽输出
	GPIO_InitStructureOut.GPIO_Speed = GPIO_Speed_100MHz;	//100MHz
	GPIO_InitStructureOut.GPIO_PuPd = GPIO_PuPd_UP;    		//上拉
	
	//PD4,外部扩展
	GPIO_InitStructureOut.GPIO_Pin=GPIO_Pin_4;
	GPIO_Init(GPIOD,&GPIO_InitStructureOut);
	GPIO_ResetBits(GPIOD,GPIO_Pin_4);

	//PC9,手持器
	GPIO_InitStructureOut.GPIO_Pin=GPIO_Pin_9;
	GPIO_Init(GPIOC,&GPIO_InitStructureOut);
	GPIO_ResetBits(GPIOC, GPIO_Pin_9);	
	
	/***************信号输出端口配置*****************/
	GPIO_InitStructureOut.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructureOut.GPIO_OType = GPIO_OType_OD;    	//开漏输出
	GPIO_InitStructureOut.GPIO_Speed = GPIO_Speed_100MHz;	//100MHz
	GPIO_InitStructureOut.GPIO_PuPd = GPIO_PuPd_DOWN;    	//下拉

	//B口
	GPIO_InitStructureOut.GPIO_Pin=GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
	GPIO_Init(GPIOB,&GPIO_InitStructureOut);
	GPIO_SetBits(GPIOB,GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5);
	
	//C口
	GPIO_InitStructureOut.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_Init(GPIOC,&GPIO_InitStructureOut);
	GPIO_SetBits(GPIOC,GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
	
	//D口
	GPIO_InitStructureOut.GPIO_Pin=GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_7;
	GPIO_Init(GPIOD,&GPIO_InitStructureOut);
	GPIO_SetBits(GPIOD,GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_7);

	//E口
	GPIO_InitStructureOut.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6;
	GPIO_Init(GPIOE,&GPIO_InitStructureOut);
	GPIO_SetBits(GPIOE,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6);
	
	for(i=0; i<=OUTPUT_NUM; i++)
	{
		SetOutput(i);
	}
	
	SetOutput(O_RUN_GREEN_LIGHT);				//运行
	SetOutput(O_WAIT_YELLOW_LIGHT);			//待机
	ResetOutput(O_ALARM_RED_LIGHT);			//报警，开机过程使用报警提示灯
	SetOutput(O_ALARM_BUZZER);	  			//蜂鸣器
}

/**************************************************************************************************
**  函数名：  SetOutput()
**	输入参数：u8 IO_Num:需要输出的端口  
**	输出参数：无
**	函数功能：置位端口电平信号
**	备注：	  
**  作者：    
**  开发日期：
***************************************************************************************************/
void SetOutput(u8 IO_Num)
{
	switch (IO_Num)
	{
		case 0://Y0
			Y0_OUT_PORT_SET;
			if(	JXS_Parameter.OutputAssociate[0]==1)	//Y0Y1关联
			{
				Y1_OUT_PORT_RESET;
			}
			break;
		case 1://Y1
			Y1_OUT_PORT_SET;
			if(	JXS_Parameter.OutputAssociate[0]==1)	//Y0Y1关联
			{
				Y0_OUT_PORT_RESET;
			}
			break;
		case 2://Y2
			Y2_OUT_PORT_SET;
			if(	JXS_Parameter.OutputAssociate[1]==1)	//Y2Y3关联
			{
				Y3_OUT_PORT_RESET;
			}
			break;
		case 3://Y3
			Y3_OUT_PORT_SET;
			if(	JXS_Parameter.OutputAssociate[1]==1)	//Y2Y3关联
			{
				Y2_OUT_PORT_RESET;
			}
			break;
		case 4://Y4
			Y4_OUT_PORT_SET;
			break;
		case 5://Y5
			Y5_OUT_PORT_SET;
			break;
		case 6://Y6
			Y6_OUT_PORT_SET;
			break;
		case 7://Y7
			Y7_OUT_PORT_SET;
			break;
		case 8://Y8
			Y8_OUT_PORT_SET;
			break;
		case 9://Y9
			Y9_OUT_PORT_SET;
			break;
		case 10://Y10
			Y10_OUT_PORT_SET;
			break;
		case 11://Y11
			Y11_OUT_PORT_SET;
			break;
		case 12://Y12
			Y12_OUT_PORT_SET;
			break;
		case 13://Y13
			Y13_OUT_PORT_SET;
			break;
		case 14://Y14
			Y14_OUT_PORT_SET;
			break;
		case 15://Y15
			Y15_OUT_PORT_SET;
			break;
		case 16://Y16
			Y16_OUT_PORT_SET;
			break;
		case 17://Y17
			Y17_OUT_PORT_SET;
			break;
	}
}


/**************************************************************************************************
**  函数名：  ResetOutput()
**	输入参数：u8 IO_Num:需要输出的端口  
**	输出参数：无
**	函数功能：复位端口电平信号
**	备注：	  
**  作者：    
**  开发日期：
***************************************************************************************************/
void ResetOutput(u8 IO_Num)
{
	switch (IO_Num)
	{
			case 0://Y0
				Y0_OUT_PORT_RESET;
				if(	JXS_Parameter.OutputAssociate[0]==1)	//Y0Y1关联
				{
					Y1_OUT_PORT_SET;
				}
				break;
			case 1://Y1
				Y1_OUT_PORT_RESET;
				if(	JXS_Parameter.OutputAssociate[0]==1)	//Y0Y1关联
				{
					Y0_OUT_PORT_SET;
				}
				break;
			case 2://Y2
				Y2_OUT_PORT_RESET;
				if(	JXS_Parameter.OutputAssociate[1]==1)	//Y2Y3关联
				{
					Y3_OUT_PORT_SET;
				}
				break;
			case 3://Y3
				Y3_OUT_PORT_RESET;
				if(	JXS_Parameter.OutputAssociate[1]==1)	//Y2Y3关联
				{
					Y2_OUT_PORT_SET;
				}
				break;
			case 4://Y4
				Y4_OUT_PORT_RESET;
				break;
			case 5://Y5
				Y5_OUT_PORT_RESET;
				break;
			case 6://Y6
				Y6_OUT_PORT_RESET;
				break;
			case 7://Y7
				Y7_OUT_PORT_RESET;
				break;
			case 8://Y8
				Y8_OUT_PORT_RESET;
				break;
			case 9://Y9
				Y9_OUT_PORT_RESET;
				break;
			case 10://Y10
				Y10_OUT_PORT_RESET;
				break;
			case 11://Y11
				Y11_OUT_PORT_RESET;
				break;
			case 12://Y12
				Y12_OUT_PORT_RESET;
				break;
			case 13://Y13
				Y13_OUT_PORT_RESET;
				break;
			case 14://Y14
				Y14_OUT_PORT_RESET;
				break;
			case 15://Y15
				Y15_OUT_PORT_RESET;
				break;
			case 16://Y16
				Y16_OUT_PORT_RESET;
				break;
			case 17://Y17
				Y17_OUT_PORT_RESET;
				break;
		}
}

/******************* (C) COPYRIGHT 2012 Kingrobot manipulator Team *****END OF FILE****/
