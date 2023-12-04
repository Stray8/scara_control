/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/

#ifndef __backtoorigin_h_
#define __backtoorigin_h_

#include "StatusControl.h"

extern u8 Reset_Step;
extern u8 JDZ_Machine_Ori_Axis_Num;
extern u8 Axis_Machine_Origin_Flag;									//ª˙–µªÿ¡„
extern u8 JDZ_Origin_Resetting_Axis_Num;
extern u8 JDZ_Origin_Setting_Axis_Num;
extern u8 JDZ_SetOrigin_Flag;
extern u8  Robot_Enable;

extern void RobotEnableOrigin(void);
extern void BackToOrigin(void);
extern void XAxis_BackToOrigin(void);
extern void ZAxis_BackToOrigin(void);
extern void LAxis_BackToOrigin(void);
extern void OAxis_BackToOrigin(void);
extern void Axis_Set_Origin(void);
extern void Axis_Machine_Origin(void);

#endif



/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team *****END OF FILE****/

