/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/

#ifndef __MD_h_
#define __MD_h_

#include "stm32f4xx.h"
#include "Parameter.h"

#define SCARA_PI											(3.1415926f)										//定义Π的rad值
#define SCARA_JOINT_NUM								4															//Scara关节个数

//关节角度
typedef struct 
{
	float theta[SCARA_JOINT_NUM];	//4个旋转轴的角度
																//theta[0]是关节1转过的角度，一般为关节电机
																//theta[1]是关节2转过的角度，一般为关节电机
																//theta[2]是关节3转过的角度，一般为垂直方向的丝杆
																//theta[3]是关节4转过的角度，一般为顶端的关节电机
}ST_SCARA_JOINT_ANGLE;					


//关节位置
typedef struct 
{
	float x;											//x和y为平面坐标系中的位置
	float y;
	float z;											//z是垂直方向的下降高度
	float c;											//末端与平面坐标系X轴的夹角
}ST_SCARA_JOINT_POS;


extern u8 MD_ReadParData(void);
extern u8 MD_ReadCurPoint(ST_MDPostion *sPostion, u8 pointType);
extern u8 MD_StackCount(s32 type, s32 calSymbol, s32 value);
extern u8 MD_LayerNumJudge(s32 type, s32 value);
extern u8 MD_LayerFullJudge(s32 type, s32 value);
extern u8 MD_StackFullJudge(s32 type, s32 value);
extern u8 MD_GoodNumJudge(s32 type, s32 value);


#endif


/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team *****END OF FILE****/


