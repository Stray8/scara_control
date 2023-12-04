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

#define SCARA_PI											(3.1415926f)										//���妰��radֵ
#define SCARA_JOINT_NUM								4															//Scara�ؽڸ���

//�ؽڽǶ�
typedef struct 
{
	float theta[SCARA_JOINT_NUM];	//4����ת��ĽǶ�
																//theta[0]�ǹؽ�1ת���ĽǶȣ�һ��Ϊ�ؽڵ��
																//theta[1]�ǹؽ�2ת���ĽǶȣ�һ��Ϊ�ؽڵ��
																//theta[2]�ǹؽ�3ת���ĽǶȣ�һ��Ϊ��ֱ�����˿��
																//theta[3]�ǹؽ�4ת���ĽǶȣ�һ��Ϊ���˵Ĺؽڵ��
}ST_SCARA_JOINT_ANGLE;					


//�ؽ�λ��
typedef struct 
{
	float x;											//x��yΪƽ������ϵ�е�λ��
	float y;
	float z;											//z�Ǵ�ֱ������½��߶�
	float c;											//ĩ����ƽ������ϵX��ļн�
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


