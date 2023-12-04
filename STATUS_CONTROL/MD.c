/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : Auto.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/
#include "stm32f4xx.h"
#include "MD.h"
#include "Parameter.h"
#include "w25qxx.h"
#include "Auto_2.h"
#include "ActionOperate.h"
#include "SignalWatch.h" 
#include "math.h" 
#include <stdio.h> 
#include <stdlib.h>

/**************************************************************************************************
**  ��������  FindHandcoor()
**	����������ؽ�2�ĽǶ�theta2����λrad
**	�����������������ϵ��1����ϵ 0����ϵ
**	�������ܣ�����theta2����scara�����˵�ǰ��ϵ
**	��ע��	  
**				����ϵ��theta2��Χ(0,��)U(-2��,-��)
**				����ϵ��theta2��Χ(-��,0)U(��,2��)
**				����λ�ã�theta2ȡֵ{-2��,-��,0,��,2��}
***************************************************************************************************/
u8 ScaraFindHandcoor(float theta2)
{
	u8 handcoor = 0;
	
	if((theta2 > 0.0f && theta2 < SCARA_PI) || (theta2 > -2.0f * SCARA_PI && theta2 < -SCARA_PI))
	{
		handcoor = 1;
	}
	else
	{
		handcoor = 0;
	}
	
	return handcoor;
}

/**************************************************************************************************
**  ��������  FindFlagJ1()
**	����������ؽڵĽǶ�theta1����λrad
**	���������scara�����˵�ǰJ1�ؽڱ�־λ
**	�������ܣ�����scara�����˵�ǰJ1�ؽڱ�־λ
**	��ע��	  
**				FlagJ1=0��theta1��Χ[-��,��];����FlagJ1=1
***************************************************************************************************/
u8 ScaraFindFlagJ1(float theta1)
{
	u8 flagJ1 = 0;
	
	if(theta1 >= -SCARA_PI && theta1 <= SCARA_PI)
	{
		flagJ1 = 0;
	}
	else
	{
		flagJ1 = 1;
	}
	
	return flagJ1;
}

/**************************************************************************************************
**  ��������  FindFlagJ2()
**	����������ؽڵĽǶ�theta2����λrad
**	���������scara�����˵�ǰJ2�ؽڱ�־λ
**	�������ܣ�����scara�����˵�ǰJ2�ؽڱ�־λ
**	��ע��	  
**				FlagJ2=0��theta2��Χ[-��,��];����FlagJ2=1
***************************************************************************************************/
u8 ScaraFindFlagJ2(float theta2)
{
	u8 flagJ2 = 0;
	
	if(theta2 >= -SCARA_PI && theta2 <= SCARA_PI)
	{
		flagJ2 = 0;
	}
	else
	{
		flagJ2 = 1;
	}
	
	return flagJ2;
}

/**************************************************************************************************
**  ��������  ScaraForwardKinematics()
**	�����������۳�(mm),С�۳�(mm),˿���ݾ�(mm),�����˹ؽڽǶ�(rad),�����˹ؽ�λ��(mm��rad)
**	���������scara�������˶�ѧ����
**	�������ܣ�����scara������ĩ��λ��(mm��rad)
**	��ע��	  
***************************************************************************************************/
u8 ScaraForwardKinematics(float L1, float L2, float screw, ST_SCARA_JOINT_ANGLE jointAngle, ST_SCARA_JOINT_POS *jointPos)
{
	u8 ret = 0;
	
	jointPos->x = L1 * cos(jointAngle.theta[0]) + L2 * cos(jointAngle.theta[0] + jointAngle.theta[1]);
	
	jointPos->y = L1 * sin(jointAngle.theta[0]) + L2 * sin(jointAngle.theta[0] + jointAngle.theta[1]);

	jointPos->z = jointAngle.theta[2] * screw / (2 * SCARA_PI);
	
	jointPos->c = jointAngle.theta[0] + jointAngle.theta[1] + jointAngle.theta[3];
	
	return ret;
}

/**************************************************************************************************
**  ��������  ScaraInverseKinematics()
**	�����������۳�(mm),С�۳�(mm),˿���ݾ�(mm),�����˹ؽ�λ��(mm��rad),
**						�����˵�ǰ��ϵ, �����˵�ǰJ1�ؽڱ�־λ, �����˵�ǰJ2�ؽڱ�־λ, ��Ҫ�õ��Ļ����˹ؽڽǶ�(rad)
**	���������scara�������˶�ѧ���
**	�������ܣ�����scara������ĩ��λ��(mm��rad)
**	��ע��	  
***************************************************************************************************/
u8 ScaraInverseKinematics(float L1, float L2, float screw, ST_SCARA_JOINT_POS jointPos, u8 handcoor, u8 flagJ1, u8 flagJ2, ST_SCARA_JOINT_ANGLE *jointAngle)
{
	u8 ret = 0;
	float c2 = 0.0f;
	float s2 = 0.0f;
	float calErr = 0.0001f;					//����������
	float temp = 0.0f;
	
	/*��ĩ����(x,y)����ϵ�У�C2����[-1,1]������ڼ��㾫�ȣ�c2����ֵ������΢����1*/
	c2 = (jointPos.x * jointPos.x + jointPos.y * jointPos.y - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
	temp = 1.0f - c2 * c2;
	if(temp < 0.0f)
	{
		if(temp > -calErr)
		{
			temp = 0.0f;
		}
    else
		{
			return 1;
		}
	}
	
	if(handcoor == 0)
	{//����ϵ
		jointAngle->theta[1] = atan2(-sqrt(temp), c2);
	}
	else
	{//����ϵ
		jointAngle->theta[1] = atan2(sqrt(temp), c2);
	}
	
	s2 = sin(jointAngle->theta[1]);
	jointAngle->theta[0] = atan2(jointPos.y, jointPos.x) - atan2(L2 * s2, L1 + L2 * c2);
	
	if(screw == 0.0f)
	{
		jointAngle->theta[2] = 0;
	}
	else
	{
		jointAngle->theta[2] = 2.0f * SCARA_PI * jointPos.z / screw;
	}
	
	if(jointAngle->theta[0] <= -SCARA_PI)
	{
		jointAngle->theta[0] = jointAngle->theta[0] + 2.0f * SCARA_PI;
	}
	else if(jointAngle->theta[0] >= SCARA_PI)
	{
		jointAngle->theta[0] = jointAngle->theta[0] - 2.0f * SCARA_PI;
	}
	
	if(flagJ1 == 1)
	{
		if(jointAngle->theta[0] >= 0.0f)
		{
			jointAngle->theta[0] = jointAngle->theta[0] - 2.0f * SCARA_PI;
		}
		else
		{
			jointAngle->theta[0] = jointAngle->theta[0] + 2.0f * SCARA_PI;
		}
	}
	
	if(flagJ2 == 1)
	{
		if(jointAngle->theta[1] >= 0.0f)
		{
			jointAngle->theta[1] = jointAngle->theta[1] - 2.0f * SCARA_PI;
		}
		else
		{
			jointAngle->theta[1] = jointAngle->theta[1] + 2.0f * SCARA_PI;
		}
	}
	
	jointAngle->theta[3] = jointPos.c - jointAngle->theta[0] - jointAngle->theta[1];

	return ret;
}

/**************************************************************************************************
**  ��������  MD_ReadParData()
**	���������
**	���������
**	�������ܣ�����ƫ�Ƽ���
**	��ע��	  
**  ���ߣ�		
**  ��������:
***************************************************************************************************/
u8 MD_GoodOffset_Calculate(s32 *point)
{
	u16 i = 0;
	u8 flagJ1 = 0;
	u8 flagJ2 = 0;
	u8 handcoor = 0;
	u8 ret = 0;
	ST_SCARA_JOINT_ANGLE sJointAngle = {0};		//�����˹ؽڽǶ�(rad)
	ST_SCARA_JOINT_POS sJointPos = {0};				//�����˹ؽ�λ��(mm��rad)
	s32 offsetPostion[Axis_Num] = {0};				//������ϵ-ƫ������
	u8 handturn_flag = 0;											//λ�ò��ܵ����ת��ϵ�ٴμ��㣬�������ܵ�������г���
	float screw = 0.0f;												//�ݾ�
	s32 pointTemp[Axis_Num] = {0};						//�����걣��
	
	for(i=0; i<Axis_Num; i++)
	{
		pointTemp[i] = point[i];
		
		if(sCartesian_Para.axisBackMinDir[i] == 0)
		{
			offsetPostion[i] = point[i] - JXS_Parameter.OrignOffset[i];
		}
		else
		{
			offsetPostion[i] = JXS_Parameter.OrignOffset[i] - point[i];
		}
		
		if(offsetPostion[i] == 0)
		{
			sJointAngle.theta[i] = 0.0f;
		}
		else
		{
			sJointAngle.theta[i] = offsetPostion[i] * SCARA_PI / 18000.0f;
		}
	}
	
	flagJ1 = ScaraFindFlagJ1(sJointAngle.theta[0]);
	flagJ2 = ScaraFindFlagJ2(sJointAngle.theta[1]);
	handcoor = ScaraFindHandcoor(sJointAngle.theta[1]);
	if(sCartesian_Para.axisType[Z_Axsis] == 1)
	{//�������ת�����Ҫ����˿���ݾ�
		screw = (float)sCartesian_Para.pitchLength / 100;
	}
	else
	{
		screw = 0.0f;
	}
	
	ret = ScaraForwardKinematics((float)sCartesian_Para.length[0] / 100, (float)sCartesian_Para.length[1] / 100, screw, sJointAngle, &sJointPos);//����
	if(ret)
	{
		return 1;
	}
	
	sJointPos.x = sJointPos.x + (float)sMD_Parameter.goodOffset[X_Axsis] / 100;
	sJointPos.y = sJointPos.y + (float)sMD_Parameter.goodOffset[L_Axsis] / 100;
	if(sCartesian_Para.axisType[Z_Axsis] == 1)
	{//�������ת�����Ҫ����˿�˸߶�
		sJointPos.z = sJointPos.z + (float)sMD_Parameter.goodOffset[Z_Axsis] / 100;
	}
	else
	{
		sJointPos.z = 0.0f;
	}
	sJointPos.c = sJointPos.c + (float)sMD_Parameter.goodOffset[O_Axsis] / 100;
	ret = ScaraInverseKinematics((float)sCartesian_Para.length[0] / 100, (float)sCartesian_Para.length[1] / 100, screw, sJointPos, handcoor, flagJ1, flagJ2, &sJointAngle);//���
	if(ret)
	{
		return 1;
	}
	
	for(i=0; i<Axis_Num; i++)
	{
		if(sCartesian_Para.axisType[i] == 1)
		{//������Ϊ��ת��ʱ����Ϊ�ѿ�������ϵ�е�����Ҫת����������ֻ��Ҫ����ֱ��ƫ��
			offsetPostion[i] = sJointAngle.theta[i] * 18000.0f / SCARA_PI;
			
			if(sCartesian_Para.axisBackMinDir[i] == 0)
			{
				point[i] = offsetPostion[i] + JXS_Parameter.OrignOffset[i];
			}
			else
			{
				point[i] = JXS_Parameter.OrignOffset[i] - offsetPostion[i];
			}
			
			if(point[i] < 0 || (Robot_SoftLimit[i].Switch_Limit == TRUE && point[i] > Robot_SoftLimit[i].Right_Limit))//(s32)((Axsis_Maxlength[i] - MINROBOTPOSITION) * 100 / Step_Coefficient[i])))
			{//����������겻�ڰ�ȫ���ڣ���Ҫ������ϵ���¼���
				handturn_flag = 1;
				break;
			}
		}
		else
		{//������ֻ��Ҫ����ֱ��ƫ��
			point[i] = pointTemp[i] + sMD_Parameter.goodOffset[i];
		}
	}
	
	if(handturn_flag)
	{//����õ���λ�ò��ڰ�ȫ���ڣ���ת��ϵ�ٴμ��㣬�������ܵ�������г���
		if(handcoor == 0)
		{//�л���ϵ
			handcoor = 1;
		}
		else
		{
			handcoor = 0;
		}
		
		//������Z�ᣬscrew=0
		ret = ScaraInverseKinematics((float)sCartesian_Para.length[0] / 100, (float)sCartesian_Para.length[1] / 100, 0, sJointPos, handcoor, flagJ1, flagJ2, &sJointAngle);//���
		if(ret)
		{
			return 1;
		}
		
		for(i=0; i<Axis_Num; i++)
		{
			if(sCartesian_Para.axisType[i] == 1)
			{//������Ϊ��ת��ʱ����Ϊ�ѿ�������ϵ�е�����Ҫת����������ֻ��Ҫ����ֱ��ƫ��
				offsetPostion[i] = sJointAngle.theta[i] * 18000.0f / SCARA_PI;
				
				if(sCartesian_Para.axisBackMinDir[i] == 0)
				{
					point[i] = offsetPostion[i] + JXS_Parameter.OrignOffset[i];
				}
				else
				{
					point[i] = JXS_Parameter.OrignOffset[i] - offsetPostion[i];
				}
				
				if(point[i] < 0 || (Robot_SoftLimit[i].Switch_Limit == TRUE && point[i] > Robot_SoftLimit[i].Right_Limit))//(s32)((Axsis_Maxlength[i] - MINROBOTPOSITION) * 100 / Step_Coefficient[i])))
				{//����������겻�ڰ�ȫ���ڣ�����
					return 1;
				}
			}
			else
			{//������ֻ��Ҫ����ֱ��ƫ��
				point[i] = pointTemp[i] + sMD_Parameter.goodOffset[i];
			}
		}
	}
	
	return 0;
}

/*************************************************************************
**  ��������  MD_PointPostion_Calculate(u8 proNum)
**	����������ѿ�������ϵ��postion1��postion2��postion3Ϊ�����ϲֵ������㣬postion_CurΪ��ǰҪ����ĵ�
**	�����������
**	�������ܣ�����scaraʽ��ǰ�����λ��
**	��ע��	  ��
**  ���ߣ�    
**  �������ڣ�
**************************************************************************/
u8 MD_PointPostion_Calculate(ST_SCARA_JOINT_POS postion1, ST_SCARA_JOINT_POS postion2, ST_SCARA_JOINT_POS postion3, ST_SCARA_JOINT_POS *postion_Cur)
{
	u8 H_count=0, V_count=0;				//��ǰ��������ڼ���
	float HD=0.0f, VD=0.0f;         //����������
	float DeltaH=0.0f, DeltaV=0.0f;	//�������򲹳�
//	float HDZ=0.0f, VDZ=0.0f;     //�ᡢ��Z����
		
	if(sMD_Parameter.horNum == 0 || sMD_Parameter.verNum == 0 || sMD_Parameter.stackLayer == 0 || sMD_Parameter.horNum > 200 || sMD_Parameter.verNum > 200 || sMD_Parameter.stackLayer > 200)
	{
		return 1;
	} 
	if(sMD_Parameter.horNum*sMD_Parameter.verNum<sMD_RunPara.curNum || sMD_Parameter.stackLayer < sMD_RunPara.curLayer)
	{
		return 1;
	}
	
	//���㵱ǰ����λ��
	//������Z��
	if(sMD_Parameter.horNum == 1 && sMD_Parameter.verNum == 1)
	{
		postion_Cur->x = postion1.x;
		postion_Cur->y = postion1.y;
		postion_Cur->z = postion1.z;
		postion_Cur->c = postion1.c;
	}
	else if(sMD_Parameter.horNum >= 1 && sMD_Parameter.verNum >= 1)
	{
		if(sMD_Parameter.horNum == 1)
		{
			HD = 0.0f;
			DeltaV = 0.0f;
//			HDZ = 0.0f;
		}
		else
		{
			HD = (postion2.x-postion1.x) / (sMD_Parameter.horNum-1);  		//ȡ��
			DeltaV = (postion2.y-postion1.y)/(sMD_Parameter.horNum-1); 	//�ڶ��е�ƫ��
//			HDZ = (postion2.z-postion1.z)/(H_num-1);
		}
		
	  if(sMD_Parameter.verNum==1)
		{
			VD = 0.0f;
			DeltaH = 0.0f;
//			VDZ = 0.0f;
		}
		else
		{
				VD = (postion3.y-postion2.y)/(sMD_Parameter.verNum-1);
				DeltaH = (postion3.x-postion2.x)/(sMD_Parameter.verNum-1); 	//�ڶ��е�ƫ��
//				VDZ = (postion3.z-postion2.z)/(V_num-1);
		}
		
		//����ƽ����
		H_count = (sMD_RunPara.curNum-1)/sMD_Parameter.horNum;//+1
		V_count = (sMD_RunPara.curNum-1)%sMD_Parameter.horNum;//+1
		postion_Cur->x = postion1.x + V_count*HD + H_count*DeltaH;
		postion_Cur->y = postion1.y + H_count*VD + V_count*DeltaV;
		postion_Cur->z = postion1.z;// + V_count*HDZ + H_count*VDZ
		postion_Cur->c = postion1.c;
	}
	return 0;
}

/*************************************************************************
**  ��������  MD_TrussMatrix_Calculate(u8 proNum)
**	���������postion1��postion2��postion3Ϊ�����ϲֵ������㣬postion_CurΪ��ǰҪ����ĵ�
**	�����������
**	�������ܣ��������ʽ��ǰ�����λ��
**	��ע��	  ��
**  ���ߣ�    
**  �������ڣ�
**************************************************************************/
u8 MD_TrussMatrix_Calculate(ST_MDPostion postion1, ST_MDPostion postion2, ST_MDPostion postion3, ST_MDPostion *postion_Cur)
{
	u8 H_count=0, V_count=0;				//��ǰ��������ڼ���
	float HD=0.0f, VD=0.0f;         //����������
	float DeltaH=0.0f, DeltaV=0.0f;	//�������򲹳�
//	float HDZ=0.0f, VDZ=0.0f;     //�ᡢ��Z����
		
	if(sMD_Parameter.horNum == 0 || sMD_Parameter.verNum == 0 || sMD_Parameter.stackLayer == 0 || sMD_Parameter.horNum > 200 || sMD_Parameter.verNum > 200 || sMD_Parameter.stackLayer > 200)
	{
		return 1;
	} 
	if(sMD_Parameter.horNum*sMD_Parameter.verNum<sMD_RunPara.curNum || sMD_Parameter.stackLayer < sMD_RunPara.curLayer)
	{
		return 1;
	}
	
	//���㵱ǰ����λ��
	if(sMD_Parameter.horNum == 1 && sMD_Parameter.verNum == 1)
	{
		postion_Cur->point[X_Axsis] = postion1.point[X_Axsis];
		postion_Cur->point[L_Axsis] = postion1.point[L_Axsis];
		postion_Cur->point[Z_Axsis] = postion1.point[Z_Axsis];
		postion_Cur->point[O_Axsis] = postion1.point[O_Axsis];
		
		postion_Cur->waitPoint[X_Axsis] = postion1.waitPoint[X_Axsis];
		postion_Cur->waitPoint[L_Axsis] = postion1.waitPoint[L_Axsis];
		postion_Cur->waitPoint[Z_Axsis] = postion1.waitPoint[Z_Axsis];
		postion_Cur->waitPoint[O_Axsis] = postion1.waitPoint[O_Axsis];
	}
	else if(sMD_Parameter.horNum >= 1 && sMD_Parameter.verNum >= 1)
	{
		/************************����**************************/
		if(sMD_Parameter.horNum == 1)
		{
			HD = 0.0f;
			DeltaV = 0.0f;
//			HDZ = 0.0f;
		}
		else
		{
			HD = (postion2.point[X_Axsis] - postion1.point[X_Axsis]) / (sMD_Parameter.horNum - 1);  		//ȡ��
			DeltaV = (postion2.point[L_Axsis] - postion1.point[L_Axsis]) / (sMD_Parameter.horNum - 1); 	//�ڶ��е�ƫ��
//			HDZ = (postion2.z-postion1.z)/(H_num-1);
		}
		
	  if(sMD_Parameter.verNum==1)
		{
			VD = 0.0f;
			DeltaH = 0.0f;
//			VDZ = 0.0f;
		}
		else
		{
				VD = (postion3.point[L_Axsis] - postion2.point[L_Axsis]) / (sMD_Parameter.verNum - 1);
				DeltaH = (postion3.point[X_Axsis]-postion2.point[X_Axsis]) / (sMD_Parameter.verNum - 1); 	//�ڶ��е�ƫ��
//				VDZ = (postion3.z-postion2.z)/(V_num-1);
		}
		
		//����ƽ����
		H_count = (sMD_RunPara.curNum-1) / sMD_Parameter.horNum;//+1
		V_count = (sMD_RunPara.curNum-1) % sMD_Parameter.horNum;//+1
		postion_Cur->point[X_Axsis] = postion1.point[X_Axsis] + V_count * HD + H_count * DeltaH;
		postion_Cur->point[L_Axsis] = postion1.point[L_Axsis] + H_count*VD + V_count*DeltaV;
		postion_Cur->point[Z_Axsis] = postion1.point[Z_Axsis];// + V_count*HDZ + H_count*VDZ
		postion_Cur->point[O_Axsis] = postion1.point[O_Axsis];
		
		/************************�ȴ���**************************/
		if(sMD_Parameter.horNum == 1)
		{
			HD = 0.0f;
			DeltaV = 0.0f;
//			HDZ = 0.0f;
		}
		else
		{
			HD = (postion2.waitPoint[X_Axsis] - postion1.waitPoint[X_Axsis]) / (sMD_Parameter.horNum - 1);  		//ȡ��
			DeltaV = (postion2.waitPoint[L_Axsis] - postion1.waitPoint[L_Axsis]) / (sMD_Parameter.horNum - 1); 	//�ڶ��е�ƫ��
//			HDZ = (postion2.z-postion1.z)/(H_num-1);
		}
		
	  if(sMD_Parameter.verNum==1)
		{
			VD = 0.0f;
			DeltaH = 0.0f;
//			VDZ = 0.0f;
		}
		else
		{
				VD = (postion3.waitPoint[L_Axsis] - postion2.waitPoint[L_Axsis]) / (sMD_Parameter.verNum - 1);
				DeltaH = (postion3.waitPoint[X_Axsis]-postion2.waitPoint[X_Axsis]) / (sMD_Parameter.verNum - 1); 	//�ڶ��е�ƫ��
//				VDZ = (postion3.z-postion2.z)/(V_num-1);
		}
		
		//����ƽ����
		H_count = (sMD_RunPara.curNum-1) / sMD_Parameter.horNum;//+1
		V_count = (sMD_RunPara.curNum-1) % sMD_Parameter.horNum;//+1
		postion_Cur->waitPoint[X_Axsis] = postion1.waitPoint[X_Axsis] + V_count * HD + H_count * DeltaH;
		postion_Cur->waitPoint[L_Axsis] = postion1.waitPoint[L_Axsis] + H_count*VD + V_count*DeltaV;
		postion_Cur->waitPoint[Z_Axsis] = postion1.waitPoint[Z_Axsis];// + V_count*HDZ + H_count*VDZ
		postion_Cur->waitPoint[O_Axsis] = postion1.waitPoint[O_Axsis];		
	}
	return 0;
}

/**************************************************************************************************
**  ��������  MD_ReadParData()
**	���������pointType 0���� 1�ȴ���
**	���������
**	�������ܣ�������͵�λ����
**	��ע��	  
**  ���ߣ�		
**  ��������:
***************************************************************************************************/
u8 MD_MatrixPoint_Calculate(ST_MDPostion *sPostion_Teach, ST_MDPostion *sPostion_Cur, u8 pointType)
{
	u16 i = 0, k = 0;
	u8 flagJ1 = 0;
	u8 flagJ2 = 0;
	u8 handcoor = 0;
	float screw = 0.0f;
	u8 ret = 0;
	ST_SCARA_JOINT_ANGLE sJointAngle = {0};		//�����˹ؽڽǶ�(rad)
	ST_SCARA_JOINT_POS sJointPos[3] = {0};		//�����˹ؽ�λ��(mm��rad)-ʾ�̵�������
	ST_SCARA_JOINT_POS sJointPos_Cur = {0};		//�����˹ؽ�λ��(mm��rad)-���㵱ǰ��
	s32 offsetPostion[Axis_Num] = {0};				//������ϵ-ƫ������
	u8 handturn_flag = 0;											//λ�ò��ܵ����ת��ϵ�ٴμ��㣬�������ܵ�������г���
	
	for(k=0; k<3; k++)
	{
		for(i=0; i<Axis_Num; i++)
		{
			if(pointType == 0)
			{
				if(sCartesian_Para.axisBackMinDir[i] == 0)
				{
					offsetPostion[i] = sPostion_Teach[k].point[i] - JXS_Parameter.OrignOffset[i];
				}
				else
				{
					offsetPostion[i] = JXS_Parameter.OrignOffset[i] - sPostion_Teach[k].point[i];
				}
			}
			else if(pointType == 1)
			{
				if(sCartesian_Para.axisBackMinDir[i] == 0)
				{
					offsetPostion[i] = sPostion_Teach[k].waitPoint[i] - JXS_Parameter.OrignOffset[i];
				}
				else
				{
					offsetPostion[i] = JXS_Parameter.OrignOffset[i] - sPostion_Teach[k].waitPoint[i];
				}
			}
			
			if(offsetPostion[i] == 0)
			{
				sJointAngle.theta[i] = 0;
			}
			else
			{
				sJointAngle.theta[i] = offsetPostion[i] * SCARA_PI / 18000.0f;
			}
		}
		flagJ1 = ScaraFindFlagJ1(sJointAngle.theta[0]);
		flagJ2 = ScaraFindFlagJ2(sJointAngle.theta[1]);
		handcoor = ScaraFindHandcoor(sJointAngle.theta[1]);
		if(sCartesian_Para.axisType[Z_Axsis] == 1)
		{//�������ת�����Ҫ����˿���ݾ�
			screw = (float)sCartesian_Para.pitchLength / 100;
		}
		else
		{
			screw = 0.0f;
		}
		ret = ScaraForwardKinematics((float)sCartesian_Para.length[0]/100, (float)sCartesian_Para.length[1]/100, screw, sJointAngle, &sJointPos[k]);//���⣺λ��11��2��3�ĵѿ�������
		if(ret)
		{
			return 1;
		}
	}
	
	//����ƫ����
	ret = MD_PointPostion_Calculate(sJointPos[0], sJointPos[1], sJointPos[2], &sJointPos_Cur);
	if(ret)
	{
		return 1;
	}
	
	//������Z�ᣬscrew=0
	ret = ScaraInverseKinematics((float)sCartesian_Para.length[0]/100, (float)sCartesian_Para.length[1]/100, screw, sJointPos_Cur, handcoor, flagJ1, flagJ2, &sJointAngle);//���
	if(ret)
	{
		return 1;
	}
	for(i=0; i<Axis_Num; i++)
	{
		if(sCartesian_Para.axisType[i] == 1)
		{
			offsetPostion[i] = sJointAngle.theta[i] * 18000.0f / SCARA_PI;
			
			if(pointType == 0)
			{
				if(sCartesian_Para.axisBackMinDir[i] == 0)
				{
					sPostion_Cur->point[i] = offsetPostion[i] + JXS_Parameter.OrignOffset[i];
				}
				else
				{
					sPostion_Cur->point[i] = JXS_Parameter.OrignOffset[i] - offsetPostion[i];
				}
				if(sPostion_Cur->point[i] < 0 || (Robot_SoftLimit[i].Switch_Limit == TRUE && sPostion_Cur->point[i] > Robot_SoftLimit[i].Right_Limit))//(s32)((Axsis_Maxlength[i] - MINROBOTPOSITION) * 100 / Step_Coefficient[i]))
				{
					handturn_flag = 1;
					break;
				}
			}
			else if(pointType == 1)
			{
				if(sCartesian_Para.axisBackMinDir[i] == 0)
				{
					sPostion_Cur->waitPoint[i] = offsetPostion[i] + JXS_Parameter.OrignOffset[i];
				}
				else
				{
					sPostion_Cur->waitPoint[i] = JXS_Parameter.OrignOffset[i] - offsetPostion[i];
				}
				if(sPostion_Cur->waitPoint[i] < 0 || (Robot_SoftLimit[i].Switch_Limit == TRUE && sPostion_Cur->waitPoint[i] > Robot_SoftLimit[i].Right_Limit))//(s32)((Axsis_Maxlength[i] - MINROBOTPOSITION) * 100 / Step_Coefficient[i]))
				{
					handturn_flag = 1;
					break;
				}
			}
		}
		else
		{
			if(pointType == 0)
			{
				sPostion_Cur->point[i] = sPostion_Teach[0].point[i];
			}
			else if(pointType == 1)
			{
				sPostion_Cur->waitPoint[i] = sPostion_Teach[0].waitPoint[i];
			}
		}
	}
	
	if(handturn_flag)
	{//λ�ò��ܵ����ת��ϵ�ٴμ��㣬�������ܵ�������г���
		if(handcoor == 0)
		{
			handcoor = 1;
		}
		else
		{
			handcoor = 0;
		}
		//������Z�ᣬscrew=0
		ret = ScaraInverseKinematics((float)sCartesian_Para.length[0]/100, (float)sCartesian_Para.length[1]/100, 0, sJointPos_Cur, handcoor, flagJ1, flagJ2, &sJointAngle);//���
		if(ret)
		{
			return 1;
		}
		for(i=0; i<Axis_Num; i++)
		{
			if(sCartesian_Para.axisType[i] == 1)
			{
				offsetPostion[i] = sJointAngle.theta[i] * 18000.0f / SCARA_PI;
				
				if(pointType == 0)
				{
					if(sCartesian_Para.axisBackMinDir[i] == 0)
					{
						sPostion_Cur->point[i] = offsetPostion[i] + JXS_Parameter.OrignOffset[i];
					}
					else
					{
						sPostion_Cur->point[i] = JXS_Parameter.OrignOffset[i] - offsetPostion[i];
					}
					if(sPostion_Cur->point[i] < 0 || (Robot_SoftLimit[i].Switch_Limit == TRUE && sPostion_Cur->point[i] > Robot_SoftLimit[i].Right_Limit))//(s32)((Axsis_Maxlength[i] - MINROBOTPOSITION) * 100 / Step_Coefficient[i])))
					{
						return 1;
					}
				}
				else if(pointType == 1)
				{
					if(sCartesian_Para.axisBackMinDir[i] == 0)
					{
						sPostion_Cur->waitPoint[i] = offsetPostion[i] + JXS_Parameter.OrignOffset[i];
					}
					else
					{
						sPostion_Cur->waitPoint[i] = JXS_Parameter.OrignOffset[i] - offsetPostion[i];
					}
					if(sPostion_Cur->waitPoint[i] < 0 || (Robot_SoftLimit[i].Switch_Limit == TRUE && sPostion_Cur->waitPoint[i] > Robot_SoftLimit[i].Right_Limit))//(s32)((Axsis_Maxlength[i] - MINROBOTPOSITION) * 100 / Step_Coefficient[i])))
					{
						return 1;
					}
				}
			}
			else
			{
				if(pointType == 0)
				{
					sPostion_Cur->point[i] = sPostion_Teach[0].point[i];
				}
				else if(pointType == 1)
				{
					sPostion_Cur->waitPoint[i] = sPostion_Teach[0].waitPoint[i];
				}
			}
		}
	}
		
	return 0;
}

/**************************************************************************************************
**  ��������  MD_ReadParData()
**	���������
**	���������
**	�������ܣ���ȡ������
**	��ע��	  
**  ���ߣ�		
**  ��������:
***************************************************************************************************/
u8 MD_ReadParData(void)
{
	u8 IIC_Parameter[80] = {0};
	u32 readStartAddr = 0;
	u16 i = 0;
	
	/*���MD�����в���*/
	if(sMD_RunPara.mdMethed > 1)
	{//ֻ֧������ģʽ�ͷּ�ģʽ
		return 1;
	}
	else if(sMD_RunPara.curGood == 0 || sMD_RunPara.curGood > MD_GOOD_NUM)
	{
		return 1;
	}
	else if(sMD_RunPara.startGood == 0 || sMD_RunPara.startGood > MD_GOOD_NUM)
	{
		return 1;
	}
	else if(sMD_RunPara.totalGood == 0 || sMD_RunPara.totalGood > MD_GOOD_NUM)
	{
		return 1;
	}
//	else if(sMD_RunPara.curLayer == 0)
//	{
//		return 1;
//	}
//	else if(sMD_RunPara.curNum == 0 || sMD_RunPara.curNum > MD_POINT_NUM)
//	{
//		return 1;
//	}
	
	if(sMD_RunPara.curGood != sMD_CurMDCode)
	{//��Ʒ��ŷ����仯ʱ�����¶�ȡ��Ʒ��������
		sMD_CurMDCode = sMD_RunPara.curGood;
		sMD_GoodOffset_flag = 0;
		
		if(sMD_RunPara.mdMethed == 1)
		{//�ּ�ģʽʱ����ǰ��͵�ǰ�������Ҫ�ӷּ�洢����ȡ
			sMD_RunPara.curLayer = sMD_FlashCurLayer[sMD_CurMDCode - 1];
			sMD_RunPara.curNum = sMD_FlashCurNum[sMD_CurMDCode - 1];
		}

		/*��������ȡ*/
		readStartAddr = P_MD_PARA_HEAD	+ P_MD_GOOD_LEN * (sMD_CurMDCode - 1);
		W25QXX_Read(IIC_Parameter, readStartAddr, P_MD_PARA_LEN);
		sMD_Parameter.stackType = IIC_Parameter[12];
		sMD_Parameter.property = IIC_Parameter[13];
		sMD_Parameter.revolveMode = IIC_Parameter[14];
		sMD_Parameter.gasPort = IIC_Parameter[15];
		sMD_Parameter.topSwitch = IIC_Parameter[16];
		sMD_Parameter.goodheight = IIC_Parameter[17] + (((u32)IIC_Parameter[18]) << 8) + (((u32)IIC_Parameter[19]) << 16) + (((u32)IIC_Parameter[20]) << 24);
		sMD_Parameter.stackLayer = IIC_Parameter[21];
		sMD_Parameter.loopLayer = IIC_Parameter[22];
		for(i=0; i<LOOP_MAX; i++)
		{//��ȡ10���ÿ�����
			sMD_Parameter.layerNum[i] = IIC_Parameter[23 + i];
			if(sMD_Parameter.layerNum[i] > MD_POINT_NUM)
			{
				sMD_Parameter.layerNum[i] = MD_POINT_NUM;
			}
		}
		for(i=0; i<Axis_Num; i++)
		{
			sMD_Parameter.goodOffset[i] = (s32)(((s32)IIC_Parameter[23+LOOP_MAX+i*4])|((s32)IIC_Parameter[24+LOOP_MAX+i*4]<<8)|((s32)IIC_Parameter[25+LOOP_MAX+i*4]<<16)|((s32)IIC_Parameter[26+LOOP_MAX+i*4]<<24));
			if(sMD_GoodOffset_flag == 0 && sMD_Parameter.goodOffset[i] != 0)
			{
				sMD_GoodOffset_flag = 1;
			}
		}
		sMD_Parameter.horNum = IIC_Parameter[39 + LOOP_MAX];
		sMD_Parameter.verNum = IIC_Parameter[40 + LOOP_MAX];
		
		/*��������Ʒ�Ļ�������*/
		if(sMD_Parameter.stackType > 1)
		{//֧��ʾ�̡����εĶ���
			return 1;
		}
		else if(sMD_Parameter.property > 1)
		{//֧�����Ͳ��
			return 1;
		}
		else if(sMD_Parameter.revolveMode > 1)
		{//O����ת֧�ֵ��������
			return 1;
		}
		else if(sMD_Parameter.revolveMode  == 1 && sMD_Parameter.gasPort > OUTPUT_NUM)
		{//����˿�С�ڶ˿���
			return 1;
		}
		else if(sMD_Parameter.topSwitch > 1)
		{//���㿪��ֻ������״̬
			return 1;
		}
//		else if(sMD_Parameter.stackLayer < sMD_RunPara.curLayer)
//		{//�ѵ���������С�ڵ�ǰ����
//			return 1;
//		}
		else if(sMD_Parameter.goodheight == 0 && sMD_Parameter.stackLayer > 1)
		{//���ѵ���������1��ʱ����Ʒ�߶ȱ��������
			return 1;
		}
		else if(sMD_Parameter.loopLayer > LOOP_MAX)
		{//ѭ���������ܴ���LOOP_MAX(10)
			return 1;
		}
	}
	
	return 0;
}

/**************************************************************************************************
**  ��������  MD_GetCurLayerNum()
**	���������
**	���������
**	�������ܣ��õ��ı������
**	��ע��	  
**  ���ߣ�       
**  ��������:
***************************************************************************************************/
u8 MD_GetCurLayerNum(void)
{
	u8 curLayer = 0;
	u8 curNum = 0;
	
	if(sMD_Parameter.stackType == 0)
	{//ʾ��
		if(sMD_Parameter.loopLayer < LOOP_MAX)
		{//ѭ������С��10��ʱ
			if(sMD_Parameter.topSwitch == 1 && sMD_RunPara.curLayer == sMD_Parameter.stackLayer)
			{//���㹦�ܿ��������һ�㰴��10��ʾ�̵�ȡ��
				curLayer = LOOP_MAX - 1;
			}
			else
			{
				curLayer = (sMD_RunPara.curLayer - 1) % sMD_Parameter.loopLayer;
			}
		}
		else if(sMD_Parameter.loopLayer == LOOP_MAX)
		{//ѭ����������10��ʱĬ�Ϲرն��㹦��
			curLayer = (sMD_RunPara.curLayer - 1) % sMD_Parameter.loopLayer;
		}
		curNum = sMD_Parameter.layerNum[curLayer];
	}
	else if(sMD_Parameter.stackType == 1)
	{//����ֻ��ʾ�̵�һ��ʾ�̵��������
		curNum = sMD_Parameter.horNum*sMD_Parameter.verNum;
	}
	return curNum;
}

/**************************************************************************************************
**  ��������  MD_ReadCurPoint()
**	���������pointType 0���� 1�ȴ���
**	�����������
**	�������ܣ���ʾ�̵ĵ�ǰ������
**	��ע��	  
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
u8 MD_ReadCurPoint(ST_MDPostion *sPostion, u8 pointType)
{
	u8 ret = 0;
	u8 IIC_Parameter[80] = {0};
	u32 readStartAddr = 0;
	u16 i = 0, k = 0;
	u8 curLayer = 0;
	ST_MDPostion postion[3] = {0};
	
	if(sMD_Parameter.stackType == 0)
	{//ʾ�̣�������Ϊ��һ�㣬���������ʽ��������ʽ���
		if(sMD_Parameter.loopLayer < LOOP_MAX)
		{//ѭ������С��10��ʱ
			if(sMD_Parameter.topSwitch == 1 && sMD_RunPara.curLayer == sMD_Parameter.stackLayer)
			{//���㹦�ܿ��������һ�㰴��10��ʾ�̵�ȡ��
				curLayer = LOOP_MAX - 1;
			}
			else
			{
				curLayer = (sMD_RunPara.curLayer - 1) % sMD_Parameter.loopLayer;
			}
		}
		else if(sMD_Parameter.loopLayer == LOOP_MAX)
		{//ѭ����������10��ʱĬ�Ϲرն��㹦��
			curLayer = (sMD_RunPara.curLayer - 1) % sMD_Parameter.loopLayer;
		}
		
		/*�����ȡ*/
		readStartAddr = P_MD_POINT_HEAD	+ P_MD_GOOD_LEN * (sMD_CurMDCode - 1) + P_MD_POINT_LEN * MD_POINT_NUM * curLayer + P_MD_POINT_LEN * (sMD_RunPara.curNum-1);
		W25QXX_Read(IIC_Parameter, readStartAddr, P_MD_POINT_LEN);
		for(k=0; k<Axis_Num; k++)
		{
			sPostion->point[k] = IIC_Parameter[0 + k * 8] + (((u32)IIC_Parameter[1 + k * 8]) << 8) + (((u32)IIC_Parameter[2 + k * 8]) << 16) + (((u32)IIC_Parameter[3 + k * 8]) << 24);
			sPostion->waitPoint[k] = IIC_Parameter[4 + k * 8] + (((u32)IIC_Parameter[5 + k * 8]) << 8) + (((u32)IIC_Parameter[6 + k * 8]) << 16) + (((u32)IIC_Parameter[7 + k * 8]) << 24);
		}
		
		/*Z��߶�ȡ��һ�㡢��һ����ĸ߶���Ϊ����*/
		readStartAddr = P_MD_POINT_HEAD	+ P_MD_GOOD_LEN * (sMD_CurMDCode - 1);
		W25QXX_Read(IIC_Parameter, readStartAddr, P_MD_POINT_LEN);
		k = Z_Axsis;
		sPostion->point[k] = IIC_Parameter[0 + k * 8] + (((u32)IIC_Parameter[1 + k * 8]) << 8) + (((u32)IIC_Parameter[2 + k * 8]) << 16) + (((u32)IIC_Parameter[3 + k * 8]) << 24);
		sPostion->waitPoint[k] = IIC_Parameter[4 + k * 8] + (((u32)IIC_Parameter[5 + k * 8]) << 8) + (((u32)IIC_Parameter[6 + k * 8]) << 16) + (((u32)IIC_Parameter[7 + k * 8]) << 24);
		
		/*����Z��߶�*/
		sPostion->point[Z_Axsis] = sPostion->point[Z_Axsis] - sMD_Parameter.goodheight * (sMD_RunPara.curLayer - 1);
		sPostion->waitPoint[Z_Axsis] = sPostion->waitPoint[Z_Axsis] - sMD_Parameter.goodheight * (sMD_RunPara.curLayer - 1);
	}
	else if(sMD_Parameter.stackType == 1)
	{//����������Ϊ��һ�㣬���������ʽ��������ʽ��⣬������һ����Ϊ��ʾ�̵����
		/*�����ȡ��ֻ��Ҫ��ȡ��һ���������*/
		for(i=0; i<3; i++)
		{
			readStartAddr = P_MD_POINT_HEAD	+ P_MD_GOOD_LEN * (sMD_CurMDCode - 1) + P_MD_POINT_LEN*i;
			W25QXX_Read(IIC_Parameter, readStartAddr, P_MD_POINT_LEN);
			for(k=0; k<Axis_Num; k++)
			{
				postion[i].point[k] = IIC_Parameter[0 + k * 8] + (((u32)IIC_Parameter[1 + k * 8]) << 8) + (((u32)IIC_Parameter[2 + k * 8]) << 16) + (((u32)IIC_Parameter[3 + k * 8]) << 24);
				postion[i].waitPoint[k] = IIC_Parameter[4 + k * 8] + (((u32)IIC_Parameter[5 + k * 8]) << 8) + (((u32)IIC_Parameter[6 + k * 8]) << 16) + (((u32)IIC_Parameter[7 + k * 8]) << 24);
			}
		}
		
		if(sCartesian_Para.carCoordSwitch == 1)
		{//ʹ�õѿ�������ϵ��������λ����
			ret = MD_MatrixPoint_Calculate(postion, sPostion, pointType);
			if(ret)
			{
				return 1;
			}
		}
		else
		{//ʹ��ֱ������ϵ������������
			MD_TrussMatrix_Calculate(postion[0], postion[1], postion[2], sPostion);
		}
		
		/*����Z��߶�*//*Z��߶�ȡ��һ�㡢��һ����ĸ߶���Ϊ����*/
		sPostion->point[Z_Axsis] = postion[0].point[Z_Axsis] - sMD_Parameter.goodheight * (sMD_RunPara.curLayer - 1);
		sPostion->waitPoint[Z_Axsis] = postion[0].waitPoint[Z_Axsis] - sMD_Parameter.goodheight * (sMD_RunPara.curLayer - 1);
	}
	
	if(sCartesian_Para.carCoordSwitch == 1 && sMD_GoodOffset_flag == 1)
	{//ʹ�õѿ�������ϵ��������ƫ��
		if(pointType == 0)
		{
			ret = MD_GoodOffset_Calculate(sPostion->point);
			if(ret)
			{
				return 1;
			}
		}
		else
		{
			ret = MD_GoodOffset_Calculate(sPostion->waitPoint);
			if(ret)
			{
				return 1;
			}
		}
	}
	else
	{//ʹ�����������ƫ��
		for(k=0; k<Axis_Num; k++)
		{
			sPostion->point[k] = sPostion->point[k] + sMD_Parameter.goodOffset[k];
			sPostion->waitPoint[k] = sPostion->waitPoint[k] + sMD_Parameter.goodOffset[k];
		}
	}
	
	/*ȷ��Z��߶Ȳ���С��0*/
	if(sPostion->point[Z_Axsis] < 0)
	{
		sPostion->point[Z_Axsis] = 0;
	}
	if(sPostion->waitPoint[Z_Axsis] < 0)
	{
		sPostion->waitPoint[Z_Axsis] = 0;
	}

	/*�����������ֵת��Ϊ����*/
	for(k=0; k<Axis_Num; k++)
	{
		if(k == O_Axsis && sMD_Parameter.revolveMode == 1)
		{//O��Ϊ����ʱ��ֱ����ԭʼֵ
			/*�����O��ֵ��Ϊ��ǰ���׵�״̬ 0��λ ��1��λ*/
			readStartAddr = P_MD_POINT_HEAD	+ P_MD_GOOD_LEN * (sMD_CurMDCode - 1) + P_MD_POINT_LEN * MD_POINT_NUM * curLayer + P_MD_POINT_LEN * (sMD_RunPara.curNum-1);
			W25QXX_Read(IIC_Parameter, readStartAddr, P_MD_POINT_LEN);
			sPostion->point[O_Axsis] = IIC_Parameter[0 + O_Axsis * 8] + (((u32)IIC_Parameter[1 + O_Axsis * 8]) << 8) + (((u32)IIC_Parameter[2 + O_Axsis * 8]) << 16) + (((u32)IIC_Parameter[3 + O_Axsis * 8]) << 24);
			sPostion->waitPoint[O_Axsis] = IIC_Parameter[4 + O_Axsis * 8] + (((u32)IIC_Parameter[5 + O_Axsis * 8]) << 8) + (((u32)IIC_Parameter[6 + O_Axsis * 8]) << 16) + (((u32)IIC_Parameter[7 + O_Axsis * 8]) << 24);
		}
		else
		{
			sPostion->point[k] = sPostion->point[k] * Step_Coefficient[k] / 100 + MINROBOTPOSITION;
			sPostion->waitPoint[k] = sPostion->waitPoint[k] * Step_Coefficient[k] / 100 + MINROBOTPOSITION;
		}
	}
	
	return 0;
}

/**************************************************************************************************
**  ��������  MD_ReadTeachOnePoint()
**	���������0ֻд��ּ𵱳���͵�ǰ������Ĵ洢�� 1ֻд��MD�˶��������ĵ�ǰ��Ʒ����ǰ�㡢��ǰ�����
**	�����������
**	�������ܣ���ʾ�̵ĵ�ǰ������
**	��ע��	  
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
void MD_WriteCurRunPara(u8 type)
{
	if(type == 0)
	{//ֻд��ּ𵱳���͵�ǰ������Ĵ洢��
		sMD_FlashCurLayer[sMD_RunPara.curGood - 1] = sMD_RunPara.curLayer;
		sMD_FlashCurNum[sMD_RunPara.curGood - 1] = sMD_RunPara.curNum;
	}
	else if(type == 1)
	{//ֻд��MD�˶��������ĵ�ǰ��Ʒ����ǰ�㡢��ǰ�����
		sMD_FlashCurLayer[sMD_RunPara.curGood - 1] = sMD_RunPara.curLayer;
		sMD_FlashCurNum[sMD_RunPara.curGood - 1] = sMD_RunPara.curNum;
	}
}

/**************************************************************************************************
**  ��������  MD_ContinueStackCount()
**	���������
**	���������
**	�������ܣ�����ģʽ-������
**	��ע��	  
**  ���ߣ�		
**  ��������:
***************************************************************************************************/
u8 MD_ContinueStackCount(s32 type, s32 calSymbol, s32 value)
{
	u8 ret = 0;
	u16 i = 0;
	
	/*����У��*/
	if(value == 0)
	{
		return 1;
	}
	
	ret = MD_ReadParData();
	if(ret == 1)
	{
		return 1;
	}
	
	if(sMD_Parameter.property == 0)
	{//��⣬ֻҪʵ����Ʒ=����Ʒ+������+
		if(type == V_MDGOOD)
		{//���-��Ʒ
			if(calSymbol == V_ADD)
			{//���-��Ʒ-�ӣ���ʵ���Զ���Ʒ˳���л�
				for(i=0; i<(value % sMD_RunPara.totalGood); i++)
				{
					sMD_RunPara.curGood++;
					if(sMD_RunPara.curGood >= sMD_RunPara.startGood + sMD_RunPara.totalGood)
					{
						sMD_RunPara.curGood = sMD_RunPara.startGood;
					}
				}
			}
			else if(calSymbol == V_MINUS)
			{//���-��Ʒ-��
				return 1;
			}
			else if(calSymbol == V_UEQUAL)
			{//���-��Ʒ-����
				if(sMD_RunPara.curGood != value && value >= sMD_RunPara.startGood && value < sMD_RunPara.startGood + sMD_RunPara.totalGood)
				{
					sMD_RunPara.curGood = value;
				}
				else if(sMD_RunPara.curGood == value)
				{
					return 0;
				}
				else
				{//��Ʒ�Ŵ���
					return 1;
				}
			}
			
			ret = MD_ReadParData();
			if(ret == 1)
			{
				return 1;
			}
			sMD_RunPara.curLayer = 1;
			sMD_RunPara.curNum = 1;
			MD_WriteCurRunPara(1);
		}
		else if(type == V_NUMBER)
		{//���-����
			if(calSymbol == V_ADD)
			{//���-����-�ӣ���ʵ���Զ���Ʒ˳���л�
				for(i=0; i<value; i++)
				{
					sMD_RunPara.curNum++;
					if(sMD_RunPara.curNum > MD_GetCurLayerNum())
					{
						sMD_RunPara.curLayer++;
						if(sMD_RunPara.curLayer > sMD_Parameter.stackLayer)
						{
							sMD_RunPara.curGood++;
							if(sMD_RunPara.curGood >= sMD_RunPara.startGood + sMD_RunPara.totalGood)
							{
								sMD_RunPara.curGood = sMD_RunPara.startGood;
							}
							
							ret = MD_ReadParData();
							if(ret == 1)
							{
								return 1;
							}
							sMD_RunPara.curLayer = 1;
						}
						sMD_RunPara.curNum = 1;
					}
				}
				
				MD_WriteCurRunPara(1);
			}
			else if(calSymbol == V_MINUS)
			{//���-����-��
				return 1;
			}
			else if(calSymbol == V_UEQUAL)
			{//���-����-����
				return 1;
			}
		}
		else if(type == V_LAYER_NUM)
		{//���-����
			return 1;
		}
		else if(type == V_FORMULATION)
		{//���-�䷽
			if(calSymbol == V_UEQUAL)
			{//���-�䷽-����
				if(value > 0 && value <= PF_IONUM)
				{//valueֵ��ʾ�����䷽ 1~PF_IONUM
					if(sMD_RunPara.curGood != PF_Parameter.pfGood[value - 1] && PF_Parameter.pfGood[value - 1] >= sMD_RunPara.startGood && PF_Parameter.pfGood[value - 1] < sMD_RunPara.startGood + sMD_RunPara.totalGood)
					{
						sMD_RunPara.curGood = PF_Parameter.pfGood[value - 1];
						ret = MD_ReadParData();
						if(ret == 1)
						{
							return 1;
						}
						sMD_RunPara.curLayer = 1;
						sMD_RunPara.curNum = 1;
						MD_WriteCurRunPara(1);
					}
					else if(sMD_RunPara.curGood == PF_Parameter.pfGood[value - 1])
					{
						return 0;
					}
					else
					{//��Ʒ�Ŵ���
						return 1;
					}
				}
				else
				{//��Ʒ�Ŵ���
					return 1;
				}
			}
			else
			{
				return 1;
			}
		}
	}
	else if(sMD_Parameter.property == 1)
	{//��磬ֻҪʵ����Ʒ=����Ʒ+������-
		if(type == V_MDGOOD)
		{//���-��Ʒ
			if(calSymbol == V_ADD)
			{//���-��Ʒ-�ӣ���ʵ���Զ���Ʒ˳���л�
				for(i=0; i<(value % sMD_RunPara.totalGood); i++)
				{
					sMD_RunPara.curGood++;
					if(sMD_RunPara.curGood >= sMD_RunPara.startGood + sMD_RunPara.totalGood)
					{
						sMD_RunPara.curGood = sMD_RunPara.startGood;
					}
				}
			}
			else if(calSymbol == V_MINUS)
			{//���-��Ʒ-��
				return 1;
			}
			else if(calSymbol == V_UEQUAL)
			{//���-��Ʒ-����
				if(sMD_RunPara.curGood != value && value >= sMD_RunPara.startGood && value < sMD_RunPara.startGood + sMD_RunPara.totalGood)
				{
					sMD_RunPara.curGood = value;
				}
				else if(sMD_RunPara.curGood == value)
				{
					return 0;
				}
				else
				{//��Ʒ�Ŵ���
					return 1;
				}
			}
			
			ret = MD_ReadParData();
			if(ret == 1)
			{
				return 1;
			}
			sMD_RunPara.curLayer = sMD_Parameter.stackLayer;
			sMD_RunPara.curNum = MD_GetCurLayerNum();
			MD_WriteCurRunPara(1);
		}
		else if(type == V_NUMBER)
		{//���-����
			if(calSymbol == V_ADD)
			{//���-����-��
				return 1;
			}
			else if(calSymbol == V_MINUS)
			{//���-����-������ʵ���Զ���Ʒ˳���л�
				for(i=0; i<value; i++)
				{
					sMD_RunPara.curNum--;
					if(sMD_RunPara.curNum == 0)
					{
						sMD_RunPara.curLayer--;
						if(sMD_RunPara.curLayer == 0)
						{
							sMD_RunPara.curGood++;
							if(sMD_RunPara.curGood >= sMD_RunPara.startGood + sMD_RunPara.totalGood)
							{
								sMD_RunPara.curGood = sMD_RunPara.startGood;
							}
							
							ret = MD_ReadParData();
							if(ret == 1)
							{
								return 1;
							}
							sMD_RunPara.curLayer = sMD_Parameter.stackLayer;
						}
						sMD_RunPara.curNum = MD_GetCurLayerNum();
					}
				}
				
				MD_WriteCurRunPara(1);
			}
			else if(calSymbol == V_UEQUAL)
			{//���-����-����
				return 1;
			}
		}
		else if(type == V_LAYER_NUM)
		{//���-����
			return 1;
		}
		else if(type == V_FORMULATION)
		{//���-�䷽
			if(calSymbol == V_UEQUAL)
			{//���-�䷽-����
				if(value > 0 && value <= PF_IONUM)
				{//valueֵ��ʾ�����䷽ 1~PF_IONUM
					if(sMD_RunPara.curGood != PF_Parameter.pfGood[value - 1] && PF_Parameter.pfGood[value - 1] >= sMD_RunPara.startGood && PF_Parameter.pfGood[value - 1] < sMD_RunPara.startGood + sMD_RunPara.totalGood)
					{
						sMD_RunPara.curGood = PF_Parameter.pfGood[value - 1];
						ret = MD_ReadParData();
						if(ret == 1)
						{
							return 1;
						}
						sMD_RunPara.curLayer = sMD_Parameter.stackLayer;
						sMD_RunPara.curNum = MD_GetCurLayerNum();
						MD_WriteCurRunPara(1);
					}
					else if(sMD_RunPara.curGood == PF_Parameter.pfGood[value - 1])
					{
						return 0;
					}
					else
					{//��Ʒ�Ŵ���
						return 1;
					}
				}
				else
				{//��Ʒ�Ŵ���
					return 1;
				}
			}
			else
			{
				return 1;
			}
		}
	}
	else
	{
		return 1;
	}
	
	return 0;
}

/**************************************************************************************************
**  ��������  MD_SortStackCount()
**	���������
**	���������
**	�������ܣ��ּ�ģʽ-������
**	��ע��	  
**  ���ߣ�		
**  ��������:
***************************************************************************************************/
u8 MD_SortStackCount(s32 type, s32 calSymbol, s32 value)
{
	u8 ret = 0;
	u16 i = 0;
	
	/*����У��*/
	if(value == 0)
	{
		return 1;
	}
	
	ret = MD_ReadParData();
	if(ret == 1)
	{
		return 1;
	}
	
	if(sMD_Parameter.property == 0)
	{//��⣬ֻҪʵ����Ʒ=����Ʒ+������+������=������=
		if(type == V_MDGOOD)
		{//���-��Ʒ
			if(calSymbol == V_ADD)
			{//���-��Ʒ-�ӣ���ʵ���Զ���Ʒ˳���л�
				for(i=0; i<(value % sMD_RunPara.totalGood); i++)
				{
					sMD_RunPara.curGood++;
					if(sMD_RunPara.curGood >= sMD_RunPara.startGood + sMD_RunPara.totalGood)
					{
						sMD_RunPara.curGood = sMD_RunPara.startGood;
					}
				}
			}
			else if(calSymbol == V_MINUS)
			{//���-��Ʒ-��
				return 1;
			}
			else if(calSymbol == V_UEQUAL)
			{//���-��Ʒ-����
				if(sMD_RunPara.curGood != value && value >= sMD_RunPara.startGood && value < sMD_RunPara.startGood + sMD_RunPara.totalGood)
				{
					sMD_RunPara.curGood = value;
				}
				else if(sMD_RunPara.curGood == value)
				{
					return 0;
				}
				else
				{//��Ʒ�Ŵ���
					return 1;
				}
			}
			/*�ּ�ģʽ��ֻ�л���Ʒ�������ò�����͸�������*/
			ret = MD_ReadParData();
			if(ret == 1)
			{
				return 1;
			}
			MD_WriteCurRunPara(1);
		}
		else if(type == V_NUMBER)
		{//���-������ֻҪ���ı���Ĳ�͸���
			if(calSymbol == V_ADD)
			{//���-����-��
				for(i=0; i<value; i++)
				{
					sMD_RunPara.curNum++;
					if(sMD_RunPara.curNum > MD_GetCurLayerNum())
					{
						sMD_RunPara.curLayer++;
						if(sMD_RunPara.curLayer > sMD_Parameter.stackLayer)
						{
							sMD_RunPara.curLayer = 1;
						}
						sMD_RunPara.curNum = 1;
					}
				}
				
				MD_WriteCurRunPara(0);
			}
			else if(calSymbol == V_MINUS)
			{//���-����-��
				return 1;
			}
			else if(calSymbol == V_UEQUAL)
			{//���-����-����
				if(value <= MD_GetCurLayerNum())
				{
					sMD_RunPara.curNum = value;
					MD_WriteCurRunPara(0);
				}
				else
				{//��Ʒ�Ŵ���
					return 1;
				}
			}
		}
		else if(type == V_LAYER_NUM)
		{//���-����
			if(calSymbol == V_ADD)
			{//���-����-��
				return 1;
			}
			else if(calSymbol == V_MINUS)
			{//���-����-��
				return 1;
			}
			else if(calSymbol == V_UEQUAL)
			{//���-����-����
				if(value <= sMD_Parameter.stackLayer)
				{
					sMD_RunPara.curNum = value;
					MD_WriteCurRunPara(0);
				}
				else
				{//��Ʒ�Ŵ���
					return 1;
				}
			}
		}
		else if(type == V_FORMULATION)
		{//���-�䷽
			if(calSymbol == V_UEQUAL)
			{//���-�䷽-����
				if(value > 0 && value <= PF_IONUM)
				{//valueֵ��ʾ�����䷽ 1~PF_IONUM
					if(sMD_RunPara.curGood != PF_Parameter.pfGood[value - 1] && PF_Parameter.pfGood[value - 1] >= sMD_RunPara.startGood && PF_Parameter.pfGood[value - 1] < sMD_RunPara.startGood + sMD_RunPara.totalGood)
					{
						sMD_RunPara.curGood = PF_Parameter.pfGood[value - 1];
						/*�ּ�ģʽ��ֻ�л���Ʒ�������ò�����͸�������*/
						ret = MD_ReadParData();
						if(ret == 1)
						{
							return 1;
						}
						MD_WriteCurRunPara(1);
					}
					else if(sMD_RunPara.curGood == PF_Parameter.pfGood[value - 1])
					{
						return 0;
					}
					else
					{//��Ʒ�Ŵ���
						return 1;
					}
				}
				else
				{//��Ʒ�Ŵ���
					return 1;
				}
			}
			else
			{
				return 1;
			}
		}
	}
	else if(sMD_Parameter.property == 1)
	{//��磬ֻҪʵ����Ʒ=����Ʒ+������-������=������=
		if(type == V_MDGOOD)
		{//���-��Ʒ
			if(calSymbol == V_ADD)
			{//���-��Ʒ-�ӣ���ʵ���Զ���Ʒ˳���л�
				for(i=0; i<(value % sMD_RunPara.totalGood); i++)
				{
					sMD_RunPara.curGood++;
					if(sMD_RunPara.curGood >= sMD_RunPara.startGood + sMD_RunPara.totalGood)
					{
						sMD_RunPara.curGood = sMD_RunPara.startGood;
					}
				}
			}
			else if(calSymbol == V_MINUS)
			{//���-��Ʒ-��
				return 1;
			}
			else if(calSymbol == V_UEQUAL)
			{//���-��Ʒ-����
				if(sMD_RunPara.curGood != value && value >= sMD_RunPara.startGood && value < sMD_RunPara.startGood + sMD_RunPara.totalGood)
				{
					sMD_RunPara.curGood = value;
				}
				else if(sMD_RunPara.curGood == value)
				{
					return 0;
				}
				else
				{//��Ʒ�Ŵ���
					return 1;
				}
			}
			
			ret = MD_ReadParData();
			if(ret == 1)
			{
				return 1;
			}
			MD_WriteCurRunPara(1);
		}
		else if(type == V_NUMBER)
		{//���-������ֻҪ���ı���Ĳ�͸���
			if(calSymbol == V_ADD)
			{//���-����-��
				return 1;
			}
			else if(calSymbol == V_MINUS)
			{//���-����-��
				for(i=0; i<value; i++)
				{
					sMD_RunPara.curNum--;
					if(sMD_RunPara.curNum == 0)
					{
						sMD_RunPara.curLayer--;
						if(sMD_RunPara.curLayer == 0)
						{
							sMD_RunPara.curLayer = sMD_Parameter.stackLayer;
						}
						sMD_RunPara.curNum = MD_GetCurLayerNum();
					}
				}
				
				MD_WriteCurRunPara(0);
			}
			else if(calSymbol == V_UEQUAL)
			{//���-����-����
				if(value <= MD_GetCurLayerNum())
				{
					sMD_RunPara.curNum = value;
					MD_WriteCurRunPara(0);
				}
				else
				{//��Ʒ�Ŵ���
					return 1;
				}
			}
		}
		else if(type == V_LAYER_NUM)
		{//���-����
			if(calSymbol == V_ADD)
			{//���-����-��
				return 1;
			}
			else if(calSymbol == V_MINUS)
			{//���-����-��
				return 1;
			}
			else if(calSymbol == V_UEQUAL)
			{//���-����-����
				if(value <= sMD_Parameter.stackLayer)
				{
					sMD_RunPara.curNum = value;
					MD_WriteCurRunPara(0);
				}
				else
				{//��Ʒ�Ŵ���
					return 1;
				}
			}
		}
		else if(type == V_FORMULATION)
		{//���-�䷽
			if(calSymbol == V_UEQUAL)
			{//���-�䷽-����
				if(value > 0 && value <= PF_IONUM)
				{//valueֵ��ʾ�����䷽ 1~PF_IONUM
					if(sMD_RunPara.curGood != PF_Parameter.pfGood[value - 1] && PF_Parameter.pfGood[value - 1] >= sMD_RunPara.startGood && PF_Parameter.pfGood[value - 1] < sMD_RunPara.startGood + sMD_RunPara.totalGood)
					{
						sMD_RunPara.curGood = PF_Parameter.pfGood[value - 1];
						/*�ּ�ģʽ��ֻ�л���Ʒ�������ò�����͸�������*/
						ret = MD_ReadParData();
						if(ret == 1)
						{
							return 1;
						}
						MD_WriteCurRunPara(1);
					}
					else if(sMD_RunPara.curGood == PF_Parameter.pfGood[value - 1])
					{
						return 0;
					}
					else
					{//��Ʒ�Ŵ���
						return 1;
					}
				}
				else
				{//��Ʒ�Ŵ���
					return 1;
				}
			}
			else
			{
				return 1;
			}
		}
	}
	else
	{
		return 1;
	}
	
	return 0;
}

/**************************************************************************************************
**  ��������  MD_StackCount()
**	���������
**	���������
**	�������ܣ�������
**	��ע��	  
**  ���ߣ�		
**  ��������:
***************************************************************************************************/
u8 MD_StackCount(s32 type, s32 calSymbol, s32 value)
{
	u8 ret = 0;
	
	if(sMD_RunPara.mdMethed == 0)
	{//����ģʽ
		ret = MD_ContinueStackCount(type, calSymbol, value);
	}
	else if(sMD_RunPara.mdMethed == 1)
	{//�ּ�ģʽ
		ret = MD_SortStackCount(type, calSymbol, value);
	}
	
	return ret;
}

/**************************************************************************************************
**  ��������  MD_LayerNumJudge()
**	���������
**	���������
**	�������ܣ������жϣ�0ʧ�� 1�ɹ�
**	��ע��	  
**  ���ߣ�		
**  ��������:
***************************************************************************************************/
u8 MD_LayerNumJudge(s32 type, s32 value)
{
	u8 ret = 0;
	
	if(type == V_ONLY_EQUAL && sMD_RunPara.curLayer == value)
	{//���-����
		ret = 1;
	}
	else if(type == V_MORE_THAN && sMD_RunPara.curLayer > value)
	{//���-����
		ret = 1;
	}
	else if(type == V_LESS_THAN && sMD_RunPara.curLayer < value)
	{//���-С��
		ret = 1;
	}
	
	return ret;
}

/**************************************************************************************************
**  ��������  MD_LayerFullJudge()
**	���������
**	���������
**	�������ܣ������жϣ�0ʧ�� 1�ɹ�
**	��ע��	  
**  ���ߣ�		
**  ��������:
***************************************************************************************************/
u8 MD_LayerFullJudge(s32 type, s32 value)
{
	u8 ret = 0;
	
	if((sMD_Parameter.property == 0 && sMD_RunPara.curNum == MD_GetCurLayerNum()) \
			|| (sMD_Parameter.property == 1 && sMD_RunPara.curNum == 1))
	{//����Ҳ����������������һ�������
		if(type == V_EQUAL && (sMD_RunPara.curLayer % value) == 0)
		{//���-����
			ret = 1;                                                                                                                                                                                                                                                                                  
		}
		else if(type == V_ONLY_EQUAL && sMD_RunPara.curLayer == value)
		{//���-����
			ret = 1;
		}
		else if(type == V_MORE_THAN && sMD_RunPara.curLayer > value)
		{//���-����
			ret = 1;
		}
		else if(type == V_LESS_THAN && sMD_RunPara.curLayer < value)
		{//���-С��
			ret = 1;
		}
	}
	
	return ret;
}

/**************************************************************************************************
**  ��������  MD_ReadGoodStaData()
**	���������
**	���������
**	�������ܣ���ȡ������Ϣ����
**	��ע��	  
**  ���ߣ�		
**  ��������:
***************************************************************************************************/
u8 MD_ReadGoodStaData(u8 goodNum, u8 *stackLayer, u8 *lastLayerNum)
{
	u8 IIC_Parameter[80] = {0};
	u32 readStartAddr = 0;
	u16 i = 0;
	u8 layer = 0;
	u8 loopLayer = 0;
	u8 layerNum[LOOP_MAX] = {0};
	
	/*��������ȡ*/
	readStartAddr = P_MD_PARA_HEAD	+ P_MD_GOOD_LEN * (sMD_CurMDCode - 1);
	W25QXX_Read(IIC_Parameter, readStartAddr, P_MD_PARA_LEN);
	layer = IIC_Parameter[21];
	loopLayer = IIC_Parameter[22];
	for(i=0; i<LOOP_MAX; i++)
	{//��ȡ10���ÿ�����
		layerNum[i] = IIC_Parameter[23 + i];
		if(layerNum[i] > MD_POINT_NUM)
		{
			layerNum[i] = MD_POINT_NUM;
		}
	}
	
	if(loopLayer < LOOP_MAX)
	{//ѭ������С��10��ʱ
		if(sMD_Parameter.topSwitch == 1)
		{
			*lastLayerNum = layerNum[LOOP_MAX - 1];
		}
		else
		{
			*lastLayerNum = layerNum[(layer - 1) % loopLayer];
		}
	}
	else if(sMD_Parameter.loopLayer == LOOP_MAX)
	{//ѭ����������10��ʱĬ�Ϲرն��㹦��
		*lastLayerNum = layerNum[(layer - 1) % loopLayer];
	}
	
	*stackLayer = layer;
	
	return 0;
}

/**************************************************************************************************
**  ��������  MD_StackFullJudge()
**	���������
**	���������
**	�������ܣ������жϣ�0ʧ�� 1�ɹ�
**	��ע��	  
**  ���ߣ�		
**  ��������:
***************************************************************************************************/
u8 MD_StackFullJudge(s32 type, s32 value)
{
	u8 ret = 0;
	u8 stackLayer = 0;
	u8 loopLayer = 0;
	
	if(type == V_AUTO || (type == V_ONLY_EQUAL && sMD_RunPara.curGood == value))//����ʵ����������޸�type�Ƚ�ֵ
	{//�Զ�ģʽ�������Ʒ���ж�
		if((sMD_Parameter.property == 0 && sMD_RunPara.curLayer == sMD_Parameter.stackLayer && sMD_RunPara.curNum == MD_GetCurLayerNum()) \
				|| (sMD_Parameter.property == 1 && sMD_RunPara.curLayer == 1 && sMD_RunPara.curNum == 1))
		{//����������������������
			ret = 1;
		}
	}
	else if(type == V_FORMULATION && sMD_RunPara.curGood == PF_Parameter.pfGood[value - 1])
	{//�䷽ģʽ�ж�
		MD_ReadGoodStaData(PF_Parameter.pfGood[value - 1], &stackLayer, &loopLayer);
		if((sMD_Parameter.property == 0 && sMD_RunPara.curLayer == stackLayer && sMD_RunPara.curNum == loopLayer) \
				|| (sMD_Parameter.property == 1 && sMD_RunPara.curLayer == 1 && sMD_RunPara.curNum == 1))
		{//����������������������
			ret = 1;
		}
	}
	
	return ret;
}

/**************************************************************************************************
**  ��������  MD_GoodNumJudge()
**	���������
**	���������
**	�������ܣ���Ʒ���жϣ�0ʧ�� 1�ɹ�
**	��ע��	  
**  ���ߣ�    
**  ��������:
***************************************************************************************************/
u8 MD_GoodNumJudge(s32 type, s32 value)
{
	u8 ret = 0;
	
	if(type == V_ONLY_EQUAL && sMD_RunPara.curGood == value)
	{//�Զ�ģʽ
		ret = 1;
	}
	
	return ret;
}


/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team ******** END OF FILE ************************/
