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
**  函数名：  FindHandcoor()
**	输入参数：关节2的角度theta2，单位rad
**	输出参数：机器人手系，1右手系 0左手系
**	函数功能：根据theta2计算scara机器人当前手系
**	备注：	  
**				右手系：theta2范围(0,Π)U(-2Π,-Π)
**				左手系：theta2范围(-Π,0)U(Π,2Π)
**				奇异位置：theta2取值{-2Π,-Π,0,Π,2Π}
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
**  函数名：  FindFlagJ1()
**	输入参数：关节的角度theta1，单位rad
**	输出参数：scara机器人当前J1关节标志位
**	函数功能：计算scara机器人当前J1关节标志位
**	备注：	  
**				FlagJ1=0：theta1范围[-Π,Π];否则，FlagJ1=1
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
**  函数名：  FindFlagJ2()
**	输入参数：关节的角度theta2，单位rad
**	输出参数：scara机器人当前J2关节标志位
**	函数功能：计算scara机器人当前J2关节标志位
**	备注：	  
**				FlagJ2=0：theta2范围[-Π,Π];否则，FlagJ2=1
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
**  函数名：  ScaraForwardKinematics()
**	输入参数：大臂长(mm),小臂长(mm),丝杆螺距(mm),机器人关节角度(rad),机器人关节位置(mm或rad)
**	输出参数：scara机器人运动学正解
**	函数功能：计算scara机器人末端位置(mm或rad)
**	备注：	  
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
**  函数名：  ScaraInverseKinematics()
**	输入参数：大臂长(mm),小臂长(mm),丝杆螺距(mm),机器人关节位置(mm或rad),
**						机器人当前手系, 机器人当前J1关节标志位, 机器人当前J2关节标志位, 需要得到的机器人关节角度(rad)
**	输出参数：scara机器人运动学逆解
**	函数功能：计算scara机器人末端位置(mm或rad)
**	备注：	  
***************************************************************************************************/
u8 ScaraInverseKinematics(float L1, float L2, float screw, ST_SCARA_JOINT_POS jointPos, u8 handcoor, u8 flagJ1, u8 flagJ2, ST_SCARA_JOINT_ANGLE *jointAngle)
{
	u8 ret = 0;
	float c2 = 0.0f;
	float s2 = 0.0f;
	float calErr = 0.0001f;					//允许计算误差
	float temp = 0.0f;
	
	/*若末端在(x,y)坐标系中，C2必在[-1,1]里，但由于计算精度，c2绝对值可能稍微大于1*/
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
	{//左手系
		jointAngle->theta[1] = atan2(-sqrt(temp), c2);
	}
	else
	{//右手系
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
**  函数名：  MD_ReadParData()
**	输入参数：
**	输出参数：
**	函数功能：物料偏移计算
**	备注：	  
**  作者：		
**  开发日期:
***************************************************************************************************/
u8 MD_GoodOffset_Calculate(s32 *point)
{
	u16 i = 0;
	u8 flagJ1 = 0;
	u8 flagJ2 = 0;
	u8 handcoor = 0;
	u8 ret = 0;
	ST_SCARA_JOINT_ANGLE sJointAngle = {0};		//机器人关节角度(rad)
	ST_SCARA_JOINT_POS sJointPos = {0};				//机器人关节位置(mm或rad)
	s32 offsetPostion[Axis_Num] = {0};				//轴坐标系-偏移坐标
	u8 handturn_flag = 0;											//位置不能到达，翻转手系再次计算，若还不能到达，则运行出错
	float screw = 0.0f;												//螺距
	s32 pointTemp[Axis_Num] = {0};						//轴坐标保存
	
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
	{//如果是旋转轴就需要传入丝杆螺距
		screw = (float)sCartesian_Para.pitchLength / 100;
	}
	else
	{
		screw = 0.0f;
	}
	
	ret = ScaraForwardKinematics((float)sCartesian_Para.length[0] / 100, (float)sCartesian_Para.length[1] / 100, screw, sJointAngle, &sJointPos);//正解
	if(ret)
	{
		return 1;
	}
	
	sJointPos.x = sJointPos.x + (float)sMD_Parameter.goodOffset[X_Axsis] / 100;
	sJointPos.y = sJointPos.y + (float)sMD_Parameter.goodOffset[L_Axsis] / 100;
	if(sCartesian_Para.axisType[Z_Axsis] == 1)
	{//如果是旋转轴就需要传入丝杆高度
		sJointPos.z = sJointPos.z + (float)sMD_Parameter.goodOffset[Z_Axsis] / 100;
	}
	else
	{
		sJointPos.z = 0.0f;
	}
	sJointPos.c = sJointPos.c + (float)sMD_Parameter.goodOffset[O_Axsis] / 100;
	ret = ScaraInverseKinematics((float)sCartesian_Para.length[0] / 100, (float)sCartesian_Para.length[1] / 100, screw, sJointPos, handcoor, flagJ1, flagJ2, &sJointAngle);//逆解
	if(ret)
	{
		return 1;
	}
	
	for(i=0; i<Axis_Num; i++)
	{
		if(sCartesian_Para.axisType[i] == 1)
		{//轴类型为旋转轴时，即为笛卡尔坐标系中的轴需要转换，其他轴只需要进行直线偏移
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
			{//计算出的坐标不在安全区内，需要更换手系重新计算
				handturn_flag = 1;
				break;
			}
		}
		else
		{//其他轴只需要进行直线偏移
			point[i] = pointTemp[i] + sMD_Parameter.goodOffset[i];
		}
	}
	
	if(handturn_flag)
	{//计算得到的位置不在安全区内，翻转手系再次计算，若还不能到达，则运行出错
		if(handcoor == 0)
		{//切换手系
			handcoor = 1;
		}
		else
		{
			handcoor = 0;
		}
		
		//不考虑Z轴，screw=0
		ret = ScaraInverseKinematics((float)sCartesian_Para.length[0] / 100, (float)sCartesian_Para.length[1] / 100, 0, sJointPos, handcoor, flagJ1, flagJ2, &sJointAngle);//逆解
		if(ret)
		{
			return 1;
		}
		
		for(i=0; i<Axis_Num; i++)
		{
			if(sCartesian_Para.axisType[i] == 1)
			{//轴类型为旋转轴时，即为笛卡尔坐标系中的轴需要转换，其他轴只需要进行直线偏移
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
				{//计算出的坐标不在安全区内，报警
					return 1;
				}
			}
			else
			{//其他轴只需要进行直线偏移
				point[i] = pointTemp[i] + sMD_Parameter.goodOffset[i];
			}
		}
	}
	
	return 0;
}

/*************************************************************************
**  函数名：  MD_PointPostion_Calculate(u8 proNum)
**	输入参数：笛卡尔坐标系：postion1、postion2、postion3为矩阵料仓的三个点，postion_Cur为当前要计算的点
**	输出参数：无
**	函数功能：计算scara式当前码垛点的位置
**	备注：	  无
**  作者：    
**  开发日期：
**************************************************************************/
u8 MD_PointPostion_Calculate(ST_SCARA_JOINT_POS postion1, ST_SCARA_JOINT_POS postion2, ST_SCARA_JOINT_POS postion3, ST_SCARA_JOINT_POS *postion_Cur)
{
	u8 H_count=0, V_count=0;				//当前横向、纵向第几个
	float HD=0.0f, VD=0.0f;         //横向、纵向间距
	float DeltaH=0.0f, DeltaV=0.0f;	//横向、纵向补偿
//	float HDZ=0.0f, VDZ=0.0f;     //横、纵Z向间距
		
	if(sMD_Parameter.horNum == 0 || sMD_Parameter.verNum == 0 || sMD_Parameter.stackLayer == 0 || sMD_Parameter.horNum > 200 || sMD_Parameter.verNum > 200 || sMD_Parameter.stackLayer > 200)
	{
		return 1;
	} 
	if(sMD_Parameter.horNum*sMD_Parameter.verNum<sMD_RunPara.curNum || sMD_Parameter.stackLayer < sMD_RunPara.curLayer)
	{
		return 1;
	}
	
	//计算当前上料位置
	//不考虑Z轴
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
			HD = (postion2.x-postion1.x) / (sMD_Parameter.horNum-1);  		//取整
			DeltaV = (postion2.y-postion1.y)/(sMD_Parameter.horNum-1); 	//第二列的偏移
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
				DeltaH = (postion3.x-postion2.x)/(sMD_Parameter.verNum-1); 	//第二行的偏移
//				VDZ = (postion3.z-postion2.z)/(V_num-1);
		}
		
		//计算平均数
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
**  函数名：  MD_TrussMatrix_Calculate(u8 proNum)
**	输入参数：postion1、postion2、postion3为矩阵料仓的三个点，postion_Cur为当前要计算的点
**	输出参数：无
**	函数功能：计算桁架式当前码垛点的位置
**	备注：	  无
**  作者：    
**  开发日期：
**************************************************************************/
u8 MD_TrussMatrix_Calculate(ST_MDPostion postion1, ST_MDPostion postion2, ST_MDPostion postion3, ST_MDPostion *postion_Cur)
{
	u8 H_count=0, V_count=0;				//当前横向、纵向第几个
	float HD=0.0f, VD=0.0f;         //横向、纵向间距
	float DeltaH=0.0f, DeltaV=0.0f;	//横向、纵向补偿
//	float HDZ=0.0f, VDZ=0.0f;     //横、纵Z向间距
		
	if(sMD_Parameter.horNum == 0 || sMD_Parameter.verNum == 0 || sMD_Parameter.stackLayer == 0 || sMD_Parameter.horNum > 200 || sMD_Parameter.verNum > 200 || sMD_Parameter.stackLayer > 200)
	{
		return 1;
	} 
	if(sMD_Parameter.horNum*sMD_Parameter.verNum<sMD_RunPara.curNum || sMD_Parameter.stackLayer < sMD_RunPara.curLayer)
	{
		return 1;
	}
	
	//计算当前上料位置
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
		/************************码垛点**************************/
		if(sMD_Parameter.horNum == 1)
		{
			HD = 0.0f;
			DeltaV = 0.0f;
//			HDZ = 0.0f;
		}
		else
		{
			HD = (postion2.point[X_Axsis] - postion1.point[X_Axsis]) / (sMD_Parameter.horNum - 1);  		//取整
			DeltaV = (postion2.point[L_Axsis] - postion1.point[L_Axsis]) / (sMD_Parameter.horNum - 1); 	//第二列的偏移
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
				DeltaH = (postion3.point[X_Axsis]-postion2.point[X_Axsis]) / (sMD_Parameter.verNum - 1); 	//第二行的偏移
//				VDZ = (postion3.z-postion2.z)/(V_num-1);
		}
		
		//计算平均数
		H_count = (sMD_RunPara.curNum-1) / sMD_Parameter.horNum;//+1
		V_count = (sMD_RunPara.curNum-1) % sMD_Parameter.horNum;//+1
		postion_Cur->point[X_Axsis] = postion1.point[X_Axsis] + V_count * HD + H_count * DeltaH;
		postion_Cur->point[L_Axsis] = postion1.point[L_Axsis] + H_count*VD + V_count*DeltaV;
		postion_Cur->point[Z_Axsis] = postion1.point[Z_Axsis];// + V_count*HDZ + H_count*VDZ
		postion_Cur->point[O_Axsis] = postion1.point[O_Axsis];
		
		/************************等待点**************************/
		if(sMD_Parameter.horNum == 1)
		{
			HD = 0.0f;
			DeltaV = 0.0f;
//			HDZ = 0.0f;
		}
		else
		{
			HD = (postion2.waitPoint[X_Axsis] - postion1.waitPoint[X_Axsis]) / (sMD_Parameter.horNum - 1);  		//取整
			DeltaV = (postion2.waitPoint[L_Axsis] - postion1.waitPoint[L_Axsis]) / (sMD_Parameter.horNum - 1); 	//第二列的偏移
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
				DeltaH = (postion3.waitPoint[X_Axsis]-postion2.waitPoint[X_Axsis]) / (sMD_Parameter.verNum - 1); 	//第二行的偏移
//				VDZ = (postion3.z-postion2.z)/(V_num-1);
		}
		
		//计算平均数
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
**  函数名：  MD_ReadParData()
**	输入参数：pointType 0码垛点 1等待点
**	输出参数：
**	函数功能：矩阵垛型点位计算
**	备注：	  
**  作者：		
**  开发日期:
***************************************************************************************************/
u8 MD_MatrixPoint_Calculate(ST_MDPostion *sPostion_Teach, ST_MDPostion *sPostion_Cur, u8 pointType)
{
	u16 i = 0, k = 0;
	u8 flagJ1 = 0;
	u8 flagJ2 = 0;
	u8 handcoor = 0;
	float screw = 0.0f;
	u8 ret = 0;
	ST_SCARA_JOINT_ANGLE sJointAngle = {0};		//机器人关节角度(rad)
	ST_SCARA_JOINT_POS sJointPos[3] = {0};		//机器人关节位置(mm或rad)-示教的三个点
	ST_SCARA_JOINT_POS sJointPos_Cur = {0};		//机器人关节位置(mm或rad)-计算当前点
	s32 offsetPostion[Axis_Num] = {0};				//轴坐标系-偏移坐标
	u8 handturn_flag = 0;											//位置不能到达，翻转手系再次计算，若还不能到达，则运行出错
	
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
		{//如果是旋转轴就需要传入丝杆螺距
			screw = (float)sCartesian_Para.pitchLength / 100;
		}
		else
		{
			screw = 0.0f;
		}
		ret = ScaraForwardKinematics((float)sCartesian_Para.length[0]/100, (float)sCartesian_Para.length[1]/100, screw, sJointAngle, &sJointPos[k]);//正解：位置11、2、3的笛卡尔坐标
		if(ret)
		{
			return 1;
		}
	}
	
	//计算偏移量
	ret = MD_PointPostion_Calculate(sJointPos[0], sJointPos[1], sJointPos[2], &sJointPos_Cur);
	if(ret)
	{
		return 1;
	}
	
	//不考虑Z轴，screw=0
	ret = ScaraInverseKinematics((float)sCartesian_Para.length[0]/100, (float)sCartesian_Para.length[1]/100, screw, sJointPos_Cur, handcoor, flagJ1, flagJ2, &sJointAngle);//逆解
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
	{//位置不能到达，翻转手系再次计算，若还不能到达，则运行出错
		if(handcoor == 0)
		{
			handcoor = 1;
		}
		else
		{
			handcoor = 0;
		}
		//不考虑Z轴，screw=0
		ret = ScaraInverseKinematics((float)sCartesian_Para.length[0]/100, (float)sCartesian_Para.length[1]/100, 0, sJointPos_Cur, handcoor, flagJ1, flagJ2, &sJointAngle);//逆解
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
**  函数名：  MD_ReadParData()
**	输入参数：
**	输出参数：
**	函数功能：读取码垛参数
**	备注：	  
**  作者：		
**  开发日期:
***************************************************************************************************/
u8 MD_ReadParData(void)
{
	u8 IIC_Parameter[80] = {0};
	u32 readStartAddr = 0;
	u16 i = 0;
	
	/*检测MD的运行参数*/
	if(sMD_RunPara.mdMethed > 1)
	{//只支持连续模式和分拣模式
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
	{//物品编号发生变化时，重新读取物品基本数据
		sMD_CurMDCode = sMD_RunPara.curGood;
		sMD_GoodOffset_flag = 0;
		
		if(sMD_RunPara.mdMethed == 1)
		{//分拣模式时，当前层和当前层个数需要从分拣存储区读取
			sMD_RunPara.curLayer = sMD_FlashCurLayer[sMD_CurMDCode - 1];
			sMD_RunPara.curNum = sMD_FlashCurNum[sMD_CurMDCode - 1];
		}

		/*码垛参数读取*/
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
		{//读取10层的每层个数
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
		
		/*检测码垛物品的基本参数*/
		if(sMD_Parameter.stackType > 1)
		{//支持示教、矩形的垛型
			return 1;
		}
		else if(sMD_Parameter.property > 1)
		{//支持码垛和拆垛
			return 1;
		}
		else if(sMD_Parameter.revolveMode > 1)
		{//O轴旋转支持电机和气缸
			return 1;
		}
		else if(sMD_Parameter.revolveMode  == 1 && sMD_Parameter.gasPort > OUTPUT_NUM)
		{//输出端口小于端口数
			return 1;
		}
		else if(sMD_Parameter.topSwitch > 1)
		{//顶层开关只有两种状态
			return 1;
		}
//		else if(sMD_Parameter.stackLayer < sMD_RunPara.curLayer)
//		{//堆叠层数不能小于当前层数
//			return 1;
//		}
		else if(sMD_Parameter.goodheight == 0 && sMD_Parameter.stackLayer > 1)
		{//当堆叠层数大于1的时候，物品高度必须大于零
			return 1;
		}
		else if(sMD_Parameter.loopLayer > LOOP_MAX)
		{//循环层数不能大于LOOP_MAX(10)
			return 1;
		}
	}
	
	return 0;
}

/**************************************************************************************************
**  函数名：  MD_GetCurLayerNum()
**	输入参数：
**	输出参数：
**	函数功能：得到的本层个数
**	备注：	  
**  作者：       
**  开发日期:
***************************************************************************************************/
u8 MD_GetCurLayerNum(void)
{
	u8 curLayer = 0;
	u8 curNum = 0;
	
	if(sMD_Parameter.stackType == 0)
	{//示教
		if(sMD_Parameter.loopLayer < LOOP_MAX)
		{//循环层数小于10层时
			if(sMD_Parameter.topSwitch == 1 && sMD_RunPara.curLayer == sMD_Parameter.stackLayer)
			{//顶层功能开启，最后一层按第10层示教点取点
				curLayer = LOOP_MAX - 1;
			}
			else
			{
				curLayer = (sMD_RunPara.curLayer - 1) % sMD_Parameter.loopLayer;
			}
		}
		else if(sMD_Parameter.loopLayer == LOOP_MAX)
		{//循环层数等于10层时默认关闭顶层功能
			curLayer = (sMD_RunPara.curLayer - 1) % sMD_Parameter.loopLayer;
		}
		curNum = sMD_Parameter.layerNum[curLayer];
	}
	else if(sMD_Parameter.stackType == 1)
	{//矩阵，只需示教第一层示教点的三个点
		curNum = sMD_Parameter.horNum*sMD_Parameter.verNum;
	}
	return curNum;
}

/**************************************************************************************************
**  函数名：  MD_ReadCurPoint()
**	输入参数：pointType 0码垛点 1等待点
**	输出参数：无
**	函数功能：读示教的当前点坐标
**	备注：	  
**  作者：    
**  开发日期：
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
	{//示教，最下面为第一层，可用于桁架式码垛和立柱式码垛
		if(sMD_Parameter.loopLayer < LOOP_MAX)
		{//循环层数小于10层时
			if(sMD_Parameter.topSwitch == 1 && sMD_RunPara.curLayer == sMD_Parameter.stackLayer)
			{//顶层功能开启，最后一层按第10层示教点取点
				curLayer = LOOP_MAX - 1;
			}
			else
			{
				curLayer = (sMD_RunPara.curLayer - 1) % sMD_Parameter.loopLayer;
			}
		}
		else if(sMD_Parameter.loopLayer == LOOP_MAX)
		{//循环层数等于10层时默认关闭顶层功能
			curLayer = (sMD_RunPara.curLayer - 1) % sMD_Parameter.loopLayer;
		}
		
		/*码垛点读取*/
		readStartAddr = P_MD_POINT_HEAD	+ P_MD_GOOD_LEN * (sMD_CurMDCode - 1) + P_MD_POINT_LEN * MD_POINT_NUM * curLayer + P_MD_POINT_LEN * (sMD_RunPara.curNum-1);
		W25QXX_Read(IIC_Parameter, readStartAddr, P_MD_POINT_LEN);
		for(k=0; k<Axis_Num; k++)
		{
			sPostion->point[k] = IIC_Parameter[0 + k * 8] + (((u32)IIC_Parameter[1 + k * 8]) << 8) + (((u32)IIC_Parameter[2 + k * 8]) << 16) + (((u32)IIC_Parameter[3 + k * 8]) << 24);
			sPostion->waitPoint[k] = IIC_Parameter[4 + k * 8] + (((u32)IIC_Parameter[5 + k * 8]) << 8) + (((u32)IIC_Parameter[6 + k * 8]) << 16) + (((u32)IIC_Parameter[7 + k * 8]) << 24);
		}
		
		/*Z轴高度取第一层、第一个点的高度做为基数*/
		readStartAddr = P_MD_POINT_HEAD	+ P_MD_GOOD_LEN * (sMD_CurMDCode - 1);
		W25QXX_Read(IIC_Parameter, readStartAddr, P_MD_POINT_LEN);
		k = Z_Axsis;
		sPostion->point[k] = IIC_Parameter[0 + k * 8] + (((u32)IIC_Parameter[1 + k * 8]) << 8) + (((u32)IIC_Parameter[2 + k * 8]) << 16) + (((u32)IIC_Parameter[3 + k * 8]) << 24);
		sPostion->waitPoint[k] = IIC_Parameter[4 + k * 8] + (((u32)IIC_Parameter[5 + k * 8]) << 8) + (((u32)IIC_Parameter[6 + k * 8]) << 16) + (((u32)IIC_Parameter[7 + k * 8]) << 24);
		
		/*计算Z轴高度*/
		sPostion->point[Z_Axsis] = sPostion->point[Z_Axsis] - sMD_Parameter.goodheight * (sMD_RunPara.curLayer - 1);
		sPostion->waitPoint[Z_Axsis] = sPostion->waitPoint[Z_Axsis] - sMD_Parameter.goodheight * (sMD_RunPara.curLayer - 1);
	}
	else if(sMD_Parameter.stackType == 1)
	{//矩阵，最下面为第一层，可用于桁架式码垛和立柱式码垛，最下面一层做为层示教点计算
		/*码垛点读取，只需要读取第一层的三个点*/
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
		{//使用笛卡尔坐标系计算矩阵点位坐标
			ret = MD_MatrixPoint_Calculate(postion, sPostion, pointType);
			if(ret)
			{
				return 1;
			}
		}
		else
		{//使用直角坐标系计算矩阵点坐标
			MD_TrussMatrix_Calculate(postion[0], postion[1], postion[2], sPostion);
		}
		
		/*计算Z轴高度*//*Z轴高度取第一层、第一个点的高度做为基数*/
		sPostion->point[Z_Axsis] = postion[0].point[Z_Axsis] - sMD_Parameter.goodheight * (sMD_RunPara.curLayer - 1);
		sPostion->waitPoint[Z_Axsis] = postion[0].waitPoint[Z_Axsis] - sMD_Parameter.goodheight * (sMD_RunPara.curLayer - 1);
	}
	
	if(sCartesian_Para.carCoordSwitch == 1 && sMD_GoodOffset_flag == 1)
	{//使用笛卡尔坐标系计算物料偏移
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
	{//使用轴坐标进行偏移
		for(k=0; k<Axis_Num; k++)
		{
			sPostion->point[k] = sPostion->point[k] + sMD_Parameter.goodOffset[k];
			sPostion->waitPoint[k] = sPostion->waitPoint[k] + sMD_Parameter.goodOffset[k];
		}
	}
	
	/*确保Z轴高度不能小于0*/
	if(sPostion->point[Z_Axsis] < 0)
	{
		sPostion->point[Z_Axsis] = 0;
	}
	if(sPostion->waitPoint[Z_Axsis] < 0)
	{
		sPostion->waitPoint[Z_Axsis] = 0;
	}

	/*将距离的坐标值转化为脉冲*/
	for(k=0; k<Axis_Num; k++)
	{
		if(k == O_Axsis && sMD_Parameter.revolveMode == 1)
		{//O轴为气缸时，直接是原始值
			/*码垛点的O轴值做为当前气缸的状态 0复位 非1置位*/
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
**  函数名：  MD_ReadTeachOnePoint()
**	输入参数：0只写入分拣当场层和当前层个数的存储区 1只写入MD运动参数区的当前物品、当前层、当前层个数
**	输出参数：无
**	函数功能：读示教的当前点坐标
**	备注：	  
**  作者：    
**  开发日期：
***************************************************************************************************/
void MD_WriteCurRunPara(u8 type)
{
	if(type == 0)
	{//只写入分拣当场层和当前层个数的存储区
		sMD_FlashCurLayer[sMD_RunPara.curGood - 1] = sMD_RunPara.curLayer;
		sMD_FlashCurNum[sMD_RunPara.curGood - 1] = sMD_RunPara.curNum;
	}
	else if(type == 1)
	{//只写入MD运动参数区的当前物品、当前层、当前层个数
		sMD_FlashCurLayer[sMD_RunPara.curGood - 1] = sMD_RunPara.curLayer;
		sMD_FlashCurNum[sMD_RunPara.curGood - 1] = sMD_RunPara.curNum;
	}
}

/**************************************************************************************************
**  函数名：  MD_ContinueStackCount()
**	输入参数：
**	输出参数：
**	函数功能：连续模式-码垛计数
**	备注：	  
**  作者：		
**  开发日期:
***************************************************************************************************/
u8 MD_ContinueStackCount(s32 type, s32 calSymbol, s32 value)
{
	u8 ret = 0;
	u16 i = 0;
	
	/*参数校验*/
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
	{//码垛，只要实现物品=、物品+、个数+
		if(type == V_MDGOOD)
		{//码垛-物品
			if(calSymbol == V_ADD)
			{//码垛-物品-加，可实现自动物品顺序切换
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
			{//码垛-物品-减
				return 1;
			}
			else if(calSymbol == V_UEQUAL)
			{//码垛-物品-等于
				if(sMD_RunPara.curGood != value && value >= sMD_RunPara.startGood && value < sMD_RunPara.startGood + sMD_RunPara.totalGood)
				{
					sMD_RunPara.curGood = value;
				}
				else if(sMD_RunPara.curGood == value)
				{
					return 0;
				}
				else
				{//物品号错误
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
		{//码垛-个数
			if(calSymbol == V_ADD)
			{//码垛-个数-加，可实现自动物品顺序切换
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
			{//码垛-个数-减
				return 1;
			}
			else if(calSymbol == V_UEQUAL)
			{//码垛-个数-等于
				return 1;
			}
		}
		else if(type == V_LAYER_NUM)
		{//码垛-层数
			return 1;
		}
		else if(type == V_FORMULATION)
		{//码垛-配方
			if(calSymbol == V_UEQUAL)
			{//码垛-配方-等于
				if(value > 0 && value <= PF_IONUM)
				{//value值表示几号配方 1~PF_IONUM
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
					{//物品号错误
						return 1;
					}
				}
				else
				{//物品号错误
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
	{//拆剁，只要实现物品=、物品+、个数-
		if(type == V_MDGOOD)
		{//拆剁-物品
			if(calSymbol == V_ADD)
			{//拆垛-物品-加，可实现自动物品顺序切换
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
			{//拆垛-物品-减
				return 1;
			}
			else if(calSymbol == V_UEQUAL)
			{//拆垛-物品-等于
				if(sMD_RunPara.curGood != value && value >= sMD_RunPara.startGood && value < sMD_RunPara.startGood + sMD_RunPara.totalGood)
				{
					sMD_RunPara.curGood = value;
				}
				else if(sMD_RunPara.curGood == value)
				{
					return 0;
				}
				else
				{//物品号错误
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
		{//拆垛-个数
			if(calSymbol == V_ADD)
			{//拆垛-个数-加
				return 1;
			}
			else if(calSymbol == V_MINUS)
			{//拆垛-个数-减，可实现自动物品顺序切换
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
			{//拆垛-个数-等于
				return 1;
			}
		}
		else if(type == V_LAYER_NUM)
		{//拆垛-层数
			return 1;
		}
		else if(type == V_FORMULATION)
		{//拆垛-配方
			if(calSymbol == V_UEQUAL)
			{//拆垛-配方-等于
				if(value > 0 && value <= PF_IONUM)
				{//value值表示几号配方 1~PF_IONUM
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
					{//物品号错误
						return 1;
					}
				}
				else
				{//物品号错误
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
**  函数名：  MD_SortStackCount()
**	输入参数：
**	输出参数：
**	函数功能：分拣模式-码垛计数
**	备注：	  
**  作者：		
**  开发日期:
***************************************************************************************************/
u8 MD_SortStackCount(s32 type, s32 calSymbol, s32 value)
{
	u8 ret = 0;
	u16 i = 0;
	
	/*参数校验*/
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
	{//码垛，只要实现物品=、物品+、个数+、个数=、层数=
		if(type == V_MDGOOD)
		{//码垛-物品
			if(calSymbol == V_ADD)
			{//码垛-物品-加，可实现自动物品顺序切换
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
			{//码垛-物品-减
				return 1;
			}
			else if(calSymbol == V_UEQUAL)
			{//码垛-物品-等于
				if(sMD_RunPara.curGood != value && value >= sMD_RunPara.startGood && value < sMD_RunPara.startGood + sMD_RunPara.totalGood)
				{
					sMD_RunPara.curGood = value;
				}
				else if(sMD_RunPara.curGood == value)
				{
					return 0;
				}
				else
				{//物品号错误
					return 1;
				}
			}
			/*分拣模式，只切换物品，不重置层计数和个数计数*/
			ret = MD_ReadParData();
			if(ret == 1)
			{
				return 1;
			}
			MD_WriteCurRunPara(1);
		}
		else if(type == V_NUMBER)
		{//码垛-个数，只要关心本垛的层和个数
			if(calSymbol == V_ADD)
			{//码垛-个数-加
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
			{//码垛-个数-减
				return 1;
			}
			else if(calSymbol == V_UEQUAL)
			{//码垛-个数-等于
				if(value <= MD_GetCurLayerNum())
				{
					sMD_RunPara.curNum = value;
					MD_WriteCurRunPara(0);
				}
				else
				{//物品号错误
					return 1;
				}
			}
		}
		else if(type == V_LAYER_NUM)
		{//码垛-层数
			if(calSymbol == V_ADD)
			{//码垛-层数-加
				return 1;
			}
			else if(calSymbol == V_MINUS)
			{//码垛-层数-减
				return 1;
			}
			else if(calSymbol == V_UEQUAL)
			{//码垛-层数-等于
				if(value <= sMD_Parameter.stackLayer)
				{
					sMD_RunPara.curNum = value;
					MD_WriteCurRunPara(0);
				}
				else
				{//物品号错误
					return 1;
				}
			}
		}
		else if(type == V_FORMULATION)
		{//码垛-配方
			if(calSymbol == V_UEQUAL)
			{//码垛-配方-等于
				if(value > 0 && value <= PF_IONUM)
				{//value值表示几号配方 1~PF_IONUM
					if(sMD_RunPara.curGood != PF_Parameter.pfGood[value - 1] && PF_Parameter.pfGood[value - 1] >= sMD_RunPara.startGood && PF_Parameter.pfGood[value - 1] < sMD_RunPara.startGood + sMD_RunPara.totalGood)
					{
						sMD_RunPara.curGood = PF_Parameter.pfGood[value - 1];
						/*分拣模式，只切换物品，不重置层计数和个数计数*/
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
					{//物品号错误
						return 1;
					}
				}
				else
				{//物品号错误
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
	{//拆剁，只要实现物品=、物品+、个数-、个数=、层数=
		if(type == V_MDGOOD)
		{//拆剁-物品
			if(calSymbol == V_ADD)
			{//拆垛-物品-加，可实现自动物品顺序切换
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
			{//拆垛-物品-减
				return 1;
			}
			else if(calSymbol == V_UEQUAL)
			{//拆垛-物品-等于
				if(sMD_RunPara.curGood != value && value >= sMD_RunPara.startGood && value < sMD_RunPara.startGood + sMD_RunPara.totalGood)
				{
					sMD_RunPara.curGood = value;
				}
				else if(sMD_RunPara.curGood == value)
				{
					return 0;
				}
				else
				{//物品号错误
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
		{//拆垛-个数，只要关心本垛的层和个数
			if(calSymbol == V_ADD)
			{//拆垛-个数-加
				return 1;
			}
			else if(calSymbol == V_MINUS)
			{//拆垛-个数-减
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
			{//拆垛-个数-等于
				if(value <= MD_GetCurLayerNum())
				{
					sMD_RunPara.curNum = value;
					MD_WriteCurRunPara(0);
				}
				else
				{//物品号错误
					return 1;
				}
			}
		}
		else if(type == V_LAYER_NUM)
		{//拆垛-层数
			if(calSymbol == V_ADD)
			{//码垛-层数-加
				return 1;
			}
			else if(calSymbol == V_MINUS)
			{//码垛-层数-减
				return 1;
			}
			else if(calSymbol == V_UEQUAL)
			{//码垛-层数-等于
				if(value <= sMD_Parameter.stackLayer)
				{
					sMD_RunPara.curNum = value;
					MD_WriteCurRunPara(0);
				}
				else
				{//物品号错误
					return 1;
				}
			}
		}
		else if(type == V_FORMULATION)
		{//拆垛-配方
			if(calSymbol == V_UEQUAL)
			{//拆垛-配方-等于
				if(value > 0 && value <= PF_IONUM)
				{//value值表示几号配方 1~PF_IONUM
					if(sMD_RunPara.curGood != PF_Parameter.pfGood[value - 1] && PF_Parameter.pfGood[value - 1] >= sMD_RunPara.startGood && PF_Parameter.pfGood[value - 1] < sMD_RunPara.startGood + sMD_RunPara.totalGood)
					{
						sMD_RunPara.curGood = PF_Parameter.pfGood[value - 1];
						/*分拣模式，只切换物品，不重置层计数和个数计数*/
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
					{//物品号错误
						return 1;
					}
				}
				else
				{//物品号错误
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
**  函数名：  MD_StackCount()
**	输入参数：
**	输出参数：
**	函数功能：码垛计数
**	备注：	  
**  作者：		
**  开发日期:
***************************************************************************************************/
u8 MD_StackCount(s32 type, s32 calSymbol, s32 value)
{
	u8 ret = 0;
	
	if(sMD_RunPara.mdMethed == 0)
	{//连续模式
		ret = MD_ContinueStackCount(type, calSymbol, value);
	}
	else if(sMD_RunPara.mdMethed == 1)
	{//分拣模式
		ret = MD_SortStackCount(type, calSymbol, value);
	}
	
	return ret;
}

/**************************************************************************************************
**  函数名：  MD_LayerNumJudge()
**	输入参数：
**	输出参数：
**	函数功能：层数判断，0失败 1成功
**	备注：	  
**  作者：		
**  开发日期:
***************************************************************************************************/
u8 MD_LayerNumJudge(s32 type, s32 value)
{
	u8 ret = 0;
	
	if(type == V_ONLY_EQUAL && sMD_RunPara.curLayer == value)
	{//码垛-等于
		ret = 1;
	}
	else if(type == V_MORE_THAN && sMD_RunPara.curLayer > value)
	{//码垛-大于
		ret = 1;
	}
	else if(type == V_LESS_THAN && sMD_RunPara.curLayer < value)
	{//码垛-小于
		ret = 1;
	}
	
	return ret;
}

/**************************************************************************************************
**  函数名：  MD_LayerFullJudge()
**	输入参数：
**	输出参数：
**	函数功能：层满判断，0失败 1成功
**	备注：	  
**  作者：		
**  开发日期:
***************************************************************************************************/
u8 MD_LayerFullJudge(s32 type, s32 value)
{
	u8 ret = 0;
	
	if((sMD_Parameter.property == 0 && sMD_RunPara.curNum == MD_GetCurLayerNum()) \
			|| (sMD_Parameter.property == 1 && sMD_RunPara.curNum == 1))
	{//码垛且层个数已满，或拆垛且一层已完成
		if(type == V_EQUAL && (sMD_RunPara.curLayer % value) == 0)
		{//码垛-倍数
			ret = 1;                                                                                                                                                                                                                                                                                  
		}
		else if(type == V_ONLY_EQUAL && sMD_RunPara.curLayer == value)
		{//码垛-等于
			ret = 1;
		}
		else if(type == V_MORE_THAN && sMD_RunPara.curLayer > value)
		{//码垛-大于
			ret = 1;
		}
		else if(type == V_LESS_THAN && sMD_RunPara.curLayer < value)
		{//码垛-小于
			ret = 1;
		}
	}
	
	return ret;
}

/**************************************************************************************************
**  函数名：  MD_ReadGoodStaData()
**	输入参数：
**	输出参数：
**	函数功能：读取垛型信息参数
**	备注：	  
**  作者：		
**  开发日期:
***************************************************************************************************/
u8 MD_ReadGoodStaData(u8 goodNum, u8 *stackLayer, u8 *lastLayerNum)
{
	u8 IIC_Parameter[80] = {0};
	u32 readStartAddr = 0;
	u16 i = 0;
	u8 layer = 0;
	u8 loopLayer = 0;
	u8 layerNum[LOOP_MAX] = {0};
	
	/*码垛参数读取*/
	readStartAddr = P_MD_PARA_HEAD	+ P_MD_GOOD_LEN * (sMD_CurMDCode - 1);
	W25QXX_Read(IIC_Parameter, readStartAddr, P_MD_PARA_LEN);
	layer = IIC_Parameter[21];
	loopLayer = IIC_Parameter[22];
	for(i=0; i<LOOP_MAX; i++)
	{//读取10层的每层个数
		layerNum[i] = IIC_Parameter[23 + i];
		if(layerNum[i] > MD_POINT_NUM)
		{
			layerNum[i] = MD_POINT_NUM;
		}
	}
	
	if(loopLayer < LOOP_MAX)
	{//循环层数小于10层时
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
	{//循环层数等于10层时默认关闭顶层功能
		*lastLayerNum = layerNum[(layer - 1) % loopLayer];
	}
	
	*stackLayer = layer;
	
	return 0;
}

/**************************************************************************************************
**  函数名：  MD_StackFullJudge()
**	输入参数：
**	输出参数：
**	函数功能：垛满判断，0失败 1成功
**	备注：	  
**  作者：		
**  开发日期:
***************************************************************************************************/
u8 MD_StackFullJudge(s32 type, s32 value)
{
	u8 ret = 0;
	u8 stackLayer = 0;
	u8 loopLayer = 0;
	
	if(type == V_AUTO || (type == V_ONLY_EQUAL && sMD_RunPara.curGood == value))//根据实际命令情况修改type比较值
	{//自动模式或根据物品号判断
		if((sMD_Parameter.property == 0 && sMD_RunPara.curLayer == sMD_Parameter.stackLayer && sMD_RunPara.curNum == MD_GetCurLayerNum()) \
				|| (sMD_Parameter.property == 1 && sMD_RunPara.curLayer == 1 && sMD_RunPara.curNum == 1))
		{//码垛且已满，或拆垛且已完成
			ret = 1;
		}
	}
	else if(type == V_FORMULATION && sMD_RunPara.curGood == PF_Parameter.pfGood[value - 1])
	{//配方模式判断
		MD_ReadGoodStaData(PF_Parameter.pfGood[value - 1], &stackLayer, &loopLayer);
		if((sMD_Parameter.property == 0 && sMD_RunPara.curLayer == stackLayer && sMD_RunPara.curNum == loopLayer) \
				|| (sMD_Parameter.property == 1 && sMD_RunPara.curLayer == 1 && sMD_RunPara.curNum == 1))
		{//码垛且已满，或拆垛且已完成
			ret = 1;
		}
	}
	
	return ret;
}

/**************************************************************************************************
**  函数名：  MD_GoodNumJudge()
**	输入参数：
**	输出参数：
**	函数功能：物品号判断，0失败 1成功
**	备注：	  
**  作者：    
**  开发日期:
***************************************************************************************************/
u8 MD_GoodNumJudge(s32 type, s32 value)
{
	u8 ret = 0;
	
	if(type == V_ONLY_EQUAL && sMD_RunPara.curGood == value)
	{//自动模式
		ret = 1;
	}
	
	return ret;
}


/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team ******** END OF FILE ************************/
