/*************** (C) COPYRIGHT 2015 Kingrobot manipulator Team ************************
* File Name          : Auto_2.c
* Author             : Fenkella Zhou
* Version            : V1.0.0
* Date               : 21/10/2015
* Description        : 自动运行控制文件
***************************************************************************************/
#include "stm32f4xx.h"
#include "Auto_2.h"
#include "ActionOperate.h"
#include "StatusControl.h"
#include "Auto.h"
#include "in.h"
#include "out.h"
#include "w25qxx.h"
#include "Error.h"
#include "SpeedControl.h"
#include "SignalWatch.h"
#include "stmflash.h"

/***************************公共参数，供上位机查询与访问**************************************/
u8  g_AutoStatue = AUTO_WAITE;		//自动运行状态
u16 g_Auto_PresentLine = 0;				//自动运行行号

u8  g_SubProgram_Step_Run[SAVEPROGRAMNUM_SUB] = {FALSE};    //是否运行子程序，在主程序中置位
u8  g_SubProgram_Start[SAVEPROGRAMNUM_SUB] = {FALSE};				//子程序开始
u8  g_SubProgram_Finish[SAVEPROGRAMNUM_SUB] = {FALSE};			//子程序结束
u8  g_SubProgram_ContralEnd[SAVEPROGRAMNUM_SUB] = {FALSE};	//控制子程序结束
u8  g_Read_SubProgram[SAVEPROGRAMNUM_SUB] = {FALSE};				//子程序读取完成
u16 g_SubProgram_PresentLine[SAVEPROGRAMNUM_SUB] = {0};			//子程序自动运行行号

u8  g_SubProgram_ActionRun_Step[SAVEPROGRAMNUM_SUB] = {0};				  //子程序
u32 g_SubProgram_ActionRun_Timer[SAVEPROGRAMNUM_SUB] = {0};					//子程序延时计数器
u32 g_SubProgram_ActionTimeOut_Time[SAVEPROGRAMNUM_SUB] = {3000}; 	//10ms为单位,30s没检测到确认信号，则认为超时

u8  g_Auto_Order_Start = FALSE;		//自动运行启动指令
u8  g_Auto_Order_Pause = FALSE;		//自动运行暂停指令
u8  g_Auto_Order_Stop  = FALSE;		//自动运行停止指令

//自行运行内部参数
u32 g_Auto_ActionRun_Timer = 0;
u8  g_Auto_ActionRun_Step = 0;
u8	g_Auto_ActionNcWait_Flag = 0;														//主程序输入检测超时检测标志位
u8 g_Auto_SubActionNcWait_Flag[SAVEPROGRAMNUM_SUB] = {0};		//子程序输入检测超时检测标志位
u32	g_Auto_ActionTimeOut_Time = 3000; 											//10ms为单位		 30s没检测到确认信号，则认为超时
u8 	g_Auto_ActionTimeOut_Flag = FALSE;											//动作检测超时
u8 	g_Auto_ActionError_Flag = FALSE;  											//自动运行进入到错误状态AUTO_ERROR
u8 	g_Auto_WorkFinished_Flag = FALSE; 											//当前工作任务已完成
u8 	g_Auto_CJWorkFinished_Flag = FALSE; 										//当前工作任务已完成
u8  MD_PositionErr_Flag = FALSE;														//码垛位置坐标计算异常

u8 g_Auto_Valid_Flag = 0;//有效时间计时标志位
u32 g_Auto_Valid_Timer = 0;//有效时间计时，出现有效信号开始计时，后一旦出现无效型号则复位计时，超过最大保持时间也复位计时

u8 g_SubAuto_Valid_Flag[SAVEPROGRAMNUM_SUB] = {0};//有效时间计时标志位
u32 g_SubAuto_Valid_Timer[SAVEPROGRAMNUM_SUB] = {0};//有效时间计时，出现有效信号开始计时，后一旦出现无效型号则复位计时，超过最大保持时间也复位计时

//jump功能参数
u16 m_JumpStepRunNum = {0}; 																		//主程序跳转到的行号
u16 m_JumpSubStepRunNum[SAVEPROGRAMNUM_SUB] = {0};															//子程序跳转到的行号

//while功能参数
u8  m_WhileNC = {0};																						//主程序while命令嵌套层数计数器
u8  m_WhileRunFlag[WHILE_NEST_MAX] = {0};											//主程序while命令运行标志
u16 m_WhileLineNum[WHILE_NEST_MAX] = {0};											//主程序while命令行号
//u16 m_WhileOverLineNum[WHILE_NEST_MAX] = {0};								//主程序while命令对应结束命令的行号
u16 m_WhileCycCounter[WHILE_NEST_MAX] = {0};										//主程序while命令循环次数
u8  m_WhileJudgeType[WHILE_NEST_MAX] = {0};										//主程序while命令条件类型
s32 m_WhileJudgePar1[WHILE_NEST_MAX] = {0};										//主程序while命令参数1
s32 m_WhileJudgePar2[WHILE_NEST_MAX] = {0};										//主程序while命令参数2
u8  m_WhileJudgeRes[WHILE_NEST_MAX] = {0};											//主程序while命令判断结果，0失败，1成功

u8  m_WhileSubNC[SAVEPROGRAMNUM_SUB] = {0};																			//子程序while命令嵌套层数计数器
u8  m_WhileSubRunFlag[SAVEPROGRAMNUM_SUB][WHILE_NEST_MAX] = {0};								//子程序while命令运行标志
u16 m_WhileSubLineNum[SAVEPROGRAMNUM_SUB][WHILE_NEST_MAX] = {0};								//子程序while命令行号
//u16 m_WhileOverSubLineNum[SAVEPROGRAMNUM_SUB][WHILE_NEST_MAX] = {0};						//子程序while命令对应结束命令的行号
u16 m_WhileSubCycCounter[SAVEPROGRAMNUM_SUB][WHILE_NEST_MAX] = {0};							//子程序while命令循环次数
u8  m_WhileSubJudgeType[SAVEPROGRAMNUM_SUB][WHILE_NEST_MAX] = {0};							//子程序while命令条件类型
s32 m_WhileSubJudgePar1[SAVEPROGRAMNUM_SUB][WHILE_NEST_MAX] = {0};							//子程序while命令参数1
s32 m_WhileSubJudgePar2[SAVEPROGRAMNUM_SUB][WHILE_NEST_MAX] = {0};							//子程序while命令参数2
u8  m_WhileSubJudgeRes[SAVEPROGRAMNUM_SUB][WHILE_NEST_MAX] = {0};								//子程序while命令判断结果，0失败，1成功

//if功能参数
u8  m_IfElseNC = {0};																					//主程序If-Else命令嵌套层数计数器
u8  m_IfElseJudgeType[IF_ELSE_NEST_MAX] = {0};									//主程序If-Else命令条件类型
s32 m_IfElseJudgePar1[IF_ELSE_NEST_MAX] = {0};									//主程序If-Else命令参数1
s32 m_IfElseJudgePar2[IF_ELSE_NEST_MAX] = {0};									//主程序If-Else命令参数2
u8  m_IfElseJudgeRes[IF_ELSE_NEST_MAX] = {0};									//主程序If-Else命令判断结果，0失败，1成功
u8  m_IfElseJudgeRunFlag[IF_ELSE_NEST_MAX] = {0};							//主程序If-Else命令组合执行标志，0未执行，1已执行

u8  m_IfElseSubNC[SAVEPROGRAMNUM_SUB] = {0};																		//子程序If-Else命令嵌套层数计数器
u8  m_IfElseSubJudgeType[SAVEPROGRAMNUM_SUB][IF_ELSE_NEST_MAX] = {0};						//子程序If-Else命令条件类型
s32 m_IfElseSubJudgePar1[SAVEPROGRAMNUM_SUB][IF_ELSE_NEST_MAX] = {0};						//子程序If-Else命令参数1
s32 m_IfElseSubJudgePar2[SAVEPROGRAMNUM_SUB][IF_ELSE_NEST_MAX] = {0};						//子程序If-Else命令参数2
u8  m_IfElseSubJudgeRes[SAVEPROGRAMNUM_SUB][IF_ELSE_NEST_MAX] = {0};						//子程序If-Else命令判断结果，0失败，1成功
u8  m_IfElseSubJudgeRunFlag[SAVEPROGRAMNUM_SUB][IF_ELSE_NEST_MAX] = {0};				//子程序If-Else命令组合执行标志，0未执行，1已执行

//搜索运动
u8 Flag_Falling_Edge = FALSE;																		//下降沿标志位
u8 Flag_Rising_Edge = FALSE;																		//上升沿标志位
u8 Flag_Falling_Edge_Sub = FALSE;																//子程序下降沿标志位
u8 Flag_Rising_Edge_Sub = FALSE;																//子程序上升沿标志位
u8 Flag_Keep_Move[Axis_Num + Ext_Axis_Num] = {FALSE};						//持续运动指令标志位

//输入检测
u8 Detect_Falling_Edge = FALSE;																	//下降沿标志位
u8 Detect_Rising_Edge = FALSE;																	//上升沿标志位
u8 Detect_Falling_Edge_Sub[SAVEPROGRAMNUM_SUB] = {FALSE};				//子程序下降沿标志位
u8 Detect_Rising_Edge_Sub[SAVEPROGRAMNUM_SUB] = {FALSE};				//子程序上升沿标志位

u8 Program_Origin_Axis = X_Axsis;																//机械回零指令轴
u8 Program_Axis_Origin_Flag[Axis_Num + Ext_Axis_Num] = {FALSE};	//机械回零指令标志位
u8 Program_Axis_Origin_Speed[Axis_Num + Ext_Axis_Num] = {FALSE};//机械回零指令速度

u16 Action_Step_List_Num=0;					  		//单次运行指令数
u16 Action_Step_Run_Num=0;					  		//已经运行的指令行数
u16 Action_Step_Confirm_Num=0;				  	//确认的指令行数
u8  g_Start_ActionRun_Step = 0;				  	//启动单次动作步骤
u8  g_Reset_ActionRun_Step = 0;				  	//启动单次动作步骤

u16 SubAction_Step_List_Num[SAVEPROGRAMNUM_SUB] = {0};					//子程序单次运行指令数
u16 SubAction_Step_Run_Num[SAVEPROGRAMNUM_SUB] = {0};					  //子程序已经运行的指令行数
u16 SubAction_Step_Confirm_Num[SAVEPROGRAMNUM_SUB] = {0};				//子程序确认的指令行数

void SubProgramStepControl(u8 subProNum);
void ActionStepControl(void);
void ActionOverOperate(void);
void AutoPauseOperate(void);
void AutoStopOperate(void);
void StartActionControl(void);

/**************************************************************************************************
**  函数名：  AutoModeControl()
**	输入参数：无
**	输出参数：无
**	函数功能：自动模式控制函数
**	备注：	  控制设备自动运行
**  作者：      
**  开发日期：
***************************************************************************************************/
void AutoModeControl(void)
{
	int i = 0;
	
	switch(g_AutoStatue)
	{
		case AUTO_WAITE://等待状态检测启动命令
			if(g_Auto_Order_Start == TRUE)
			{//接收到启动自动运行指令
				g_Auto_Order_Start = FALSE;		//复位标志位
				g_AutoStatue = AUTO_RUNNING;	//进入自动运行状态
				Program_RunTime = 0;
				for(i=0; i<USER_NUM; i++)
				{
					if(USER_Parameter.START_RESET[i] == TRUE)
					{
						USER_Parameter.CURR_Num[i] = USER_Parameter.INIT_Num[i];
					}
				}
				
				sMD_CurMDCode = 0;						//清零当前码垛参数存放的数据
			}
			break; 
		case AUTO_RUNNING://自动运行相关操作
			if(g_Auto_Order_Pause == TRUE)		
			{//重新启动自动运行指令
				g_Auto_Order_Pause = FALSE;		//复位标志位
				g_AutoStatue = AUTO_PAUSE;		//回到自动运行状态
				AutoPauseOperate();						//暂停运行相关操作		
				if(g_Program_Is_Debuging == TRUE)
				{
					g_AutoStatue = AUTO_RUNNING;
				}				
			}
			
			if(g_Auto_Order_Stop == TRUE)		
			{//自动运行停止指令
				g_Auto_Order_Stop = FALSE;		//复位标志位
				g_AutoStatue = AUTO_WAITE;		//回到自动等待状态
				AutoPauseOperate();						//暂停运行相关操作
				AutoStopOperate();						//停止运行相关操作
			}
			
			if(g_AutoStatue == AUTO_RUNNING)
			{//运行程序
				if(g_Program_Is_Debuging == FALSE)
				{
					g_Write_FlashFlag = TRUE;
				}
				ActionStepControl();//单行动作步骤控制（对Free_Program_Operate结构体进行控制）

				for(i=0; i<SAVEPROGRAMNUM_SUB; i++)
				{
					if(g_SubProgram_Step_Run[i] == TRUE)
					{//运行子程序
						SubProgramStepControl(i);
					}
				}
			}
			break;
		case AUTO_PAUSE://暂停转状态检测是否启动或停止
			if(g_Auto_Order_Start == TRUE)	//重新启动自动运行指令
			{
				g_Auto_Order_Start = FALSE;		//复位标志位
				g_AutoStatue = AUTO_RUNNING;	//回到自动运行状态
			}
			if(g_Auto_Order_Stop == TRUE)		//自动运行停止指令
			{
				g_Auto_Order_Stop = FALSE;		//复位标志位
				g_AutoStatue = AUTO_WAITE;		//回到自动等待状态
				AutoStopOperate();						//停止运行相关操作
			}
			break;
		case AUTO_ERROR://错误检测处理
			g_Auto_ActionError_Flag=TRUE;
			g_Auto_Order_Stop = TRUE;
			if(g_Auto_Order_Stop == TRUE)		//自动运行停止指令
			{
				g_Auto_Order_Stop = FALSE;		//复位标志位
				g_AutoStatue = AUTO_WAITE;		//回到自动等待状态
				AutoPauseOperate();
				AutoStopOperate();						//停止运行相关操作
			}
			break;
		default:
			break;
	}		
}

/**************************************************************************************************
**  函数名：  MainLogicComEndDeal()
**	输入参数：无
**	输出参数：无
**	函数功能：主程序逻辑命令完成后的处理
**	备注：	  控制每行程序单独动作  
**  开发日期：
***************************************************************************************************/
void MainLogicComEndDeal(SaveProgram *Program_Operate, u8 ActionLine)
{
	u16 i = 0;
	u8 while_Counter = 0;							//实现while嵌套
	u8 if_else_Counter = 0;						//实现if-else嵌套
	
	switch(Program_Operate->Program[ActionLine].Key)
	{
		case K_JUMP://跳转命令
						if(Program_Operate->Program[ActionLine].Value1 == V_JUMP_LABEL)
			{
				g_Auto_PresentLine = g_Auto_PresentLine+1;
				break;
			}
			if(ActionLine < m_JumpStepRunNum)
			{
				for(i=ActionLine+1; i<m_JumpStepRunNum; i++)
				{//向下跳转时搜索逻辑命令
					if(Program_Operate->Program[i].Key == K_WHILE)
					{
						m_WhileNC++;
						m_WhileLineNum[m_WhileNC - 1] = i;
						m_WhileRunFlag[m_WhileNC - 1] = 1;						//循环命令结束
					}
					else if(Program_Operate->Program[i].Key == K_CYCLEOVER)
					{
						m_WhileRunFlag[m_WhileNC - 1] = 0;						//循环命令结束
						m_WhileNC--;
					}
					
					if(Program_Operate->Program[i].Key == K_IF)
					{
						m_IfElseNC++;
						m_IfElseJudgeRunFlag[m_IfElseNC - 1] = 1;
					}
					else if(Program_Operate->Program[i].Key == K_ELSE)
					{
						m_IfElseJudgeRunFlag[m_IfElseNC - 1] = 1;
					}
					else if(Program_Operate->Program[i].Key == K_OVER)
					{
						m_IfElseJudgeRunFlag[m_IfElseNC - 1] = 0;
						m_IfElseNC--;
					}
				}
				
				if(Program_Operate->Program[m_JumpStepRunNum].Key == K_ELSE)
				{//如果跳转到else需要清除已执行标志
					m_IfElseJudgeRunFlag[m_IfElseNC - 1] = 0;
				}
			}
			else if(m_JumpStepRunNum > 0 && ActionLine > m_JumpStepRunNum)
			{
				for(i=ActionLine-1; i>m_JumpStepRunNum; i--)
				{//向上跳转时搜索逻辑命令
					if(Program_Operate->Program[i].Key == K_WHILE)
					{
						m_WhileRunFlag[m_WhileNC - 1] = 0;						//循环命令结束
						m_WhileNC--;
					}
					else if(Program_Operate->Program[i].Key == K_CYCLEOVER)
					{
						m_WhileNC++;
						m_WhileRunFlag[m_WhileNC - 1] = 1;						//循环命令结束
					}
					
					if(Program_Operate->Program[i].Key == K_IF)
					{
						m_IfElseJudgeRunFlag[m_IfElseNC - 1] = 0;
						m_IfElseNC--;
					}
					else if(Program_Operate->Program[i].Key == K_ELSE)
					{
						m_IfElseJudgeRunFlag[m_IfElseNC - 1] = 0;
					}
					else if(Program_Operate->Program[i].Key == K_OVER)
					{
						m_IfElseNC++;
						m_IfElseJudgeRunFlag[m_IfElseNC - 1] = 1;
					}
				}
				
				if(Program_Operate->Program[m_JumpStepRunNum].Key == K_ELSE)
				{//如果跳转到else需要清除已执行标志
					m_IfElseJudgeRunFlag[m_IfElseNC - 1] = 0;
				}
			}
			g_Auto_PresentLine = m_JumpStepRunNum;
			break;
		case K_WHILE://循环命令
			if(m_WhileJudgeRes[m_WhileNC - 1] == 1)
			{//判断成功
				m_WhileJudgeRes[m_WhileNC - 1] = 0;
				g_Auto_PresentLine = g_Auto_PresentLine + 1;
			}
			else
			{
				m_WhileRunFlag[m_WhileNC - 1] = 0;						//循环命令结束
				for(i=ActionLine+1; i<Program_Operate->Num; i++)
				{//搜索当前while对应的循环结束命令
					if(Program_Operate->Program[i].Key == K_WHILE)
					{
						while_Counter++;
					}
					else if(Program_Operate->Program[i].Key == K_CYCLEOVER)
					{
						if(while_Counter > 0)
						{
							while_Counter--;
						}
						else
						{
							m_WhileNC--;
							g_Auto_PresentLine = i + 1;
							break;
						}
					}
				}
				if(i == Program_Operate->Num)
				{//未收索到循环结束命令则指向最后一行程序
					g_Auto_PresentLine = Program_Operate->Num - 1;
					m_WhileNC--;
				}
			}
			break;
		case K_CYCLEOVER://循环结束命令
			g_Auto_PresentLine = m_WhileLineNum[m_WhileNC - 1];
			m_WhileNC--;
			break;
		case K_IF://IF判断条件命令
		case K_ELSE://ELSE判断条件命令
			if(m_IfElseJudgeRes[m_IfElseNC - 1] == 1 && m_IfElseJudgeRunFlag[m_IfElseNC - 1] == 0)
			{
				m_IfElseJudgeRunFlag[m_IfElseNC - 1] = 1;
				g_Auto_PresentLine = g_Auto_PresentLine + 1;
			}
			else
			{
				m_IfElseJudgeRes[m_IfElseNC - 1] = 0;
				for(i=ActionLine+1; i<Program_Operate->Num; i++)
				{//搜索当前IF-ELSE对应的命令
					if(Program_Operate->Program[i].Key == K_IF)
					{//如果IF-ELSE命令还未执行过且先读到新的ELSE，那么就执行新行的判断
						if_else_Counter++;
					}
					else if(if_else_Counter == 0 && Program_Operate->Program[i].Key == K_ELSE && m_IfElseJudgeRunFlag[m_IfElseNC - 1] == 0)
					{//如果IF-ELSE命令还未执行过且先读到新的ELSE，那么就执行新行的判断
						g_Auto_PresentLine = i;
						break;
					}
					else if(Program_Operate->Program[i].Key == K_OVER)
					{//如果先读取到判断结束符，那么执行下一行
						if(if_else_Counter > 0)
						{
							if_else_Counter--;
						}
						else
						{
							m_IfElseNC--;
							g_Auto_PresentLine = i + 1;
							break;
						}
					}
				}
				if(i == Program_Operate->Num)
				{//未收索到判断结束命令则指向最后一行程序
					m_IfElseNC = 0;
					g_Auto_PresentLine = Program_Operate->Num - 1;
				}
			}
			break;
		case K_OVER://判断命令结束
			m_IfElseJudgeRunFlag[m_IfElseNC - 1] = 0;
			m_IfElseNC--;
			g_Auto_PresentLine = g_Auto_PresentLine + 1;
			break;
		case K_SPECIAL://特殊指令处理
			if(Program_Operate->Program[ActionLine].Value1 == V_SUSPEND)
			{
				g_Auto_Order_Pause = TRUE;						//暂停运行相关操作
			}
			else if(Program_Operate->Program[ActionLine].Value1 == V_STOP)
			{
				g_Auto_Order_Stop = TRUE;							//停止运行相关操作
			}
			g_Auto_PresentLine = g_Auto_PresentLine + 1;
			break;
		default://其他命令对运行行号的操作
			g_Auto_PresentLine = g_Auto_PresentLine + Action_Step_Run_Num;
			break;
	}
}

/**************************************************************************************************
**  函数名：  SubLogicComEndDeal()
**	输入参数：无
**	输出参数：无
**	函数功能：子程序逻辑命令完成后的处理
**	备注：	  控制每行程序单独动作  
**  开发日期：
***************************************************************************************************/
void SubLogicComEndDeal(u8 subProNum, u8 ActionLine)
{
		u16 i = 0;
	u8 while_Counter = 0;							//实现while嵌套
	u8 if_else_Counter = 0;						//实现if-else嵌套
	
	switch(SubProgram_Operate[subProNum].Program[ActionLine].Key)
	{
		case K_JUMP://跳转命令
			if(SubProgram_Operate[subProNum].Program[ActionLine].Value1 == V_JUMP_LABEL)
			{
				g_SubProgram_PresentLine[subProNum] = g_SubProgram_PresentLine[subProNum]+1;
				break;
			}
			if(ActionLine < m_JumpSubStepRunNum[subProNum])
			{
				for(i=ActionLine+1; i<m_JumpSubStepRunNum[subProNum]; i++)
				{//向下跳转时搜索逻辑命令
					if(SubProgram_Operate[subProNum].Program[i].Key == K_WHILE)
					{
						m_WhileSubNC[subProNum]++;
						m_WhileSubLineNum[subProNum][m_WhileSubNC[subProNum] - 1] = i;
						m_WhileSubRunFlag[subProNum][m_WhileSubNC[subProNum] - 1] = 1;						//循环命令结束
					}
					else if(SubProgram_Operate[subProNum].Program[i].Key == K_CYCLEOVER)
					{
						m_WhileSubRunFlag[subProNum][m_WhileSubNC[subProNum] - 1] = 0;						//循环命令结束
						m_WhileSubNC[subProNum]--;
					}
					
					if(SubProgram_Operate[subProNum].Program[i].Key == K_IF)
					{
						m_IfElseSubNC[subProNum]++;
						m_IfElseSubJudgeRunFlag[subProNum][m_IfElseSubNC[subProNum] - 1] = 1;
					}
					else if(SubProgram_Operate[subProNum].Program[i].Key == K_ELSE)
					{
						m_IfElseSubJudgeRunFlag[subProNum][m_IfElseSubNC[subProNum] - 1] = 1;
					}
					else if(SubProgram_Operate[subProNum].Program[i].Key == K_OVER)
					{
						m_IfElseSubJudgeRunFlag[subProNum][m_IfElseSubNC[subProNum] - 1] = 0;
						m_IfElseSubNC[subProNum]--;
					}
				}
				
				if(SubProgram_Operate[subProNum].Program[m_JumpSubStepRunNum[subProNum]].Key == K_ELSE)
				{//如果跳转到else需要清除已执行标志
					m_IfElseSubJudgeRunFlag[subProNum][m_IfElseSubNC[subProNum] - 1] = 0;
				}
			}
			else if(m_JumpSubStepRunNum[subProNum] > 0 && ActionLine > m_JumpSubStepRunNum[subProNum])
			{
				for(i=ActionLine-1; i>m_JumpSubStepRunNum[subProNum]; i--)
				{//向下跳转时搜索逻辑命令
					if(SubProgram_Operate[subProNum].Program[i].Key == K_WHILE)
					{
						m_WhileSubRunFlag[subProNum][m_WhileSubNC[subProNum] - 1] = 0;						//循环命令结束
						m_WhileSubNC[subProNum]--;
					}
					else if(SubProgram_Operate[subProNum].Program[i].Key == K_CYCLEOVER)
					{
						m_WhileSubNC[subProNum]++;
						m_WhileSubRunFlag[subProNum][m_WhileSubNC[subProNum] - 1] = 1;						//循环命令结束
					}
					
					if(SubProgram_Operate[subProNum].Program[i].Key == K_IF)
					{
						m_IfElseSubJudgeRunFlag[subProNum][m_IfElseSubNC[subProNum] - 1] = 0;
						m_IfElseSubNC[subProNum]--;
					}
					else if(SubProgram_Operate[subProNum].Program[i].Key == K_ELSE)
					{
						m_IfElseSubJudgeRunFlag[subProNum][m_IfElseSubNC[subProNum] - 1] = 0;
					}
					else if(SubProgram_Operate[subProNum].Program[i].Key == K_OVER)
					{
						m_IfElseSubNC[subProNum]++;
						m_IfElseSubJudgeRunFlag[subProNum][m_IfElseSubNC[subProNum] - 1] = 1;
					}
				}
				
				if(SubProgram_Operate[subProNum].Program[m_JumpSubStepRunNum[subProNum]].Key == K_ELSE)
				{//如果跳转到else需要清除已执行标志
					m_IfElseSubJudgeRunFlag[subProNum][m_IfElseSubNC[subProNum] - 1] = 0;
				}
			}
			g_SubProgram_PresentLine[subProNum] = m_JumpSubStepRunNum[subProNum];
			break;
		case K_WHILE://循环命令
			if(m_WhileSubJudgeRes[subProNum][m_WhileSubNC[subProNum] - 1] == 1)
			{
				m_WhileSubJudgeRes[subProNum][m_WhileSubNC[subProNum] - 1] = 0;
				g_SubProgram_PresentLine[subProNum] = g_SubProgram_PresentLine[subProNum] + 1;
			}
			else
			{
				m_WhileSubRunFlag[subProNum][m_WhileSubNC[subProNum] - 1] = 0;						//循环命令结束
				for(i=ActionLine+1; i<SubProgram_Operate[subProNum].Num; i++)
				{//搜索当前while对应的循环结束命令
					if(SubProgram_Operate[subProNum].Program[i].Key == K_WHILE)
					{
						while_Counter++;
					}
					else if(SubProgram_Operate[subProNum].Program[i].Key == K_CYCLEOVER)
					{
						if(while_Counter > 0)
						{
							while_Counter--;
						}
						else
						{
							m_WhileSubNC[subProNum]--;
							g_SubProgram_PresentLine[subProNum] = i + 1;
							break;
						}
					}
				}
				if(i == SubProgram_Operate[subProNum].Num)
				{//未收索到循环结束命令则指向最后一行程序
					g_SubProgram_PresentLine[subProNum] = SubProgram_Operate[subProNum].Num - 1;
					m_WhileSubNC[subProNum]--;
				}
			}
			break;
		case K_CYCLEOVER://循环结束命令
			g_SubProgram_PresentLine[subProNum] = m_WhileSubLineNum[subProNum][m_WhileSubNC[subProNum] - 1];
			m_WhileSubNC[subProNum]--;
			break;
		case K_IF://IF判断条件命令
		case K_ELSE://ELSE判断条件命令
			if(m_IfElseSubJudgeRes[subProNum][m_IfElseSubNC[subProNum] - 1] == 1 && \
					m_IfElseSubJudgeRunFlag[subProNum][m_IfElseSubNC[subProNum] - 1] == 0)
			{
				m_IfElseSubJudgeRunFlag[subProNum][m_IfElseSubNC[subProNum] - 1] = 1;
				g_SubProgram_PresentLine[subProNum] = g_SubProgram_PresentLine[subProNum] + 1;
			}
			else
			{
				m_IfElseSubJudgeRes[subProNum][m_IfElseSubNC[subProNum] - 1] = 0;
				for(i=ActionLine+1; i<SubProgram_Operate[subProNum].Num; i++)
				{//搜索当前IF-ELSE对应的命令
					if(SubProgram_Operate[subProNum].Program[i].Key == K_IF)
					{//如果IF-ELSE命令还未执行过且先读到新的ELSE，那么就执行新行的判断
						if_else_Counter++;
					}
					else if(if_else_Counter == 0 && SubProgram_Operate[subProNum].Program[i].Key == K_ELSE && \
										m_IfElseSubJudgeRunFlag[subProNum][m_IfElseSubNC[subProNum] - 1] == 0)
					{//如果IF-ELSE命令还未执行过且先读到新的ELSE，那么就执行新行的判断
						g_SubProgram_PresentLine[subProNum] = i;
						break;
					}
					else if(SubProgram_Operate[subProNum].Program[i].Key == K_OVER)
					{//如果先读取到判断结束符，那么执行下一行
						if(if_else_Counter > 0)
						{
							if_else_Counter--;
						}
						else
						{
							m_IfElseSubNC[subProNum]--;
							g_SubProgram_PresentLine[subProNum] = i + 1;
							break;
						}
					}
				}
				if(i == SubProgram_Operate[subProNum].Num)
				{//未收索到判断结束命令则指向最后一行程序
					m_IfElseSubNC[subProNum] = 0;
					g_SubProgram_PresentLine[subProNum] = SubProgram_Operate[subProNum].Num - 1;
				}
			}
			break;
		case K_OVER://判断命令结束
			m_IfElseSubJudgeRunFlag[subProNum][m_IfElseSubNC[subProNum] - 1] = 0;
			m_IfElseSubNC[subProNum]--;
			g_SubProgram_PresentLine[subProNum] = g_SubProgram_PresentLine[subProNum] + 1;
			break;
		case K_SPECIAL://特殊指令处理
			if(SubProgram_Operate[subProNum].Program[ActionLine].Value1 == V_SUSPEND)
			{
				g_Auto_Order_Pause = TRUE;						//暂停运行相关操作
			}
			else if(SubProgram_Operate[subProNum].Program[ActionLine].Value1 == V_STOP)
			{
				g_Auto_Order_Stop = TRUE;							//停止运行相关操作
			}
			g_SubProgram_PresentLine[subProNum] = g_SubProgram_PresentLine[subProNum] + 1;
			break;
		default://其他命令对运行行号的操作
			g_SubProgram_PresentLine[subProNum] = g_SubProgram_PresentLine[subProNum] + SubAction_Step_Run_Num[subProNum];
			break;
	}
}

/**************************************************************************************************
**  函数名：  ActionStepControl()
**	输入参数：无
**	输出参数：无
**	函数功能：单行动作步骤控制函数
**	备注：	  控制每行程序单独动作  
**  开发日期：
***************************************************************************************************/
void ActionStepControl(void)
{
	u16 AutoPresentLine=0;//当前需要运行行号
	u8 ActionSetResult = 0;//动作执行状态寄存变量
	u8 ioNum = 0;
	
	switch(g_Auto_ActionRun_Step)//根据Step确定对应的执行环节
	{
		case 0:{		//动作输出环节
			if(Single_Mode_Enable == ENABLE && g_Auto_Reset_Flag == FALSE)
			{
				Action_Step_List_Num = 1;//单次运行指令=1
			}
			else if(Action_Step_List_Num == 0)//非单步模式且没有关联行数信息的时候
			{
				Action_Step_List_Num = AutoActionStepList(&Free_Program_Operate, g_Auto_PresentLine);	  //获取关联行的行数
				Action_Step_Run_Num = 0;											  																				//单次已经运行行数置0
			}
			
			if(JXS_Parameter.SpeedLevel != Temp_JXS_Parameter_SpeedLevel)
			{//实现机械手速度等级运动中进行修改
				JXS_Parameter.SpeedLevel = Temp_JXS_Parameter_SpeedLevel;
			}
			
			AutoPresentLine = g_Auto_PresentLine + Action_Step_Run_Num;												//当前需要运行行号=自动运行的行号+已运行行数
			ActionSetResult = AutoActionOutControl(&Free_Program_Operate, AutoPresentLine);		//获取执行情况
			
			g_Auto_ActionRun_Timer = 1;
			switch(ActionSetResult)//针对不同情况进行处理
			{
				case 0:		//动作正常被执行
					 g_Auto_ActionRun_Step = 1;	//进入动作确认环节
					 Action_Step_List_Num=0;	//关联行清空
					 Action_Step_Confirm_Num=0;	//
					 break;
				case 9:		//动作循环结束，这里完成了产量相关处理、步骤跳转到动作流程结束
					 ActionOverOperate();
					 break;
				case 10:	//当前行号长度异常
					 g_AutoStatue = AUTO_ERROR;
					//插入对应异常报警处理函数与报c警标志位
					 break;
				case 11:	//主要指令类型异常
					 g_AutoStatue = AUTO_ERROR;
					//插入对应异常报警处理函数与报警标志位
					 break;
				case 12:	//基本指令类型异常
					 g_AutoStatue = AUTO_ERROR;
					//插入对应异常报警处理函数与报警标志位
					 break;
				case 13:	//轴控指令类型异常
					 g_AutoStatue = AUTO_ERROR;
					//插入对应异常报警处理函数与报警标志位
					 break;
				case 14:	//IO控制指令类型异常
					 g_AutoStatue = AUTO_ERROR;
					//插入对应异常报警处理函数与报警标志位
					 break;
				case 15:
				   break;
				default:
					 break;
			}
		}break;
		case 1:{//动作确认环节
			if(AutoActionOutConfirm(&Free_Program_Operate, g_Auto_PresentLine + Action_Step_Confirm_Num) == TRUE)//判断动作确认情况
			{//动作确认成功
				Action_Step_Confirm_Num++;
				if(Action_Step_Confirm_Num == Action_Step_Run_Num)
				{//并行命令执行完成后，进入动作确认环节
					g_Auto_ActionRun_Step = 2;
					g_Auto_ActionRun_Timer = 0;
					Action_Step_Confirm_Num = 0;
					g_Auto_Valid_Timer = 0;
					g_Auto_Valid_Flag = FALSE;
				}
				
				if(g_Auto_ActionNcWait_Flag)//确认成功，清除输入超时检测标志
				{
					g_Auto_ActionNcWait_Flag = 0;
				}
			}
			else
			{//动作确认不成功
				if(Free_Program_Operate.Program[g_Auto_PresentLine + Action_Step_Confirm_Num].Key == K_DELAY)//延时命令可不进行超时检测
				{}
				else if(Free_Program_Operate.Program[g_Auto_PresentLine + Action_Step_Confirm_Num].Key == K_SUBPROGRAM)//子程序命令可不进行超时检测
				{}
				else if(Free_Program_Operate.Program[g_Auto_PresentLine+Action_Step_Confirm_Num].Key == K_OUTDETECT)
				{
					ioNum = Free_Program_Operate.Program[g_Auto_PresentLine+Action_Step_Confirm_Num].Value1;
					if(g_Auto_ActionRun_Timer > Free_Program_Operate.Program[g_Auto_PresentLine+Action_Step_Confirm_Num].Value3)
					{//判断输出检测超时
						Scan_TimeOut_OUTPUT(ioNum);	//会跳转至Error.c里面
						g_Auto_Order_Pause = TRUE;
						if(g_Program_Is_Debuging)
						{
							g_Auto_Order_Stop = TRUE;
						}
						
						g_Auto_ActionRun_Timer = 0;
					}
				}
				else if(g_Auto_ActionNcWait_Flag == 0)//非输入检测超时等待
				{
//					if(g_Auto_ActionRun_Timer > g_Auto_ActionTimeOut_Time)//当前命令执行时间未超时，防止命令执行时间过长
//					{
//						if(Free_Program_Operate.Program[g_Auto_PresentLine + Action_Step_Confirm_Num].Order == OR_IOORDER)//判断是IO指令
//						{//读取对于的IO指令，置相应的报警，暂用常开常闭报警信息。								
//							Scan_TimeOut_IO(Free_Program_Operate.Program[g_Auto_PresentLine + Action_Step_Confirm_Num].Key);
//						}
//						else
//						{
//							g_Auto_ActionTimeOut_Flag = TRUE;
//						}
//						g_Auto_Order_Pause = TRUE;
//						g_Auto_ActionRun_Timer=0;
//						g_Auto_Valid_Timer = 0;
//						g_Auto_Valid_Flag = FALSE;
//					}
				}
				else
				{//输入检测超时等待
					if(g_Auto_ActionRun_Timer > IO_Input_waittime[g_Auto_ActionNcWait_Flag-1])
					{//判断输入检测超时
						if(Free_Program_Operate.Program[g_Auto_PresentLine+Action_Step_Confirm_Num].Order <= OR_AXISORDER)
						{
						}
						else
						{
							//针对IO超时显示具体信息 20190524
							if(Free_Program_Operate.Program[g_Auto_PresentLine+Action_Step_Confirm_Num].Order == OR_IOORDER)//判断是IO指令
							{
								//读取对于的IO指令，置相应的报警，暂用常开常闭报警信息。
								Scan_TimeOut_IO(Free_Program_Operate.Program[g_Auto_PresentLine+Action_Step_Confirm_Num].Key);	//会跳转至Error.c里面
							}
							else
							{
								g_Auto_ActionTimeOut_Flag=TRUE;
							}
							g_Auto_Order_Pause = TRUE;
							if(g_Program_Is_Debuging)
							{
								g_Auto_Order_Stop = TRUE;
							}
						}
						g_Auto_ActionRun_Timer=0;
						g_Auto_Valid_Timer = 0;
						g_Auto_Valid_Flag = FALSE;
					}
				}
			}
		}break;
		case 2:{    //动作延时环节
				if(Action_Step_Run_Num > 1 || AutoActionOutDelay(&Free_Program_Operate, g_Auto_PresentLine + (Action_Step_Run_Num-1)) == TRUE)
				{//只执行并行语句最后一个语句的延时
					g_Auto_ActionRun_Step = 0;
					Action_Step_List_Num = 0;
					
					MainLogicComEndDeal(&Free_Program_Operate, g_Auto_PresentLine);//处理逻辑命令
					
					if(Single_Mode_Enable == ENABLE) 
					{//单步模式-进入暂停状态
						g_Auto_Order_Pause = TRUE;	
					}
					
					if(g_Program_Is_Debuging)
					{
						g_Auto_Order_Stop = TRUE;
					}
				}
			}break;
			case 3:{		//动作结束
				g_Auto_ActionRun_Step = 0;
				Action_Step_List_Num = 0;
				if(Single_Mode_Enable == ENABLE) //单步模式-进入暂停状态
				{
					g_Auto_Order_Pause = TRUE;	
				}
				if(g_Program_Is_Debuging)
				{
					g_Auto_Order_Stop = TRUE;
				}
				else
				{
					if(SC_Parameter.SC_Num == SC_Parameter.RW_Num && Program_Reset == FALSE)//当前加工任务完成
					{
						g_Auto_Order_Pause = TRUE;
						g_Auto_WorkFinished_Flag=TRUE;
					}
					else if(SC_Parameter.CJ_Num >= SC_Parameter.JG_Num && SC_Parameter.JG_Num != 0 && Program_Reset == FALSE)//当前加工任务完成 lin
					{
						g_Auto_Order_Pause = TRUE;
						g_Auto_CJWorkFinished_Flag = TRUE;
					}				
				}		
			}break;
			default:
				break;
	}
}

/**************************************************************************************************
**  函数名：  SubProgramActionControl(u8 subProNum)
**	输入参数：subProNum 执行的子程序编号
**	输出参数：无
**	函数功能：子程序运行控制函数，子程序不支持并行语句执行
**	备注：	  
**  作者：       
**  开发日期：
***************************************************************************************************/
void SubProgramActionControl(u8 subProNum)
{
	u8 SubProgramActionSetResult = 0;
	u16 AutoPresentLine = 0;
	u8 ioNum = 0;
	
	switch(g_SubProgram_ActionRun_Step[subProNum])
	{
		case 0:{		//动作输出环节

			SubAction_Step_List_Num[subProNum] = AutoActionStepList(&SubProgram_Operate[subProNum], g_SubProgram_PresentLine[subProNum]);	  //获取关联行的行数

			if(SubAction_Step_List_Num[subProNum] == 1)//不关联
			{
				SubAction_Step_Run_Num[subProNum] = 0;
			}
			
			AutoPresentLine = g_SubProgram_PresentLine[subProNum] + SubAction_Step_Run_Num[subProNum];		//当前需要运行行号=自动运行的行号+已运行行数
			SubProgramActionSetResult = SubProgramActionOutControl(subProNum, AutoPresentLine);						//此处动作输出函数另起了一个
			g_SubProgram_ActionRun_Timer[subProNum] = 1;
			switch(SubProgramActionSetResult)
			{//case条件不能是1~8，为同时执行的命令行预留
				case 0:		//动作正常被执行
					g_SubProgram_ActionRun_Step[subProNum] = 1;
					SubAction_Step_List_Num[subProNum] = 0;	//关联行清空
					SubAction_Step_Confirm_Num[subProNum] = 0;
					break;
				case 9:		//动作循环结束，子程序不需要进行程序结束后的产量相关处理
					SubAction_Step_List_Num[subProNum] = 0;					//子程序单次运行指令数	改子程序运行0617
					SubAction_Step_Run_Num[subProNum] = 0;					  //子程序已经运行的指令行数
					SubAction_Step_Confirm_Num[subProNum] = 0;				//子程序确认的指令行数
					g_SubProgram_ActionRun_Step[subProNum] = 0;							
					break;
				case 10:	//当前行号长度异常
					g_AutoStatue = AUTO_ERROR;
					//插入对应异常报警处理函数与报c警标志位
					break;
				case 11:	//主要指令类型异常
					g_AutoStatue = AUTO_ERROR;
					//插入对应异常报警处理函数与报警标志位
					break;
				case 12:	//基本指令类型异常
					g_AutoStatue = AUTO_ERROR;
					//插入对应异常报警处理函数与报警标志位
					break;
				case 13:	//轴控指令类型异常
					g_AutoStatue = AUTO_ERROR;
					//插入对应异常报警处理函数与报警标志位
					break;
				case 14:	//IO控制指令类型异常
					g_AutoStatue = AUTO_ERROR;
					//插入对应异常报警处理函数与报警标志位
					break;
				case 15:	//子程序动作结束
					g_SubProgram_ActionRun_Step[subProNum] = 3;
					break;
				default:
					break;
			}
		}break;
		case 1:{//动作确认环节			
			if(SubProgramActionOutConfirm(subProNum, g_SubProgram_PresentLine[subProNum] + SubAction_Step_Confirm_Num[subProNum]) == TRUE)		
			{//此处动作确认函数也另起了一个
				SubAction_Step_Confirm_Num[subProNum]++;
				if(SubAction_Step_Confirm_Num[subProNum] == SubAction_Step_Run_Num[subProNum])
				{
					g_SubProgram_ActionRun_Step[subProNum] = 2;
					g_SubProgram_ActionRun_Timer[subProNum] = 0;
					SubAction_Step_Confirm_Num[subProNum] = 0;
					g_SubAuto_Valid_Timer[subProNum] = 0;
					g_SubAuto_Valid_Flag[subProNum] = FALSE;
				}
				if(g_Auto_SubActionNcWait_Flag[subProNum])//确认成功，清除输入检测超时标志
				{
					g_Auto_SubActionNcWait_Flag[subProNum] = 0;
				}
			}
			else		//动作确认不成功
			{
				if(SubProgram_Operate[subProNum].Program[g_SubProgram_PresentLine[subProNum] + SubAction_Step_Confirm_Num[subProNum]].Key == K_DELAY)//延时命令可不进行超时检测
				{}
				else if(SubProgram_Operate[subProNum].Program[g_SubProgram_PresentLine[subProNum] + SubAction_Step_Confirm_Num[subProNum]].Key == K_SUBPROGRAM)//子程序命令可不进行超时检测
				{}
				else if(SubProgram_Operate[subProNum].Program[g_SubProgram_PresentLine[subProNum] + SubAction_Step_Confirm_Num[subProNum]].Key == K_OUTDETECT)
				{
					ioNum = SubProgram_Operate[subProNum].Program[g_SubProgram_PresentLine[subProNum] + SubAction_Step_Confirm_Num[subProNum]].Value1;
					if(g_SubProgram_ActionRun_Timer[subProNum] > SubProgram_Operate[subProNum].Program[g_SubProgram_PresentLine[subProNum] + SubAction_Step_Confirm_Num[subProNum]].Value3)
					{//判断输出检测超时
						Scan_TimeOut_OUTPUT(ioNum);	//会跳转至Error.c里面
						g_Auto_Order_Pause = TRUE;
						if(g_Program_Is_Debuging)
						{
							g_Auto_Order_Stop = TRUE;
						}
						
						g_SubProgram_ActionRun_Timer[subProNum] = 0;
					}
				}
				else if(g_Auto_SubActionNcWait_Flag[subProNum] == 0)//非输入检测超时等待
				{
//					if(g_SubProgram_ActionRun_Timer[subProNum] > g_SubProgram_ActionTimeOut_Time[subProNum])	//动作执行超时判断
//					{
//						if(SubProgram_Operate[subProNum].Program[g_SubProgram_PresentLine[subProNum] + SubAction_Step_Confirm_Num[subProNum]].Order == OR_IOORDER)//判断是IO指令
//						{
//							//读取对应的IO指令，置相应的报警，暂用常开常闭报警信息。
//							Scan_TimeOut_IO(SubProgram_Operate[subProNum].Program[g_SubProgram_PresentLine[subProNum] + SubAction_Step_Confirm_Num[subProNum]].Key);
//						}
//						else
//						{
//							g_Auto_ActionTimeOut_Flag = TRUE;
//						}
//						g_Auto_Order_Pause = TRUE;
//						g_SubProgram_ActionRun_Timer[subProNum] = 0;
//						g_SubAuto_Valid_Timer[subProNum] = 0;
//						g_SubAuto_Valid_Flag[subProNum] = FALSE;
//					}
				}
				else//输入检测超时等待
				{//判断输入检测超时等待
					if(g_SubProgram_ActionRun_Timer[subProNum] > IO_Input_waittime[g_Auto_SubActionNcWait_Flag[subProNum] - 1])
					{
						if(SubProgram_Operate[subProNum].Program[g_SubProgram_PresentLine[subProNum] + SubAction_Step_Confirm_Num[subProNum]].Order <= OR_AXISORDER)
						{
						}
						else
						{
							//针对IO超时显示具体信息 20190524
							if(SubProgram_Operate[subProNum].Program[g_SubProgram_PresentLine[subProNum]].Order == OR_IOORDER)//判断是IO指令
							{
								//读取对于的IO指令，置相应的报警，暂用常开常闭报警信息。
								Scan_TimeOut_IO(SubProgram_Operate[subProNum].Program[g_SubProgram_PresentLine[subProNum] + SubAction_Step_Confirm_Num[subProNum]].Key);
							}
							else
							{
								g_Auto_ActionTimeOut_Flag = TRUE;
							}
							g_Auto_Order_Pause = TRUE;
							if(g_Program_Is_Debuging)
							{
								g_Auto_Order_Stop = TRUE;
							}
						}
//						g_Auto_ActionRun_Timer=0;
						g_SubProgram_ActionRun_Timer[subProNum] = 0;
						g_SubAuto_Valid_Timer[subProNum] = 0;
						g_SubAuto_Valid_Flag[subProNum] = FALSE;
					}
				}
			}
		}break;
		case 2:{		//动作延时环节
			if(SubAction_Step_Run_Num[subProNum] > 1 || SubProgramActionOutDelay(subProNum, g_SubProgram_PresentLine[subProNum] + SubAction_Step_Run_Num[subProNum] - 1) == TRUE)
			{//如果有并行执行的程序，就不执行后面的延时
				g_SubProgram_ActionRun_Step[subProNum] = 0;
				SubAction_Step_List_Num[subProNum] = 0;							//关联行清空
				SubAction_Step_Confirm_Num[subProNum] = 0;
				
				SubLogicComEndDeal(subProNum, g_SubProgram_PresentLine[subProNum]);				//逻辑命令处理函数
				
				SubAction_Step_Run_Num[subProNum] = 0;
			}
		}break;
		case 3:{		//动作结束
			g_SubProgram_ActionRun_Step[subProNum] = 0;
			g_SubProgram_PresentLine[subProNum] = 0;
			SubAction_Step_Confirm_Num[subProNum] = 0;
			SubAction_Step_Run_Num[subProNum] = 0;
			SubAction_Step_List_Num[subProNum] = 0;			
			if(g_SubProgram_ContralEnd[subProNum] == TRUE)
			{
				g_SubProgram_Start[subProNum] = FALSE;
				g_SubProgram_Finish[subProNum] = TRUE;
				g_SubProgram_ContralEnd[subProNum] = FALSE;
			}
		}break;
		default:
			break;
	}
}

/**************************************************************************************************
**  函数名：  SubProgramStepControl(u8 subProNum)
**	输入参数：subProNum 子程序编号 
**	输出参数：无
**	函数功能：执行子程序
**	备注：	  
**  作者：       
**  开发日期：
***************************************************************************************************/
void SubProgramStepControl(u8 subProNum)
{
	if(g_SubProgram_Start[subProNum] == TRUE)
	{
		SubProgramActionControl(subProNum);			//对SubProgram_Operate结构体操作
	}
}

/**************************************************************************************************
**  函数名：  ActionOverOperate()
**	输入参数：无
**	输出参数：无
**	函数功能：循环结束操作处理函数
**	备注：	  一个周期结束后，数据恢复处理
**  作者：    
**  开发日期：
***************************************************************************************************/
void ActionOverOperate(void)
{
//	u8 Temp_data[30]={0};
	g_Auto_PresentLine = 0;
	Action_Step_List_Num=0;
	Action_Step_Run_Num=0;
	Action_Step_Confirm_Num=0;
	//写IIC,累计产量以及生产产量
	if((g_Program_Is_Debuging == FALSE) && (Program_Reset == FALSE))
	{
		SC_Parameter.SC_Num++;
		SC_Parameter.LJ_Num++;
				
		if(SC_Parameter.JG_Num!=0)
		{
			SC_Parameter.CJ_Num++;	
		}
		if(SC_Parameter.LJ_Num>MINROBOTPOSITION)
		{
			SC_Parameter.LJ_Num=0;
		}
//		Temp_data[0] = SC_Parameter.RW_Num;
//		Temp_data[1] = SC_Parameter.RW_Num>>8;
//		Temp_data[2] = SC_Parameter.RW_Num>>16;
//		Temp_data[3] = SC_Parameter.RW_Num>>24;
//		Temp_data[4] = SC_Parameter.CJ_Num;
//		Temp_data[5] = SC_Parameter.CJ_Num>>8;
//		Temp_data[6] = SC_Parameter.CJ_Num>>16;
//		Temp_data[7] = SC_Parameter.CJ_Num>>24;
//		Temp_data[8] = SC_Parameter.JG_Num;
//		Temp_data[9] = SC_Parameter.JG_Num>>8;
//		Temp_data[10] = SC_Parameter.JG_Num>>16;
//		Temp_data[11] = SC_Parameter.JG_Num>>24;
//		Temp_data[12] = SC_Parameter.SC_Num;
//		Temp_data[13] = SC_Parameter.SC_Num>>8;
//		Temp_data[14] = SC_Parameter.SC_Num>>16;
//		Temp_data[15] = SC_Parameter.SC_Num>>24;
//		Temp_data[16] = SC_Parameter.LJ_Num;
//		Temp_data[17] = SC_Parameter.LJ_Num>>8;
//		Temp_data[18] = SC_Parameter.LJ_Num>>16;
//		Temp_data[19] = SC_Parameter.LJ_Num>>24;
//		Temp_data[20] = SC_Parameter.NG_Num;
//		Temp_data[21] = SC_Parameter.NG_Num>>8;
//		Temp_data[22] = SC_Parameter.NG_Num>>16;
//		Temp_data[23] = SC_Parameter.NG_Num>>24;
//		W25QXX_Write(Temp_data,P_SC_NUM_ADDRESS,24);
	}
	g_Auto_ActionRun_Step = 3;
}

/**************************************************************************************************
**  函数名：  AutoPauseOperate()
**	输入参数：无
**	输出参数：无
**	函数功能：自动暂停操作
**	备注：	  暂停正在运行的全自动动作，设置相关标志位
**  作者：    
**  开发日期：
***************************************************************************************************/
void AutoPauseOperate(void)
{	
	u16 i = 0;
	
	if(AxisMoveFlag[X_Axsis] == ENABLE)		//如果正在发送脉冲
	{
		Servo_Stop(X_Axsis);
	}
	if(AxisMoveFlag[Z_Axsis] == ENABLE)		//如果正在发送脉冲
	{
		Servo_Stop(Z_Axsis);
	}
	if(AxisMoveFlag[L_Axsis] == ENABLE)		//如果正在发送脉冲
	{
		Servo_Stop(L_Axsis);
	}
	if(AxisMoveFlag[O_Axsis] == ENABLE)		//如果正在发送脉冲
	{
		Servo_Stop(O_Axsis);
	}
	if(AxisMoveFlag[U_Axsis] == ENABLE)		//如果正在发送脉冲
	{
		Servo_Stop(U_Axsis);
	}
	if(AxisMoveFlag[V_Axsis] == ENABLE)		//如果正在发送脉冲
	{
		Servo_Stop(V_Axsis);
	}
	
	for(i=0;i<OUTPUT_NUM;i++)
	{
		if(OutPut_Pause[i]==PAUSE_Select)
		{
			SetOutput(i);//指示灯灭
		}
	}	
	Action_Step_List_Num = 0;
	Action_Step_Run_Num = 0;
	Action_Step_Confirm_Num = 0;
	g_Auto_ActionRun_Step = 0;
	g_Auto_Valid_Timer = 0;
	g_Auto_Valid_Flag = FALSE;
	g_ActionDelay_Step = 0;
	for(i=0; i<SAVEPROGRAMNUM_SUB; i++)
	{
		g_SubProgram_ActionRun_Step[i] = 0;
		
		SubAction_Step_List_Num[i] = 0;					//子程序单次运行指令数	改子程序运行0617
		SubAction_Step_Run_Num[i] = 0;					  //子程序已经运行的指令行数
		SubAction_Step_Confirm_Num[i] = 0;				//子程序确认的指令行数
		g_SubProgram_ActionRun_Step[i] = 0;
		g_SubAuto_Valid_Timer[i] = 0;
		g_SubAuto_Valid_Flag[i] = FALSE;
		g_SubProgramDelay_Step[i] = 0;
	}
	g_Reset_ActionRun_Step = 0;
	if(g_Write_FlashFlag == TRUE && Auto_Mode != SINGLE_MODE)
	{
		STMFLASH_WriteRunData();
	}
}

/**************************************************************************************************
**  函数名：  AutoStopOperate()
**	输入参数：无
**	输出参数：无
**	函数功能：动作执行前，合法性判断
**	备注：	  停止全自动运行动作，并根据需求复位标志位
**  作者：    
**  开发日期：
***************************************************************************************************/
void AutoStopOperate(void)
{
	u16 i = 0;
//	u8 Temp_data[8]={0};
	
	for(i=0;i<OUTPUT_NUM;i++)
	{
		if(OutPut_Stop[i]==STOP_Select)
		{
			SetOutput(i);//指示灯灭
		}
	}

	if(g_Program_Is_Debuging)
	{
		g_Program_Is_Debuging = FALSE;
		g_AutoStatue = AUTO_WAITE;
	}
	else
	{
		
		if(g_Auto_PresentLine != 0 && g_Auto_PresentLine != Free_Program_Operate.Num-1)
		{
			SC_Parameter.NG_Num++;
		}
		g_AutoStatue = AUTO_WAITE;
		g_Auto_Order_Start = FALSE;
		g_Auto_Order_Pause = FALSE;
		g_Auto_Order_Stop = FALSE;
		g_Auto_PresentLine = 0;
		Single_Mode_Enable = DISABLE;
		Loop_Mode_Enable = DISABLE;
		Once_Mode_Enable = DISABLE;
		g_Auto_ActionTimeOut_Flag = FALSE;
//		Puls_Delay_Num = 0;
		Action_Step_List_Num = 0;
		Action_Step_Run_Num = 0;
		Action_Step_Confirm_Num = 0;
		g_Auto_Valid_Timer = 0;
		g_Auto_Valid_Flag = FALSE;
		for(i=0; i<SAVEPROGRAMNUM_SUB; i++)
		{
			g_Read_SubProgram[i] = FALSE;
			g_SubProgram_PresentLine[i] = 0;
			g_SubProgram_Step_Run[i] = FALSE;
			g_SubProgram_Start[i] = FALSE;
			g_SubProgram_Finish[i] = FALSE;
			
			SubAction_Step_List_Num[i] = 0;					//子程序单次运行指令数	改子程序运行0617
			SubAction_Step_Run_Num[i] = 0;					  //子程序已经运行的指令行数
			SubAction_Step_Confirm_Num[i] = 0;				//子程序确认的指令行数	
			g_SubProgram_ActionRun_Step[i] = 0;
			g_SubAuto_Valid_Timer[i] = 0;
			g_SubAuto_Valid_Flag[i] = FALSE;
		}
		
		m_ProRunTimeTotalCount =  0;
		
		for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
		{
			Flag_Keep_Move[i] = 0;
			Increment_Finished[i] = FALSE;
			if(Program_Axis_Origin_Flag[i] == TRUE)
			{
				Origin_Backed = FALSE;
				Program_Axis_Origin_Flag[i] = FALSE;
			}
			SlowPointFlag[i] = 0;
		}
		
		g_ActionDelay_Step = 0;
		g_Auto_ActionNcWait_Flag = 0;
		for(i=0; i<SAVEPROGRAMNUM_SUB; i++)
		{
			g_Auto_SubActionNcWait_Flag[i] = 0;
			g_SubProgramDelay_Step[i] = 0;
		}
	}
	
	if(g_Write_FlashFlag == TRUE)
	{
  	STMFLASH_WriteRunData();
	}
}


/******************* (C) COPYRIGHT 2015 Kingrobot manipulator Team *****END OF FILE****/
