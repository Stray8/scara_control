/*************** (C) COPYRIGHT 2015 Kingrobot manipulator Team ************************
* File Name          : Auto_2.h
* Author             : Fenkella Zhou
* Version            : V1.0.0
* Date               : 21/10/2015
* Description        : 自动运行头文件
***************************************************************************************/
#ifndef _Auto_2_h_
#define _Auto_2_h_

#include "Parameter.h"
#include "StatusControl.h"

#define AUTO_WAITE				0
#define AUTO_RUNNING			1
#define AUTO_PAUSE				2
#define AUTO_ERROR				3

#define LISTNUM 					6						//并行程序不能大于6行

extern u8  g_AutoStatue;
extern u16 g_Auto_PresentLine;

//子程序相关变量
extern u8  g_SubProgram_Step_Run[SAVEPROGRAMNUM_SUB];    				//是否运行子程序，在主程序中置位
extern u8  g_SubProgram_Start[SAVEPROGRAMNUM_SUB];							//子程序开始
extern u8  g_SubProgram_Finish[SAVEPROGRAMNUM_SUB];							//子程序结束
extern u8  g_SubProgram_ContralEnd[SAVEPROGRAMNUM_SUB];					//控制子程序结束
extern u8  g_Read_SubProgram[SAVEPROGRAMNUM_SUB];								//子程序读取完成
extern u16 g_SubProgram_PresentLine[SAVEPROGRAMNUM_SUB];				//子程序运行行号
extern u32 g_SubProgram_ActionRun_Timer[SAVEPROGRAMNUM_SUB];
extern u8  g_SubProgram_ActionRun_Step[SAVEPROGRAMNUM_SUB];
extern u32 g_SubProgram_ActionTimeOut_Time[SAVEPROGRAMNUM_SUB];

extern u8  g_Auto_Order_Start;
extern u8  g_Auto_Order_Pause;
extern u8  g_Auto_Order_Stop;

extern u32 g_Auto_ActionRun_Timer;
extern u8  g_Auto_ActionRun_Step;
extern u8  g_Auto_ActionNcWait_Flag;
extern u8  g_Auto_SubActionNcWait_Flag[SAVEPROGRAMNUM_SUB];
extern u32 g_Auto_ActionTimeOut_Time;
extern u8  g_Auto_ActionTimeOut_Flag;
extern u8  g_Auto_ActionError_Flag;
extern u8  g_Auto_WorkFinished_Flag;
extern u8  g_Auto_CJWorkFinished_Flag;
extern u8  MD_PositionErr_Flag;
extern u8  g_Start_ActionRun_Step;
extern u8  g_Reset_ActionRun_Step;
extern u32 g_Auto_Valid_Timer;
extern u8  g_Auto_Valid_Flag;
extern u8 g_SubAuto_Valid_Flag[SAVEPROGRAMNUM_SUB];
extern u32 g_SubAuto_Valid_Timer[SAVEPROGRAMNUM_SUB];


extern u16 Action_Step_List_Num;
extern u16 Action_Step_Run_Num;
extern u16 Action_Step_Confirm_Num;
extern u16 SubAction_Step_List_Num[SAVEPROGRAMNUM_SUB];
extern u16 SubAction_Step_Run_Num[SAVEPROGRAMNUM_SUB];
extern u16 SubAction_Step_Confirm_Num[SAVEPROGRAMNUM_SUB];

//jump功能参数
extern u16 m_JumpStepRunNum;											//主程序跳转到的行号
extern u16 m_JumpSubStepRunNum[SAVEPROGRAMNUM_SUB];								//子程序跳转到的行号

//while功能参数
#define WHILE_NEST_MAX							10														//while嵌套最大层数
extern u8  m_WhileNC;														//主程序while命令嵌套层数计数器
extern u8  m_WhileRunFlag[WHILE_NEST_MAX];				//主程序while命令运行标志
extern u16 m_WhileLineNum[WHILE_NEST_MAX];				//主程序while命令行号
//extern u16 m_WhileOverLineNum[WHILE_NEST_MAX];	//主程序while命令对应结束命令的行号
extern u16 m_WhileCycCounter[WHILE_NEST_MAX];		//主程序while命令循环次数
extern u8  m_WhileJudgeType[WHILE_NEST_MAX];			//主程序while命令条件类型
extern s32 m_WhileJudgePar1[WHILE_NEST_MAX];			//主程序while命令参数1
extern s32 m_WhileJudgePar2[WHILE_NEST_MAX];			//主程序while命令参数2
extern u8  m_WhileJudgeRes[WHILE_NEST_MAX];			//主程序while命令判断结果，0失败，1成功

extern u8  m_WhileSubNC[SAVEPROGRAMNUM_SUB];																//子程序while命令嵌套层数计数器
extern u8  m_WhileSubRunFlag[SAVEPROGRAMNUM_SUB][WHILE_NEST_MAX];						//子程序while命令运行标志
extern u16 m_WhileSubLineNum[SAVEPROGRAMNUM_SUB][WHILE_NEST_MAX];						//子程序while命令行号
//extern u16 m_WhileOverSubLineNum[SAVEPROGRAMNUM_SUB][WHILE_NEST_MAX];			//子程序while命令对应结束命令的行号
extern u16 m_WhileSubCycCounter[SAVEPROGRAMNUM_SUB][WHILE_NEST_MAX];				//子程序while命令循环次数
extern u8  m_WhileSubJudgeType[SAVEPROGRAMNUM_SUB][WHILE_NEST_MAX];					//子程序while命令条件类型
extern s32 m_WhileSubJudgePar1[SAVEPROGRAMNUM_SUB][WHILE_NEST_MAX];					//子程序while命令参数1
extern s32 m_WhileSubJudgePar2[SAVEPROGRAMNUM_SUB][WHILE_NEST_MAX];					//子程序while命令参数2
extern u8  m_WhileSubJudgeRes[SAVEPROGRAMNUM_SUB][WHILE_NEST_MAX];					//子程序while命令判断结果，0失败，1成功

//if-else功能参数
#define IF_ELSE_NEST_MAX							10																			//If-Else嵌套最大层数
extern u8  m_IfElseNC;																				//主程序If-Else命令嵌套层数计数器
extern u8  m_IfElseJudgeType[IF_ELSE_NEST_MAX];							//主程序If-Else命令条件类型
extern s32 m_IfElseJudgePar1[IF_ELSE_NEST_MAX];							//主程序If-Else命令参数1
extern s32 m_IfElseJudgePar2[IF_ELSE_NEST_MAX];							//主程序If-Else命令参数2
extern u8  m_IfElseJudgeRes[IF_ELSE_NEST_MAX];								//主程序If-Else命令判断结果，0失败，1成功
extern u8  m_IfElseJudgeRunFlag[IF_ELSE_NEST_MAX];						//主程序If-Else命令组合执行标志，0未执行，1已执行

extern u8  m_IfElseSubNC[SAVEPROGRAMNUM_SUB];																	//子程序If-Else命令嵌套层数计数器
extern u8  m_IfElseSubJudgeType[SAVEPROGRAMNUM_SUB][IF_ELSE_NEST_MAX];				//子程序If-Else命令条件类型
extern s32 m_IfElseSubJudgePar1[SAVEPROGRAMNUM_SUB][IF_ELSE_NEST_MAX];				//子程序If-Else命令参数1
extern s32 m_IfElseSubJudgePar2[SAVEPROGRAMNUM_SUB][IF_ELSE_NEST_MAX];				//子程序If-Else命令参数2
extern u8  m_IfElseSubJudgeRes[SAVEPROGRAMNUM_SUB][IF_ELSE_NEST_MAX];					//子程序If-Else命令判断结果，0失败，1成功
extern u8  m_IfElseSubJudgeRunFlag[SAVEPROGRAMNUM_SUB][IF_ELSE_NEST_MAX];			//子程序If-Else命令组合执行标志，0未执行，1已执行

extern u8 Flag_Falling_Edge;
extern u8 Flag_Rising_Edge;
extern u8 Flag_Falling_Edge_Sub;
extern u8 Flag_Rising_Edge_Sub;
extern u8 Flag_Keep_Move[Axis_Num + Ext_Axis_Num];

extern u8 Detect_Falling_Edge;
extern u8 Detect_Rising_Edge;
extern u8 Detect_Falling_Edge_Sub[SAVEPROGRAMNUM_SUB];
extern u8 Detect_Rising_Edge_Sub[SAVEPROGRAMNUM_SUB];

extern u8 Program_Origin_Axis;
extern u8 Program_Axis_Origin_Flag[Axis_Num + Ext_Axis_Num];
//void Program_Axis_Origin(u8);

void AutoModeControl(void);
void AutoPauseOperate(void);
void AutoStopOperate(void);
void ActionStepControl(void);


//主要指令--order--
#define   OR_BASICORDER      			0x01  //基本指令
#define   OR_AXISORDER       			0x02  //轴控指令
#define   OR_IOORDER         			0x03  //IO指令

//子命令---key------//
#define   K_PROGRAMSTART           0x01	 //主程序开始
#define   K_PROGRAMEND             0x02	 //主程序结束
#define   K_DELAY                  0x03	 //延时
#define   K_SUBPROGRAM             0x04  //子程序
#define   K_JUMP                   0x05  //跳转指令
#define   K_WHILE                  0x06  //While指令
#define   K_CYCLEOVER              0x07  //循环结束
#define   K_IF                     0x08  //执行IF语句
#define   K_ELSE                   0x09  //执行ELSE语句
#define   K_OVER                   0x0A	 //执行结束IF-ELSE语句
#define   K_SPECIAL                0x0B  //执行特殊指令
#define   K_OUTDETECT            	 0x0C	 //输出检测
#define   K_PULSE_OUTPUT           0x67	 //脉宽输出
#define   K_USER                   0x68	 //用户变量
#define   K_KEEP_MOVE			   			0x0D	 //正向搜索
#define   K_MDPOSITION          	0x0E	 //码垛位置
#define   K_MDPOCOUNT	          	0x0F	 //码垛计数
#define   K_XAXIS                 0x10	 //X轴移动
#define   K_LAXIS                 0x11	 //L轴移动
#define   K_ZAXIS                 0x12	 //Z轴移动
#define   K_OAXIS                 0x13	 //O轴移动
#define   K_INCREMENT_RUNNING     0x66	 //102增量运动
#define   K_NEGTIVE_SEARCH        0x69	 //105反向搜索
#define   K_MACHINE_ORIGIN        0x6A	 //106机械回零
#define   K_POSSET				        0x6B	 //107位置设置
#define		K_SLOWPOINT							0x6C	 //108减速点
#define		K_INTER_START						0x6D		//109插补开始
#define		K_INTER_OVER						0x6E		//110插补结束
#define		K_ADVENCE								0x6F		//111提前确认
#define		K_INTER_LINE						0x70		//112直线插补
#define		K_AXISMOVE							0x71		//113轴移动
#define		K_ANGLE_ARC							0x72		//114圆心角圆弧

//单阀输出-双路检测-4路
#define   K_IOINSTRUCT_OUTPUT1     0x14	//输出指令1-置位Y0
#define   K_IOINSTRUCT_OUTPUT2     0x15	//输出指令2-复位Y0
#define   K_IOINSTRUCT_OUTPUT3     0x16	//输出指令3
#define   K_IOINSTRUCT_OUTPUT4     0x17	//输出指令4
#define   K_IOINSTRUCT_OUTPUT5     0x18	//输出指令5
#define   K_IOINSTRUCT_OUTPUT6     0x19	//输出指令6
#define   K_IOINSTRUCT_OUTPUT7     0x1A	//输出指令7
#define   K_IOINSTRUCT_OUTPUT8     0x1B	//输出指令8

#define   K_IOINSTRUCT_OUTPUT9     0x1C	//输出指令9 -置位Y4
#define   K_IOINSTRUCT_OUTPUT10    0x1D	//输出指令10-复位Y4
#define   K_IOINSTRUCT_OUTPUT11    0x1E	//输出指令11
#define   K_IOINSTRUCT_OUTPUT12    0x1F	//输出指令12
#define   K_IOINSTRUCT_OUTPUT13    0x20	//输出指令13
#define   K_IOINSTRUCT_OUTPUT14    0x21	//输出指令14
#define   K_IOINSTRUCT_OUTPUT15    0x22	//输出指令15
#define   K_IOINSTRUCT_OUTPUT16    0x23	//输出指令16

#define   K_IOINSTRUCT_OUTPUT17    0x24	//输出指令17
#define   K_IOINSTRUCT_OUTPUT18    0x25	//输出指令18
#define   K_IOINSTRUCT_OUTPUT19    0x26	//输出指令19
#define   K_IOINSTRUCT_OUTPUT20    0x27	//输出指令20

#define   K_IOINSTRUCT_OUTPUT21    0x28	//输出指令21
#define   K_IOINSTRUCT_OUTPUT22    0x29	//输出指令22
#define   K_IOINSTRUCT_OUTPUT23    0x2A	//输出指令23
#define   K_IOINSTRUCT_OUTPUT24    0x2B	//输出指令24
#define   K_IOINSTRUCT_OUTPUT25    0x2C	//输出指令25
#define   K_IOINSTRUCT_OUTPUT26    0x2D	//输出指令26
#define   K_IOINSTRUCT_OUTPUT27    0x2E	//输出指令27
#define   K_IOINSTRUCT_OUTPUT28    0x2F	//输出指令28
#define   K_IOINSTRUCT_OUTPUT29    0x30	//输出指令29
#define   K_IOINSTRUCT_OUTPUT30    0x31	//输出指令30
#define   K_IOINSTRUCT_OUTPUT31    0x32	//输出指令31
#define   K_IOINSTRUCT_OUTPUT32    0x33	//输出指令32
#define   K_IOINSTRUCT_OUTPUT33    0x34	//输出指令33
#define   K_IOINSTRUCT_OUTPUT34    0x35	//输出指令34
#define   K_IOINSTRUCT_OUTPUT35    0x36	//输出指令35
#define   K_IOINSTRUCT_OUTPUT36    0x37	//输出指令36
#define   K_IOINSTRUCT_OUTPUT37    0x38	//输出指令37
#define   K_IOINSTRUCT_OUTPUT38    0x39	//输出指令38
#define   K_IOINSTRUCT_OUTPUT39    0x3A	//输出指令39
#define   K_IOINSTRUCT_OUTPUT40    0x3B	//输出指令40
#define   K_IOINSTRUCT_OUTPUT41    0x3C	//输出指令41
#define   K_IOINSTRUCT_OUTPUT42    0x3D	//输出指令42
#define   K_IOINSTRUCT_OUTPUT43    0x3E	//输出指令43
#define   K_IOINSTRUCT_OUTPUT44    0x3F	//输出指令44
#define   K_IOINSTRUCT_OUTPUT45    0x40	//输出指令45
#define   K_IOINSTRUCT_OUTPUT46    0x41	//输出指令46
#define   K_IOINSTRUCT_OUTPUT47    0x42	//输出指令47
#define   K_IOINSTRUCT_OUTPUT48    0x43	//输出指令48
#define   K_IOINSTRUCT_OUTPUT49    0x44	//输出指令49
#define   K_IOINSTRUCT_OUTPUT50    0x45	//输出指令50
#define   K_IOINSTRUCT_OUTPUT51    0x46	//输出指令51
#define   K_IOINSTRUCT_OUTPUT52    0x47	//输出指令52

//输入信号检测接口
#define   K_IOINSTRUCT_INPUT1      0x48	//输入指令1
#define   K_IOINSTRUCT_INPUT2      0x49	//输入指令2
#define   K_IOINSTRUCT_INPUT3      0x4A	//输入指令3
#define   K_IOINSTRUCT_INPUT4      0x4B	//输入指令4
#define   K_IOINSTRUCT_INPUT5      0x4C	//输入指令5
#define   K_IOINSTRUCT_INPUT6      0x4D	//输入指令6
#define   K_IOINSTRUCT_INPUT7      0x4E	//输入指令7
#define   K_IOINSTRUCT_INPUT8      0x4F	//输入指令8
#define   K_IOINSTRUCT_INPUT9      0x50	//输入指令9
#define   K_IOINSTRUCT_INPUT10     0x51	//输入指令10
#define   K_IOINSTRUCT_INPUT11     0x52	//输入指令11
#define   K_IOINSTRUCT_INPUT12     0x53	//输入指令12
#define   K_IOINSTRUCT_INPUT13     0x54	//输入指令13
#define   K_IOINSTRUCT_INPUT14     0x55	//输入指令14
#define   K_IOINSTRUCT_INPUT15     0x56	//输入指令15
#define   K_IOINSTRUCT_INPUT16     0x57	//输入指令16
#define   K_IOINSTRUCT_INPUT17     0x58	//输入指令17
#define   K_IOINSTRUCT_INPUT18     0x59	//输入指令18
#define   K_IOINSTRUCT_INPUT19     0x5A	//输入指令19
#define   K_IOINSTRUCT_INPUT20     0x5B	//输入指令20
#define   K_IOINSTRUCT_INPUT21     0x5C	//输入指令21
#define   K_IOINSTRUCT_INPUT22     0x5D	//输入指令22
#define   K_IOINSTRUCT_INPUT23     0x5E	//输入指令23
#define   K_IOINSTRUCT_INPUT24     0x5F	//输入指令24
#define   K_IOINSTRUCT_INPUT25     0x60	//输入指令25
#define   K_IOINSTRUCT_INPUT26     0x61	//输入指令26
#define   K_IOINSTRUCT_INPUT27     0x62	//输入指令27
#define   K_IOINSTRUCT_INPUT28     0x63	//输入指令28
#define   K_IOINSTRUCT_INPUT29     0x64	//输入指令29
#define   K_IOINSTRUCT_INPUT30     0x65	//输入指令30


//程序的value值定义
#define V_Z_POSITION				0X04	//Z轴坐标
#define V_ONCE							0X08	//单次
#define V_CYCLE							0X09	//循环
#define V_O_METHOD					0X0A	//O方式
#define V_X_POSITION				0X0B	//X轴坐标
#define V_Y_POSITION				0X0C	//Y轴坐标
#define V_PROGRAM_START			0x10	//程序开始
#define V_PROGRAM_END				0x11	//程序结束
#define V_ONCE_RUN					0x12	//单次执行
#define V_ONLY_EQUAL				0x14	//只等于
#define V_EQUAL							0x15	//等于（倍数）
#define V_NOT_EQUAL					0x16	//不等于
#define V_NOTONLY_EQUAL			0x17	//不等于（倍数）
#define V_R_METHOD					0x18	//R方式
#define V_I_METHOD					0x19	//I方式
#define V_R_LINE_NUM_IO			0x1A	//R行号/IO口
#define V_R_NUM							0x1B	//R数量/H&L
#define V_RI_NULL						0x1C	//NULL
#define V_MD_AXSIS					0X1D	//码垛轴选择 29-36
#define V_SUBPROGRAM_SEQ		0x25	//子程序号
#define V_JUMP_TO						0x26	//跳转
#define V_HIGH_LEVEL       	0x27	//高电平
#define V_LOW_LEVEL					0x28	//低电平
#define V_RISING_EDGE				0x29	//上升沿
#define V_FALLING_EDGE			0x2A	//下降沿
#define V_KEEP_MOVE_X				0x2B	//X轴
#define V_KEEP_MOVE_Y				0x2C	//Y轴
#define V_KEEP_MOVE_Z				0x2D	//Z轴
#define V_KEEP_MOVE_O				0x2E	//O轴
#define V_SUSPEND						0X2F	//特殊指令-暂停
#define V_STOP							0X30	//特殊指令停止
#define V_COUNTER						0X31	//R方式-计数器
#define V_JUMP_LABEL				0X32	//标签
#define V_LABEL_NUM					0X33	//标签号
#define V_DISTANCE    	    0X34	//52增量位移
#define V_USEFUL						0X35	//53有效
#define V_USELESS    	      0X36	//54无效
#define V_SET	   	  				0X38	//56置位
#define V_RESET    	  			0X39	//57复位
#define V_USER1    	  			0X3A	//58user1
#define V_USER2    	  			0X3B	//59user2
#define V_USER3    	  			0X3C	//60user3
#define V_USER4    	  			0X3D	//61user4
#define V_USER5    	  			0X3E	//62user5
#define V_USER6    	  			0X3F	//63user6
#define V_USER7    	  			0X40	//64user7
#define V_USER8    	  			0X41	//65user8
#define V_ADD    	    			0X42	//66 +
#define V_MINUS    	  			0X43	//67 -
#define V_UEQUAL						0X44	//68 =
#define V_MULTIP      			0X45	//69 *
#define V_DIVIDE    				0X46	//70 /
#define V_EXCESS    				0X47	//71 %
#define V_SLOWPOS						0X49	//74减速点
#define V_MDGOOD						0X55	//85物品
#define V_NUMBER						0X56	//86个数
#define V_LAYER_NUM					0X57	//87层数
#define V_LAYER_FULL				0X58	//88层满
#define V_STACK_FULL				0X59	//89垛满
#define V_MORE_THAN					0X5A	//90大于
#define V_LESS_THAN					0X5B	//91小于
#define V_AUTO							0X5C	//92自动
#define V_P_METHOD					0X5D	//93P方式
#define V_AXISCHOOSE    		0X5E	//94轴选择
#define V_SETPOSITION    		0X5F	//95设置坐标值
#define V_XAXISGREATER    	0X60	//96X轴大于等于 
#define V_YAXISGREATER    	0X61	//97Y轴大于等于
#define V_ZAXISGREATER    	0X62	//98Z轴大于等于
#define V_OAXISGREATER    	0X63	//99O轴大于等于
#define V_XAXISEQUAL       	0X64	//100X轴等于
#define V_YAXISEQUAL       	0X65	//101Y轴等于
#define V_ZAXISEQUAL       	0X66	//102Z轴等于
#define V_OAXISEQUAL       	0X67	//103O轴等于
#define V_XAXISLESS        	0X68	//104X轴小于等于
#define V_YAXISLESS        	0X69	//105Y轴小于等于
#define V_ZAXISLESS         0X6A	//106Z轴小于等于
#define V_OAXISLESS       	0X6B	//107O轴小于等于
#define V_FORMULATION      	0X6C	//108配方
#define V_KEEP_MOVE_U				0X6D	//109U轴
#define V_KEEP_MOVE_V				0X6E	//110V轴
#define V_ADVANCE						0X6F	//111提前确认量

#endif

/******************* (C) COPYRIGHT 2015 Kingrobot manipulator Team *****END OF FILE****/

