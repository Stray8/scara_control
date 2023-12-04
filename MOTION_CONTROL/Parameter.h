/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/

#ifndef __parameter_h_
#define __parameter_h_
#include "stm32f4xx.h"
#include "StatusControl.h"

/******************参数命令定义***************************/
#define LARGESTPROGRAMNUM 				200						//编程最多指令数
#define SAVEPROGRAMNUM 						20						//可存储的程序总数
#define SAVEPROGRAMNUM_MAIN 			10	  				//可存储的主程序个数
#define SAVEPROGRAMNUM_SUB 				10	  					//可存储的子程序个数

#define SAVEPROINT 								75						//每个轴最多能保存的点
#define SAVESAFEAREA 							3							//最多可用的安全区个数
#define SAVESOFTLIMIT 						4							//最多可用的软限位个数

#define MINROBOTPOSITION   				1000000    		//轴运动的最小坐标
#define MAXROBOTPOSITION   				9000000    		//轴运动的最大坐标

#define MAXSAVEPROINT 						40						//每个轴最多能保存的点
#define OUTPUT_ASSOCIATE_NUM    	2		//输出关联组

#define	OUTPUT_NUM	18   //输入输出个数
#define	INPUT_NUM		24

//夹爪个数
#define MDgrip_Num	6
//用户个数
#define USER_NUM	8


//自由编程
typedef struct 
{
	u8  Flag;  				//标志位
	u8  List;  				//指令编号
	u8  Order; 				//指令类型
	u8  Key;   				//子命令
	u32 Value1;				//指令参数
	u32 Value2;
	s32 Value3;
//	s32 XPoint;
//	s32 YPoint;
//	s32 ZPoint;
}FreeProgram;

//程序存储
typedef struct 
{
	u8  Flag;	   			//是否有程序
	u8  Code;	   			//程序的编号，下发给主控制板是识别用
	u32 Name;	   			//程序名称
	u32 Name2;
	u32 Name3;
	u16 Num;	   			//保存程序的规模大小，用于运行时显示判断
	FreeProgram Program[LARGESTPROGRAMNUM];
}SaveProgram;

//程序存储
typedef struct 
{
	u8  Flag;	   			//是否有程序
	u8  Code;	   			//程序编号
	u32 Name;	   			//程序名称
	u32 Name2;
	u32 Name3;
	u16 Num;	   			//程序规模，程序的条数
	u32 Address;   		//IIC的存储地址
}SaveProgramIICAddress;

extern SaveProgramIICAddress Program_IIC_Address[SAVEPROGRAMNUM];	//保存所有程序的基本信息及存储地址
extern SaveProgram Free_Program_Operate;													//当前选中的主程序
extern SaveProgram SubProgram_Operate[SAVEPROGRAMNUM_SUB];				//子程序存放数组

extern u8 Start_Recieve_Program_Flag;															//程序接收开始标志
extern u8 Current_Delete_Program;	        												//当前删除程序

typedef struct 
{
	u32 Left_Limit  ; //-
	u32 Right_Limit ; //+
	u8  Switch_Limit; //开关
}SoftLimit;

typedef struct
{
	u32 MaxDistance;	//X轴和O轴最大距离
	u32 MinDistance;	//X轴和O轴最小安全距离
} SoftDistance;//安全行程

typedef struct 
{
	u32 X_Left ;
	u32 Z_Up ;
	u32 X_Right ;
	u32 Z_Down ;
	u8  SafeArea_Switch;
}SafeAreas;

typedef struct
{
	u8 carCoordSwitch;						//笛卡尔坐标开关 0关 1开
	u8 MDCoordType;								//码垛坐标类型 0轴坐标 1笛卡尔坐标
	u32 pitchLength;							//丝杆螺距
	u32 length[2];								//机械臂1、2长度
//	s32 startPoint[Axis_Num];			//坐标轴上机械末端起点
//	s32 revolveAngle[Axis_Num];		//旋转角度
//	s32 revolvePoint[Axis_Num];		//旋转角度对应位移
	u8 axisType[Axis_Num];				//轴类型 0直线轴 1旋转轴
	u8 axisBackMinDir[Axis_Num];	//轴回最小位置的方向 0顺时针 1逆时针
	u8 axisInterpFlag[Axis_Num];	//轴插补开关 0关 1开
} ST_Cartesian; //笛卡尔坐标系

//typedef struct
//{
//	float MC_P[Axis_Num];
//	float MC_I[Axis_Num];
//	float MC_D[Axis_Num];
//	float MC_C;													//用于消除稳态误差的值
//} ST_MotorControl_PID; //PID参数设置

//点存储
typedef struct 
{
	u8  Flag;			//是否有点
	u32 Name;		  //点的名称  clj0831
	u32 Name2;
	u32 Name3;
	u32 Point_X;		//X的值
	u32 Point_L;		//Y的值	
	u32 Point_Z;		//Z的值
	u32 Point_O;		//O的值
}SavePoint;

////点存储缓存区定义存储
//typedef struct 
//{
//	u8  Flag;
//	u32 Point;			//X的值
//}SavePointPar;

typedef struct
{
	u16 Time[Axis_Num];
}ACC;

typedef struct 
{
	u8  Axis;
	u8  Origin;											//回原点方式
	ACC Accelerate;
	u8  SpeedLevel;
	u32 AxisOriginSpeed[Axis_Num];	//单轴回零速度
	u8  OriginDir[Axis_Num];				//四轴回零方向，0正向，1反向
	u8  OriginPos[Axis_Num];				//四轴原点位置，0负端，1正端
//	u32 PulseTime;
	u8	NcPausein;									//外部暂停
	u8  NcStopin; 									//外部停止-DPF
	u8  NcStartin;									//外部启动-DPF
	u8  NcOrignin;									//外部回零-DPF
	u8  LCcirculation;							//料仓循环方式--参数暂时留存
	u32 OrignOffset[Axis_Num];			//原点偏移
	u8  OrignSignOnOff[Axis_Num];		//原点信号常开常闭
	u8  LimitSignOnOff[Axis_Num];		//限位信号常开常闭
	u8  AlarmSignal;								//报警信号高低电平
	u8  AlarmSwitch[Axis_Num];			//检测电机报警信号开关
	u8  MDgripSwitch;								//码垛夹爪IO开关
	u8  MDgripNum;									//码垛夹爪IO序号
	u8  MDgripPort[MDgrip_Num];			//码垛夹爪IO端口
	u32 A_Circle_Pulse[Axis_Num];		//单圈脉冲
	u32 A_Circle_Distance[Axis_Num];//单圈距离
	u8  OutputAssociate[OUTPUT_ASSOCIATE_NUM];	//输出关联开关Y0Y1,Y2Y3	
	u8  ZAxsisAvoidace;							//Z轴防撞
	u32 ZAxsisLimit;								//Z轴下限		
}JXSParameter;

typedef struct 
{
	u8  Switch;											//功能开关
	u8  Server;											//伺服选择
	u8  Resolu; 										//分辨率	
	u32 Circle_Pluse[Axis_Num];			//单圈脉冲
	u8  Motion_Dir[Axis_Num];				//运动方向
  u8  OriginSetting[Axis_Num];		//绝对值原点设置
	u32 SERVO_STEP_LONG;						//编码器位数对应的指令数	23位-8388608 17位-131072

}JDZParameter;	//绝对值参数设定

typedef struct
{
	u32  Name;
	u32  Name1;
	u32  Name2;
} IONameParameter;

#define LOOP_MAX						10			//最多循环层数

#define MD_GOOD_NUM					60		//物品数量：15*4=60
#define MD_GOOD_PAGE_NUM		4
#define MD_GOOD_PAGE_PER		15

#define MD_POINT_NUM				64		//每层示教点数量：8*8=64
#define MD_POINT_PAGE_NUM		8
#define MD_POINT_PAGE_PER		8

#define PF_IONUM						4

typedef struct
{
	s32 point[Axis_Num];			//码垛点
	s32 waitPoint[Axis_Num];	//等待点
} ST_MDPostion; //码垛位置

typedef struct
{
	u8 stackType;					//垛型：0：示教，1：矩形
	u8 property;					//属性：0：码垛，1：拆垛
	u8 revolveMode;				//旋转方式：0：电机，1：气缸
	u8 gasPort;						//气缸端口
	u8 topSwitch;					//顶层功能
	s32 goodOffset[Axis_Num];	//物料偏移--笛卡尔坐标系
	s32 goodheight;				//物品高度
	u8 stackLayer;				//堆叠层数
	u8 loopLayer;					//循环层数：最多10层，顶层功能开启时，最上面一层为顶层
	u8 layerNum[LOOP_MAX];//本层个数
	u8 horNum;						//横向个数
	u8 verNum;						//纵向个数
//	ST_MDPostion sPostion[LOOP_MAX][MD_POINT_NUM];		//全部示教点
} ST_MDParameter; //码垛参数

typedef struct
{
	u8 mdMethed;					//码垛方式：0：连续，1：分拣
	u8 totalGood;					//总物品数
	u8 startGood;					//起始物品
	u8 curGood;						//当前物品
	u8 curLayer;					//当前层数
	u8 curNum;						//当前个数
} ST_MDRunPara; //码垛运行参数

typedef struct
{
	u8	pfIOnum[PF_IONUM];					//配方端口
	u8	pfGood[PF_IONUM];						//配方物品
	u8	pfSwitch[PF_IONUM];					//配方开关
} PFParameter; //配方参数

typedef struct 
{
	u32 RW_Num ;					//任务计划
	u32 SC_Num ;					//生产
	u32 CJ_Num ;					//抽检
	u32 JG_Num ;					//间隔
	u32 LJ_Num ;					//累计
	u32 NG_Num ;					//错误产量
}SCParameter;

typedef struct
{
	u32  USER_Name1[USER_NUM] ;
	u32  USER_Name2[USER_NUM] ;
	u32  USER_Name3[USER_NUM] ;
	s32  INIT_Num[USER_NUM] ;	//初始值
	s32  CURR_Num[USER_NUM] ;	//当前值
	u8   ELEC_RESET[USER_NUM] ;	 //开机置0
	u8   START_RESET[USER_NUM] ; //启动置0
}USERParameter; //用户变量

typedef struct
{
	u8   Switch;
	u8  Sequence[12];	
} InternetParameter;

typedef struct
{
	u8  E_OriginPosition;     //原点位置
	u32 E_OriginOffset;       //原点偏移
	u32 E_Circle_Pulse;	      //单圈脉冲
	u32 E_Circle_Distance;	  //单圈距离
	u16 E_AccAcc;             //加加速
	u16 E_AccTime;            //加速度时间
	u32 E_MaxDistance;	      //最大行程
	u8  E_Origin_Set; 			  //原点设置
} ExtendAixParameter;

extern ExtendAixParameter ExtendAix_Parameter[EXTEN_AXIS_NUM];//扩展轴参数
extern InternetParameter Internet_Parameter;                  //物联网参数
extern JXSParameter JXS_Parameter;						  							//轴运动参数
extern SCParameter SC_Parameter ;						  								//生产参数
extern USERParameter USER_Parameter ;						  						//用户变量
extern SoftLimit Robot_SoftLimit[SAVESOFTLIMIT];		    			//轴软限位参数
extern SafeAreas Robot_Safe_Area[SAVESAFEAREA];		      			//安全区域参数
extern u8 All_Program_Deleted_Flag;														//恢复出厂设置所有程序已经删除标志位
extern JDZParameter JDZ_Parameter;						  							//JDZ相关参数
extern ST_MDParameter sMD_Parameter; 												  //码垛参数
extern ST_MDRunPara sMD_RunPara;															//码垛运行参数
extern u8 sMD_CurMDCode;																			//当前物品编号
extern u8 sMD_GoodOffset_flag;																//当前物品-物料偏移标志
extern u8 All_MD_Deleted_Flag;																//恢复出厂设置所有码垛已经删除标志位
extern ST_Cartesian sCartesian_Para;													//笛卡尔坐标系参数
//extern ST_MotorControl_PID sMC_PID_Para;											//电机控制PID参数
extern PFParameter PF_Parameter;															//配方
extern u8 sMD_FlashCurLayer[MD_GOOD_NUM];  										//各个码垛的当前层
extern u8 sMD_FlashCurNum[MD_GOOD_NUM];												//各个码垛的当前个
extern u8  g_Write_FlashFlag;				  												//产量等参数写flash标志位

extern u8  g_Run_Program_Num ;																//当前运行的程序编号,0表示无选择程序
extern u8 g_Run_Program_Num_Pre;															//用于保持当前选中的程序号
extern u32 SaveProgram_IIC_Address;														//待处理的程序地址
extern u8  SaveProgram_IIC_Num ;															//待处理的程序行数

extern u32 Axsis_Maxlength[Axis_Num + Ext_Axis_Num];													//轴运动的最大位置
extern u32 Axsis_Minlength[Axis_Num + Ext_Axis_Num];													//轴运动的最小位置

extern u8  Temp_JXS_Parameter_SpeedLevel;											//速度等级暂存变量，用于实现在运动时修改速度等级

extern u8 Temp_IO_Switch_Parameter[INPUT_NUM];
extern u8 Temp_OUT_Switch_Parameter[OUTPUT_NUM];

extern SoftDistance  Robot_SoftDistance;
extern SoftDistance  Temp_SoftDistance;


#define ManulSavePointMaxNum											40					//点库最大的点数量
extern SavePoint Manul_Save_Point[ManulSavePointMaxNum];
extern float Step_Coefficient[Axis_Num + EXTEN_AXIS_NUM];


#define SYSTEM_TIME_MAX 2000000000														//系统时钟计数最大值，单位10ms，20000000s
extern u32 m_SystemTimeCounter;																//系统时钟计数器

extern u8 SpeedLevel_ParSave;																	//速度等级修改标志
extern u8 JDZ_ZeroPos_ParSave;																//绝对位置设置标志

/*总线电机速度、加速度、位置的转换系数*/
extern float  Axsis_ParVelChange;														//速度、加速度转换系数
extern float  Axsis_ParPosChange;														//位置转换系数

extern u32 DistanceTransPulse(u8 axis,u32 distance);
extern void ParameterOrder(void);
extern void FreeProgramSend(void);
extern void ReadIICData(void);
extern void ReadIICSysParameter(void);
extern void Write_System_Set_IIC(void);
extern void Write_MDPara_Copy_IIC(void);
extern void AxisInterpParSave(void);

extern u16 Torque_T_count; //

#endif

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team *****END OF FILE****/

