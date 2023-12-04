/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/

#ifndef __signalwatch_h_
#define __signalwatch_h_

/******************USART命令类型Command***************************/
#define WORK_MODE         0x0A	   //工作模式
#define AUTO_MODE         0x0B	   //自动模式
#define MANUAL_OPERATE	  0x0C	   //手动操作动作命令
#define PARAMETER_COMMAND 0x0D	   //参数命令
#define WATCH_COMMAND     0x0E	   //监视命令
#define READ_IIC		  		0x0F	   //开机读取IIC命令
#define REAL_TIME_WATCH   0x10	   //实时监测命令
#define BACK_TO_ORIGIN    0x11	   //回原点命令
#define CANCLE_ALARM      0x12	   //取消报警
#define INITIALIZE_FINISH 0xBD     //初始化完成
#define ORIGIN_BACK		  	0xBE     //回原点完成

//读取IIC数据命令
#define   M_READ_IIC_1		0xF1			//功能设定参数
#define   M_READ_IIC_2		0xF2			//点参数
#define   M_READ_IIC_3		0xF3			//程序信息
#define   M_READ_IIC_4		0xF4			//程序内容
#define   M_READ_IIC_5		0xF5			//软限位信息
#define   M_READ_IIC_6		0xF6			//安全区信息
#define   M_READ_IIC_7		0xF7			//IO检测参数
#define   M_READ_IIC_8		0xF8			//
#define   M_READ_IIC_9	  0xF9	    //码垛参数
#define   M_READ_IIC_10	  0xFA	    //码垛名称信息
#define   M_READ_IIC_11	  0xFB	    //输出IO复位选择
#define   M_READ_IIC_12	  0xFC	    //参数拷贝
#define   M_READ_IIC_13	  0xFD	    //码垛拷贝
 
/**-------0x0E-----监视命令--------------**/
//#define   IO_DEBUG_LOCAL         	0x01	//IO调试-本地IO
#define   IO_DEBUG_INPUT1        	0x02	//IO调试-输入1
//#define   IO_DEBUG_INPUT2        	0x03	//IO调试-输入2
//#define   IO_DEBUG_INPUT3        	0x04	//IO调试-输入3
#define   IO_DEBUG_OUTPUT1_LCD   	0x05	//IO调试-输出1界面
//#define   IO_DEBUG_OUTPUT2_LCD   	0x06	//IO调试-输出2界面
//#define   IO_DEBUG_OUTPUT3_LCD   	0x07	//IO调试-输出3界面
#define   ROBOT_ORIGINED		 			0x08	//机械手是否已回原点
#define   ACTION_RESET_SCAN	     	0x09	//复位扫描
#define   ALARM_CLEAR    	     		0x0A	//消除报警
#define   ROBOT_STATUS      	 		0x0B	//机械手状态
#define   AUTO_PARAMETER         	0x0C	//自动运行参数-当前行-产量
#define   X_AXSIS_POSITION	     	0x0D	//X轴坐标
#define   L_AXSIS_POSITION       	0x0E	//L轴坐标
#define   Z_AXSIS_POSITION       	0x0F	//Z轴坐标
#define   O_AXSIS_POSITION       	0x10	//O轴坐标
#define   ROBOT_PRE_STATUS       	0x11	//机械手运行状态
#define   ROBOT_DEBUG_STATUS     	0x12	//机械手调试状态
#define   DELETE_POINT_STATUS	 		0x13	//删除点状态-DPF
#define   DELETE_PROGRAM_STATUS	 	0x14	//删除程序状态-DPF
#define	  ORIGIN_SETTING_STATUS		0x15	//获取原点设置状态
#define	  ORIGIN_RESETTING_STATUS	0x16	//获取原点重置状态
#define	  MACHINE_ORIGIN_STATUS		0x17	//获取机械回原点状态
#define   USER_CURNUM        		 	0x18	//用户变量
#define   DELETE_MD_STATUS  			0x19	//查询码垛是否删除完
#define   EXTENDCOM_STATUS			  0x1A	//获取机械手485通信状态变量
#define	  ONE_POINT_POSITON			  0x1B	//获取某一个点的位置信息
#define	  EXTEND_SERIAL_NUM			  0x1C	//获取序列号
#define   UV_AXSIS_POSITION       0x1D	//UV轴坐标
#define   MD_CURLAYER_CURNUM      0x1E	//查询物品当前层和当前个数

/**-------0x0D----参数命令--------------**/
#define   P_ROBOT_ENABLE_A_ORIGIN  	0x0A	//机械手使能和回原点
#define   P_WORK_MODE              	0x0B	//机械手工作模式
#define   P_AUTO_RUN               	0x0C	//机械手自动运行
#define   P_FREE_PROGRAM_SEND      	0x0D	//自由编程程序下发
#define   P_WATCH_COMMAND          	0x0E	//监视命令
#define   P_READ_IIC	           		0x0F	//开机读IIC
#define   P_IO_DEBUG_OUTPUT1       	0x1A	//IO调试-输出1
#define   P_IO_DEBUG_OUTPUT2       	0x1B	//IO调试-输出2
#define   P_IO_DEBUG_OUTPUT3       	0x1C	//IO调试-输出3
#define   P_MANUL_DEBUG            	0x1D	//手动调试
#define   P_PARAMETER_ORDER        	0x1E	//参数命令
#define   P_SYSTEM_SET_SEND		   		0x1F 	//设置下发
#define   MDPARA_COPY_SEND		 	 		0x2A 	//码垛拷贝下发

/**-------0x0A-----使能和回原点--------------**/
#define   P_ROBOT_ORIGIN   	     		0x01	//机械手回原点
#define		P_ORIGNSETTING						0x02	//原点设置
#define		P_ORIGNRESETTING					0x03	//原点重置
#define		P_MACHINEORIGIN						0x04	//机械回零
#define   P_ROBOT_ENABLE	     			0x07	//机械手是否使能

/**-------0x0B-----工作模式--------------**/
#define   P_AUTO_MODE              	0x01	//自动模式
#define   P_FREE_PROGRAM           	0x02	//自由编程
#define   P_IO_MODE                	0x03	//IO调试
#define   P_MANUL_MODE             	0x04	//手动调试

/**-------0x0C-----自动运行--------------**/
#define 	P_ONCE_MODE    	    		 0x0C	//单次模式
#define   P_CYCLE_MODE             0x01	//循环模式
#define   P_SINGLE_MODE            0x02	//单步模式
#define   P_ACTION_RUN             0x03	//启动
#define   P_ACTION_PAUSE           0x04	//暂停
#define   P_ACTION_STOP            0x05	//停止
#define   P_ACTION_PROGRAM         0x06	//选择运行的程序
#define   P_ACTION_RESET           0x07	//机械手复位
#define   P_ACTION_DEBUG           0x08	//机械手调试-自由编程的调试按钮
#define   P_ACTION_LORIGIN         0x09	//机械手L轴回零
#define   P_ACTION_COMMUTE         0x0A	//机械手下班功能

/**-------0x0D-----自由编程--------------**/
#define   P_PROGRAM_START          	0xE1	//程序开始
#define   PROGRAM_INFO             	0xE2	//程序信息描述
#define   PROGRAM_CONT             	0xE3	//程序内容
#define   PROGRAM_DELETE           	0xE4	//程序删除
#define   PROGRAM_FROM_USB_START   	0xE5	//USB程序拷贝发送开始
#define   PROGRAM_FROM_USB_END     	0xE6	//USB程序拷贝发送结束
#define   P_PROGRAM_DELETE		   		0xE7	//恢复出厂设置程序删除-DPF
#define   P_PROGRAM_END            	0xED	//程序结束

/**-------0x1A-----IO调试-输出1--- 0x01 Y0--0x0F Y14---------**/
#define   P_IODEBUG_OUTPUT1_1      0x01
#define   P_IODEBUG_OUTPUT1_2      0x02
#define   P_IODEBUG_OUTPUT1_3      0x03
#define   P_IODEBUG_OUTPUT1_4      0x04
#define   P_IODEBUG_OUTPUT1_5      0x05
#define   P_IODEBUG_OUTPUT1_6      0x06
#define   P_IODEBUG_OUTPUT1_7      0x07
#define   P_IODEBUG_OUTPUT1_8      0x08
#define   P_IODEBUG_OUTPUT1_9      0x09
#define   P_IODEBUG_OUTPUT1_10     0x0A
#define   P_IODEBUG_OUTPUT1_11     0x0B
#define   P_IODEBUG_OUTPUT1_12     0x0C
#define   P_IODEBUG_OUTPUT1_13     0x0D
#define   P_IODEBUG_OUTPUT1_14     0x0E
#define   P_IODEBUG_OUTPUT1_15     0x0F

/**-------0x1B-----IO调试-输出2--- 0X01 Y15---0X0F Y29--------**/
#define   P_IODEBUG_OUTPUT2_1      0x01
#define   P_IODEBUG_OUTPUT2_2      0x02
#define   P_IODEBUG_OUTPUT2_3      0x03
#define   P_IODEBUG_OUTPUT2_4      0x04
#define   P_IODEBUG_OUTPUT2_5      0x05
#define   P_IODEBUG_OUTPUT2_6      0x06
#define   P_IODEBUG_OUTPUT2_7      0x07
#define   P_IODEBUG_OUTPUT2_8      0x08
#define   P_IODEBUG_OUTPUT2_9      0x09
#define   P_IODEBUG_OUTPUT2_10     0x0A
#define   P_IODEBUG_OUTPUT2_11     0x0B
#define   P_IODEBUG_OUTPUT2_12     0x0C
#define   P_IODEBUG_OUTPUT2_13     0x0D
#define   P_IODEBUG_OUTPUT2_14     0x0E
#define   P_IODEBUG_OUTPUT2_15     0x0F

/**-------0x1C-----IO调试-输出3--- 0X01 Y30---0X0A Y39--------**/
#define   P_IODEBUG_OUTPUT3_1      0x01
#define   P_IODEBUG_OUTPUT3_2      0x02
#define   P_IODEBUG_OUTPUT3_3      0x03
#define   P_IODEBUG_OUTPUT3_4      0x04
#define   P_IODEBUG_OUTPUT3_5      0x05
#define   P_IODEBUG_OUTPUT3_6      0x06
#define   P_IODEBUG_OUTPUT3_7      0x07
#define   P_IODEBUG_OUTPUT3_8      0x08
#define   P_IODEBUG_OUTPUT3_9      0x09
#define   P_IODEBUG_OUTPUT3_10     0x0A

/**-------0x1D-----手动调试-----------------**/
#define	  P_AXIS_MANUL_SPEED		 0x01	//轴-手动速度值
#define	  P_AXIS_STEP_MM		     0x02	//轴-寸动距离 mm为单位

#define   P_XAXIS_MOVE_LEFT      0x03	//X轴-左移
#define   P_XAXIS_MOVE_RIGHT     0x04	//X轴-右移

#define   P_YAXIS_MOVE_LEFT      0x05	//Y轴-左移
#define   P_YAXIS_MOVE_RIGHT     0x06	//Y轴-右移

#define   P_ZAXIS_MOVE_LEFT      0x07	//Z轴-左移
#define   P_ZAXIS_MOVE_RIGHT     0x08	//Z轴-右移

#define   P_OAXIS_MOVE_LEFT      0x09	//O轴-左移
#define   P_OAXIS_MOVE_RIGHT     0x0A	//O轴-右移

#define   P_UAXIS_MOVE_LEFT      0x0B	//U轴-左移
#define   P_UAXIS_MOVE_RIGHT     0x0C	//U轴-右移

#define   P_VAXIS_MOVE_LEFT      0x0D	//V轴-左移
#define   P_VAXIS_MOVE_RIGHT     0x0E	//V轴-右移

#define   P_POINT_SAVE           0x10	//点保存
#define   P_DELETE_ONEPOINT      0x11	//存储点管理中删除一个点信息
#define   P_MODIFY_ONEAXSIS      0x12	//存储点管理中修改一个点中的一个轴坐标
#define   P_RENAME_ONEPOINT      0x13	//存储点管理中修改一个点的名称

#define   P_DELETE_ALLPOINT  		 0x20	//删除所有点

////^^^20210219^^^^////



#define   P_POINT_SAVE_HEAD    0x5000	//轴存储地址头
#define   P_POINT_SAVE_LEN     0x20		//一个点预留32位
#define   P_POINT_SAVE_NUM     0x28		//一共40个点


#define   P_XAXIS_SAVE_HEAD    0x5000	//轴存储地址头
#define   P_XAXIS_SAVE_LEN     0x1E		//一个点预留30位
#define   P_XAXIS_SAVE_NUM     0x4B		//一个轴75个点

#define   P_ZAXIS_SAVE_HEAD    (P_XAXIS_SAVE_HEAD + P_XAXIS_SAVE_LEN * P_XAXIS_SAVE_NUM)
#define   P_ZAXIS_SAVE_LEN     P_XAXIS_SAVE_LEN
#define   P_LAXIS_SAVE_HEAD    (P_ZAXIS_SAVE_HEAD + P_ZAXIS_SAVE_LEN * P_XAXIS_SAVE_NUM)
#define   P_LAXIS_SAVE_LEN     P_XAXIS_SAVE_LEN
#define   P_OAXIS_SAVE_HEAD    (P_LAXIS_SAVE_HEAD + P_LAXIS_SAVE_LEN * P_XAXIS_SAVE_NUM)
#define   P_OAXIS_SAVE_LEN     P_XAXIS_SAVE_LEN


/**************************笛卡尔坐标系参数存储*******************************/
#define   P_CARTESIAN_PARA_HEAD    0x7330																				//笛卡尔坐标系参数存储-起始地址
#define   P_CARTESIAN_PARA_LEN     0x50																					//参数长度-预留80字节

/**************************配方参数存储**************************************/
#define   P_FORMULATION_PARA_HEAD    0x7390																			//配方参数存储-起始地址
#define   P_FORMULATION_PARA_LEN     0x20																				//参数长度-预留32字节

/**************************电机控制参数存储**********************************/
#define   P_MOTORCONTROL_PARA_HEAD    0x73C0																		//电机控制参数存储-起始地址
#define   P_MOTORCONTROL_PARA_LEN     0x40																			//参数长度-预留64字节

///**************************物联网参数存储************************************/
//#define   P_INTERNET_ADDRESS         0x7410  															//物联网序列号起始地址，物联网序列号不需要拷贝
//#define   P_INTERNET_PARA_LEN        0x40																	//参数长度-预留64字节

/**************************扩展轴参数存储************************************/
#define   P_EXTENDAIX_ADDRESS        0x7460  																		//两个扩展轴
#define   P_EXTENDAIX_PARA_SIZE      0x30																				//每个扩展轴参数保存数据大小48字节

/**************************绝对值零点位置存储********************************/
#define   P_SERVO_JDZ_ZERO_HEAD    0x7FE0																				//绝对值零点位置存储-起始地址
#define   P_SERVO_JDZ_ZERO_LEN     0x20																					//参数长度-预留16字节

/**************************码垛相关参数存储*********************************/
#define   P_MD_PARA_HEAD    0x30000																				//码垛参数存储-起始地址
#define   P_MD_PARA_LEN     0x40																					//参数长度-预留64位
#define   P_MD_POINT_HEAD  	(P_MD_PARA_HEAD + P_MD_PARA_LEN)							//物品1码垛点1-起始地址
#define   P_MD_POINT_LEN    0x40																					//一个点存储长度-预留64位：一个物品每层64个点，最大循环层数为4
#define   P_MD_POINT_TOTLE  (P_MD_POINT_LEN * MD_POINT_NUM * LOOP_MAX)		//一个物品所有点的总长度：一个物品每层64个点，最大循环层数为4

#define   P_MD_GOOD_LEN     (P_MD_PARA_LEN + P_MD_POINT_TOTLE)						//一个物品参数总长度

#define   P_MD_PARA_END    	(P_MD_PARA_HEAD	+ P_MD_GOOD_LEN * MD_GOOD_NUM)	//码垛参数存储-结束地址:0x30000 + 0x12C000 = 0x15C000

#define   P_MD_CLEAR_NUM    (((P_MD_PARA_END - P_MD_PARA_HEAD) / 4096) + 1)		//码垛参数存储-多少页：300

/**************************运行时可变参数的存储区*************************************/

#define   P_SC_NUM_ADDRESS				  0xFFF000															//生产产量、累积产量、NG产量，存储在最后一个产区,占用0xFFF000-0xFFF01E

#define   P_USER_ADDRESS				    0xFFF070															//用户变量,占用0xFFF070-0xFFF128

#define   P_MD_SORT_ADDRESS				  0xFFF200															//存储码垛分拣模式下的每个物品的当前层和个数,占用MD_GOOD_NUM*2=60*2=0x78

#define   P_INTERNET_ADDRESS        0xFFF300  														//物联网序列号起始地址，物联网序列号不需要拷贝
#define   P_INTERNET_PARA_LEN       0x40																	//物联网参数长度-预留64字节


/*运行时可变参数的存储区*/
#define SAVE_PROTECT_SIZE				  					0x1000						//保护区域的大小为4K
#define SAVE_PROTECT_START_ADDRESS				  0xFFF000					//保护区域的起始地址
#define SAVE_PROTECT_END_ADDRESS				  	(SAVE_PROTECT_START_ADDRESS + SAVE_PROTECT_SIZE - 1)		//保护区域的结束地址
#define SAVE_PROTECT_BACKUP_ADDRESS					0xFFE000					//保护区域备份区的起始地址
#define SAVE_PROTECT_BACKUP_END_ADDRESS			(SAVE_PROTECT_BACKUP_ADDRESS + SAVE_PROTECT_SIZE - 1)		//保护区域备份区的结束地址
#define SAVE_PROTECT_VALUE									(0x55)						//保护区域最后一个地址的校验字节的固定数值，用于验证是否写失败

/**-------0x1E-----参数命令-----------------**/
#define	  P_PARAMETER_SOFT_LIMIT	   		0x01			//软限位
#define   P_PARAMETER_SAFE_AREA        	0x02			//安全区
#define   P_PARAMETER_FUCTION_SET      	0x03			//功能参数
#define	  P_PARAMETER_IO_DETECT_SET	   	0X04 			//输入参数
#define	  P_PARAMETER_OUTPUT_SET	    	0X05 			//输出IO复位选择
#define	  P_PARAMETER_IO_INSWITCH_SET	  0X06 			//输入复用开关
#define	  P_PARAMETER_OUT_INSWITCH_SET	0X07 			//输出复用开关
#define   P_PARAMETER_MD_PARA      			0x08			//码垛参数
#define   P_PARAMETER_MD_POINT      		0x09			//码垛点参数
#define   P_PARAMETER_MD_DELETE      		0x0A			//码垛参数-恢复出厂
#define   P_PARAMETER_CARTESIAN_PARA   	0x0B			//笛卡尔坐标系参数

extern void WatchFunction(void);
extern void OrderDecoding(void);
extern u32 Program_RunTime;

#endif

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team *****END OF FILE****/

