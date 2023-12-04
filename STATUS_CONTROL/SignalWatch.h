/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/

#ifndef __signalwatch_h_
#define __signalwatch_h_

/******************USART��������Command***************************/
#define WORK_MODE         0x0A	   //����ģʽ
#define AUTO_MODE         0x0B	   //�Զ�ģʽ
#define MANUAL_OPERATE	  0x0C	   //�ֶ�������������
#define PARAMETER_COMMAND 0x0D	   //��������
#define WATCH_COMMAND     0x0E	   //��������
#define READ_IIC		  		0x0F	   //������ȡIIC����
#define REAL_TIME_WATCH   0x10	   //ʵʱ�������
#define BACK_TO_ORIGIN    0x11	   //��ԭ������
#define CANCLE_ALARM      0x12	   //ȡ������
#define INITIALIZE_FINISH 0xBD     //��ʼ�����
#define ORIGIN_BACK		  	0xBE     //��ԭ�����

//��ȡIIC��������
#define   M_READ_IIC_1		0xF1			//�����趨����
#define   M_READ_IIC_2		0xF2			//�����
#define   M_READ_IIC_3		0xF3			//������Ϣ
#define   M_READ_IIC_4		0xF4			//��������
#define   M_READ_IIC_5		0xF5			//����λ��Ϣ
#define   M_READ_IIC_6		0xF6			//��ȫ����Ϣ
#define   M_READ_IIC_7		0xF7			//IO������
#define   M_READ_IIC_8		0xF8			//
#define   M_READ_IIC_9	  0xF9	    //������
#define   M_READ_IIC_10	  0xFA	    //���������Ϣ
#define   M_READ_IIC_11	  0xFB	    //���IO��λѡ��
#define   M_READ_IIC_12	  0xFC	    //��������
#define   M_READ_IIC_13	  0xFD	    //��⿽��
 
/**-------0x0E-----��������--------------**/
//#define   IO_DEBUG_LOCAL         	0x01	//IO����-����IO
#define   IO_DEBUG_INPUT1        	0x02	//IO����-����1
//#define   IO_DEBUG_INPUT2        	0x03	//IO����-����2
//#define   IO_DEBUG_INPUT3        	0x04	//IO����-����3
#define   IO_DEBUG_OUTPUT1_LCD   	0x05	//IO����-���1����
//#define   IO_DEBUG_OUTPUT2_LCD   	0x06	//IO����-���2����
//#define   IO_DEBUG_OUTPUT3_LCD   	0x07	//IO����-���3����
#define   ROBOT_ORIGINED		 			0x08	//��е���Ƿ��ѻ�ԭ��
#define   ACTION_RESET_SCAN	     	0x09	//��λɨ��
#define   ALARM_CLEAR    	     		0x0A	//��������
#define   ROBOT_STATUS      	 		0x0B	//��е��״̬
#define   AUTO_PARAMETER         	0x0C	//�Զ����в���-��ǰ��-����
#define   X_AXSIS_POSITION	     	0x0D	//X������
#define   L_AXSIS_POSITION       	0x0E	//L������
#define   Z_AXSIS_POSITION       	0x0F	//Z������
#define   O_AXSIS_POSITION       	0x10	//O������
#define   ROBOT_PRE_STATUS       	0x11	//��е������״̬
#define   ROBOT_DEBUG_STATUS     	0x12	//��е�ֵ���״̬
#define   DELETE_POINT_STATUS	 		0x13	//ɾ����״̬-DPF
#define   DELETE_PROGRAM_STATUS	 	0x14	//ɾ������״̬-DPF
#define	  ORIGIN_SETTING_STATUS		0x15	//��ȡԭ������״̬
#define	  ORIGIN_RESETTING_STATUS	0x16	//��ȡԭ������״̬
#define	  MACHINE_ORIGIN_STATUS		0x17	//��ȡ��е��ԭ��״̬
#define   USER_CURNUM        		 	0x18	//�û�����
#define   DELETE_MD_STATUS  			0x19	//��ѯ����Ƿ�ɾ����
#define   EXTENDCOM_STATUS			  0x1A	//��ȡ��е��485ͨ��״̬����
#define	  ONE_POINT_POSITON			  0x1B	//��ȡĳһ�����λ����Ϣ
#define	  EXTEND_SERIAL_NUM			  0x1C	//��ȡ���к�
#define   UV_AXSIS_POSITION       0x1D	//UV������
#define   MD_CURLAYER_CURNUM      0x1E	//��ѯ��Ʒ��ǰ��͵�ǰ����

/**-------0x0D----��������--------------**/
#define   P_ROBOT_ENABLE_A_ORIGIN  	0x0A	//��е��ʹ�ܺͻ�ԭ��
#define   P_WORK_MODE              	0x0B	//��е�ֹ���ģʽ
#define   P_AUTO_RUN               	0x0C	//��е���Զ�����
#define   P_FREE_PROGRAM_SEND      	0x0D	//���ɱ�̳����·�
#define   P_WATCH_COMMAND          	0x0E	//��������
#define   P_READ_IIC	           		0x0F	//������IIC
#define   P_IO_DEBUG_OUTPUT1       	0x1A	//IO����-���1
#define   P_IO_DEBUG_OUTPUT2       	0x1B	//IO����-���2
#define   P_IO_DEBUG_OUTPUT3       	0x1C	//IO����-���3
#define   P_MANUL_DEBUG            	0x1D	//�ֶ�����
#define   P_PARAMETER_ORDER        	0x1E	//��������
#define   P_SYSTEM_SET_SEND		   		0x1F 	//�����·�
#define   MDPARA_COPY_SEND		 	 		0x2A 	//��⿽���·�

/**-------0x0A-----ʹ�ܺͻ�ԭ��--------------**/
#define   P_ROBOT_ORIGIN   	     		0x01	//��е�ֻ�ԭ��
#define		P_ORIGNSETTING						0x02	//ԭ������
#define		P_ORIGNRESETTING					0x03	//ԭ������
#define		P_MACHINEORIGIN						0x04	//��е����
#define   P_ROBOT_ENABLE	     			0x07	//��е���Ƿ�ʹ��

/**-------0x0B-----����ģʽ--------------**/
#define   P_AUTO_MODE              	0x01	//�Զ�ģʽ
#define   P_FREE_PROGRAM           	0x02	//���ɱ��
#define   P_IO_MODE                	0x03	//IO����
#define   P_MANUL_MODE             	0x04	//�ֶ�����

/**-------0x0C-----�Զ�����--------------**/
#define 	P_ONCE_MODE    	    		 0x0C	//����ģʽ
#define   P_CYCLE_MODE             0x01	//ѭ��ģʽ
#define   P_SINGLE_MODE            0x02	//����ģʽ
#define   P_ACTION_RUN             0x03	//����
#define   P_ACTION_PAUSE           0x04	//��ͣ
#define   P_ACTION_STOP            0x05	//ֹͣ
#define   P_ACTION_PROGRAM         0x06	//ѡ�����еĳ���
#define   P_ACTION_RESET           0x07	//��е�ָ�λ
#define   P_ACTION_DEBUG           0x08	//��е�ֵ���-���ɱ�̵ĵ��԰�ť
#define   P_ACTION_LORIGIN         0x09	//��е��L�����
#define   P_ACTION_COMMUTE         0x0A	//��е���°๦��

/**-------0x0D-----���ɱ��--------------**/
#define   P_PROGRAM_START          	0xE1	//����ʼ
#define   PROGRAM_INFO             	0xE2	//������Ϣ����
#define   PROGRAM_CONT             	0xE3	//��������
#define   PROGRAM_DELETE           	0xE4	//����ɾ��
#define   PROGRAM_FROM_USB_START   	0xE5	//USB���򿽱����Ϳ�ʼ
#define   PROGRAM_FROM_USB_END     	0xE6	//USB���򿽱����ͽ���
#define   P_PROGRAM_DELETE		   		0xE7	//�ָ��������ó���ɾ��-DPF
#define   P_PROGRAM_END            	0xED	//�������

/**-------0x1A-----IO����-���1--- 0x01 Y0--0x0F Y14---------**/
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

/**-------0x1B-----IO����-���2--- 0X01 Y15---0X0F Y29--------**/
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

/**-------0x1C-----IO����-���3--- 0X01 Y30---0X0A Y39--------**/
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

/**-------0x1D-----�ֶ�����-----------------**/
#define	  P_AXIS_MANUL_SPEED		 0x01	//��-�ֶ��ٶ�ֵ
#define	  P_AXIS_STEP_MM		     0x02	//��-�綯���� mmΪ��λ

#define   P_XAXIS_MOVE_LEFT      0x03	//X��-����
#define   P_XAXIS_MOVE_RIGHT     0x04	//X��-����

#define   P_YAXIS_MOVE_LEFT      0x05	//Y��-����
#define   P_YAXIS_MOVE_RIGHT     0x06	//Y��-����

#define   P_ZAXIS_MOVE_LEFT      0x07	//Z��-����
#define   P_ZAXIS_MOVE_RIGHT     0x08	//Z��-����

#define   P_OAXIS_MOVE_LEFT      0x09	//O��-����
#define   P_OAXIS_MOVE_RIGHT     0x0A	//O��-����

#define   P_UAXIS_MOVE_LEFT      0x0B	//U��-����
#define   P_UAXIS_MOVE_RIGHT     0x0C	//U��-����

#define   P_VAXIS_MOVE_LEFT      0x0D	//V��-����
#define   P_VAXIS_MOVE_RIGHT     0x0E	//V��-����

#define   P_POINT_SAVE           0x10	//�㱣��
#define   P_DELETE_ONEPOINT      0x11	//�洢�������ɾ��һ������Ϣ
#define   P_MODIFY_ONEAXSIS      0x12	//�洢��������޸�һ�����е�һ��������
#define   P_RENAME_ONEPOINT      0x13	//�洢��������޸�һ���������

#define   P_DELETE_ALLPOINT  		 0x20	//ɾ�����е�

////^^^20210219^^^^////



#define   P_POINT_SAVE_HEAD    0x5000	//��洢��ַͷ
#define   P_POINT_SAVE_LEN     0x20		//һ����Ԥ��32λ
#define   P_POINT_SAVE_NUM     0x28		//һ��40����


#define   P_XAXIS_SAVE_HEAD    0x5000	//��洢��ַͷ
#define   P_XAXIS_SAVE_LEN     0x1E		//һ����Ԥ��30λ
#define   P_XAXIS_SAVE_NUM     0x4B		//һ����75����

#define   P_ZAXIS_SAVE_HEAD    (P_XAXIS_SAVE_HEAD + P_XAXIS_SAVE_LEN * P_XAXIS_SAVE_NUM)
#define   P_ZAXIS_SAVE_LEN     P_XAXIS_SAVE_LEN
#define   P_LAXIS_SAVE_HEAD    (P_ZAXIS_SAVE_HEAD + P_ZAXIS_SAVE_LEN * P_XAXIS_SAVE_NUM)
#define   P_LAXIS_SAVE_LEN     P_XAXIS_SAVE_LEN
#define   P_OAXIS_SAVE_HEAD    (P_LAXIS_SAVE_HEAD + P_LAXIS_SAVE_LEN * P_XAXIS_SAVE_NUM)
#define   P_OAXIS_SAVE_LEN     P_XAXIS_SAVE_LEN


/**************************�ѿ�������ϵ�����洢*******************************/
#define   P_CARTESIAN_PARA_HEAD    0x7330																				//�ѿ�������ϵ�����洢-��ʼ��ַ
#define   P_CARTESIAN_PARA_LEN     0x50																					//��������-Ԥ��80�ֽ�

/**************************�䷽�����洢**************************************/
#define   P_FORMULATION_PARA_HEAD    0x7390																			//�䷽�����洢-��ʼ��ַ
#define   P_FORMULATION_PARA_LEN     0x20																				//��������-Ԥ��32�ֽ�

/**************************������Ʋ����洢**********************************/
#define   P_MOTORCONTROL_PARA_HEAD    0x73C0																		//������Ʋ����洢-��ʼ��ַ
#define   P_MOTORCONTROL_PARA_LEN     0x40																			//��������-Ԥ��64�ֽ�

///**************************�����������洢************************************/
//#define   P_INTERNET_ADDRESS         0x7410  															//���������к���ʼ��ַ�����������кŲ���Ҫ����
//#define   P_INTERNET_PARA_LEN        0x40																	//��������-Ԥ��64�ֽ�

/**************************��չ������洢************************************/
#define   P_EXTENDAIX_ADDRESS        0x7460  																		//������չ��
#define   P_EXTENDAIX_PARA_SIZE      0x30																				//ÿ����չ������������ݴ�С48�ֽ�

/**************************����ֵ���λ�ô洢********************************/
#define   P_SERVO_JDZ_ZERO_HEAD    0x7FE0																				//����ֵ���λ�ô洢-��ʼ��ַ
#define   P_SERVO_JDZ_ZERO_LEN     0x20																					//��������-Ԥ��16�ֽ�

/**************************�����ز����洢*********************************/
#define   P_MD_PARA_HEAD    0x30000																				//�������洢-��ʼ��ַ
#define   P_MD_PARA_LEN     0x40																					//��������-Ԥ��64λ
#define   P_MD_POINT_HEAD  	(P_MD_PARA_HEAD + P_MD_PARA_LEN)							//��Ʒ1����1-��ʼ��ַ
#define   P_MD_POINT_LEN    0x40																					//һ����洢����-Ԥ��64λ��һ����Ʒÿ��64���㣬���ѭ������Ϊ4
#define   P_MD_POINT_TOTLE  (P_MD_POINT_LEN * MD_POINT_NUM * LOOP_MAX)		//һ����Ʒ���е���ܳ��ȣ�һ����Ʒÿ��64���㣬���ѭ������Ϊ4

#define   P_MD_GOOD_LEN     (P_MD_PARA_LEN + P_MD_POINT_TOTLE)						//һ����Ʒ�����ܳ���

#define   P_MD_PARA_END    	(P_MD_PARA_HEAD	+ P_MD_GOOD_LEN * MD_GOOD_NUM)	//�������洢-������ַ:0x30000 + 0x12C000 = 0x15C000

#define   P_MD_CLEAR_NUM    (((P_MD_PARA_END - P_MD_PARA_HEAD) / 4096) + 1)		//�������洢-����ҳ��300

/**************************����ʱ�ɱ�����Ĵ洢��*************************************/

#define   P_SC_NUM_ADDRESS				  0xFFF000															//�����������ۻ�������NG�������洢�����һ������,ռ��0xFFF000-0xFFF01E

#define   P_USER_ADDRESS				    0xFFF070															//�û�����,ռ��0xFFF070-0xFFF128

#define   P_MD_SORT_ADDRESS				  0xFFF200															//�洢���ּ�ģʽ�µ�ÿ����Ʒ�ĵ�ǰ��͸���,ռ��MD_GOOD_NUM*2=60*2=0x78

#define   P_INTERNET_ADDRESS        0xFFF300  														//���������к���ʼ��ַ�����������кŲ���Ҫ����
#define   P_INTERNET_PARA_LEN       0x40																	//��������������-Ԥ��64�ֽ�


/*����ʱ�ɱ�����Ĵ洢��*/
#define SAVE_PROTECT_SIZE				  					0x1000						//��������Ĵ�СΪ4K
#define SAVE_PROTECT_START_ADDRESS				  0xFFF000					//�����������ʼ��ַ
#define SAVE_PROTECT_END_ADDRESS				  	(SAVE_PROTECT_START_ADDRESS + SAVE_PROTECT_SIZE - 1)		//��������Ľ�����ַ
#define SAVE_PROTECT_BACKUP_ADDRESS					0xFFE000					//�������򱸷�������ʼ��ַ
#define SAVE_PROTECT_BACKUP_END_ADDRESS			(SAVE_PROTECT_BACKUP_ADDRESS + SAVE_PROTECT_SIZE - 1)		//�������򱸷����Ľ�����ַ
#define SAVE_PROTECT_VALUE									(0x55)						//�����������һ����ַ��У���ֽڵĹ̶���ֵ��������֤�Ƿ�дʧ��

/**-------0x1E-----��������-----------------**/
#define	  P_PARAMETER_SOFT_LIMIT	   		0x01			//����λ
#define   P_PARAMETER_SAFE_AREA        	0x02			//��ȫ��
#define   P_PARAMETER_FUCTION_SET      	0x03			//���ܲ���
#define	  P_PARAMETER_IO_DETECT_SET	   	0X04 			//�������
#define	  P_PARAMETER_OUTPUT_SET	    	0X05 			//���IO��λѡ��
#define	  P_PARAMETER_IO_INSWITCH_SET	  0X06 			//���븴�ÿ���
#define	  P_PARAMETER_OUT_INSWITCH_SET	0X07 			//������ÿ���
#define   P_PARAMETER_MD_PARA      			0x08			//������
#define   P_PARAMETER_MD_POINT      		0x09			//�������
#define   P_PARAMETER_MD_DELETE      		0x0A			//������-�ָ�����
#define   P_PARAMETER_CARTESIAN_PARA   	0x0B			//�ѿ�������ϵ����

extern void WatchFunction(void);
extern void OrderDecoding(void);
extern u32 Program_RunTime;

#endif

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team *****END OF FILE****/

