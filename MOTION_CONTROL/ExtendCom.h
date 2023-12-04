/********************************* START OF FILE ***************************************
* File Name          : ExtendCom.c
* Author             : 
* Version            : 
* Date               : 
* Description        : This is the ....
***************************************************************************************/

#ifndef __ExtendCom_h_
#define __ExtendCom_h_

#include "stm32f4xx.h"

/*����Ӧ�ó����д������¼�������*/
extern u16 ExtendProgramNum;							//ͨ��ͨ��ѡ�еĳ����
extern u8  ExtendEmergencyStop;				    //ͨ����ͣ�豸��־
extern u8  ExtendYieldChange;							//ͨ��ͨ���޸Ĳ�����ر�����־
extern u8  ExtendPosChange;								//ͨ��ͨ���޸�������ر�����־
extern u8  ExtendPosChangeNum;						//ͨ��ͨ���޸�����ı��
extern u8  ExtendStateChange;							//ͨ��ͨ���޸�����ı��
extern u8  ExtendCancleAlarm;
extern u8  ExtendSerialNum;								//ͨ��ͨ����¼���к�

#define DIR_485_H  {GPIO_SetBits(GPIOD, GPIO_Pin_1);delay_ms(5);}
#define DIR_485_L  {GPIO_ResetBits(GPIOD, GPIO_Pin_1);}

/*�㲥����ʱ��ID*/
#define EXTEND_RADIO_ID									0x00        //485��չ���ܵĹ㲥ID

/*��չģ��Ľ������ݱ�־*/
#define EXTEND_WAIT_REC									0x00        //������
#define EXTEND_START_REC								0x01        //��ʼ����
#define EXTEND_END_REC									0x02        //��ɽ���

/*��չģ�鹦���붨��*/
#define EXTEND_FUN_READ									0x03        //������-����Ĵ�����ȡ
#define EXTEND_FUN_WRITE								0x10        //д����-��Ĵ���д��

/*485���ʵ�ַ*/
//ϵͳ��
#define EXT_ADDR_SERIAL_NUM							0x0000      //���кŵ�ַ���ɶ�����д
#define EXT_ADDR_SYSTEM_VER							0x0006      //ϵͳ�汾�ŵ�ַ���ɶ�����д
#define EXT_ADDR_PRODUC_FAC							0x000C      //�������ҵ�ַ���ɶ�����д
#define EXT_ADDR_WRITE_SERIAL_NUM			  0x0012      //�����豸���кŵ�ַ�����ɶ���д

//ʱ����
#define EXT_ADDR_ONCE_T									0x0100      //��ǰ��Ʒ���μӹ�ʱ���ַ���ɶ�����д
#define EXT_ADDR_POWER_T								0x0102      //����ʱ���ַ���ɶ�����д
#define EXT_ADDR_RUN_T									0x0104      //����ʱ���ַ���ɶ�����д
#define EXT_ADDR_TOTAL_POWER_T					0x0106      //�ܿ���ʱ���ַ���ɶ�����д
#define EXT_ADDR_TOTAL_RUN_T						0x0108      //������ʱ���ַ���ɶ�����д

//״̬��
#define EXT_ADDR_RUN_STA								0x0200      //��ǰ����״̬��ַ���ɶ�����д
#define EXT_ADDR_RESET_STA							0x0201      //��ǰ��λ״̬��ַ���ɶ�����д
#define EXT_ADDR_BACK_ORI_STA						0x0202      //��ǰ����״̬��ַ���ɶ�����д
#define EXT_ADDR_ALARM_INFO							0x0203      //��ǰ�豸������ַ���ɶ�����д

//���в���
#define EXT_ADDR_CUR_RUN_PRO						0x0300      //��ǰ���г����ַ���ɶ�����д
#define EXT_ADDR_CUR_YIELD							0x0306      //��ǰ������ַ���ɶ�����д
#define EXT_ADDR_TAR_YIELD							0x0308      //Ŀ�������ַ���ɶ�����д
#define EXT_ADDR_TOTAL_YIELD						0x030A      //�ܲ�����ַ���ɶ��ɲ�д
#define EXT_ADDR_NG_YIELD								0x030C      //��Ʒ������ַ���ɶ�����д
#define EXT_ADDR_CHK_YIELD							0x030E      //��������ַ���ɶ���д

//������
//#define EXT_ADDR_RUN_STOP								0x0600      //�����豸��ͣ��ַ�����ɶ���д
#define EXT_ADDR_INPUT_STA							0x0400      //��ǰ�豸�����ַ���ɶ�����д 0x0400~0x04FF,Ԥ��256·
#define EXT_ADDR_OUT_CONTROL						0x0500      //�����豸�����ַ�����ɶ���д 0x0500~0x05FF,Ԥ��256·

/*���豸��ǰλ�õ�ַ*/
#define EXT_ADDR_CUR_POSITION_X					0x1000      //��ǰλ��X��ַ���ɶ�����д
#define EXT_ADDR_CUR_POSITION_Y					0x1002      //��ǰλ��Y��ַ���ɶ�����д
#define EXT_ADDR_CUR_POSITION_Z					0x1004      //��ǰλ��Z��ַ���ɶ�����д
#define EXT_ADDR_CUR_POSITION_O					0x1006      //��ǰλ��O��ַ���ɶ�����д

//�����豸״̬��ַ
#define EXT_ADDR_BACK_ORI								0x1100      //�豸�����ַ�����ɶ���д
#define EXT_ADDR_RESET									0x1101      //�豸��λ��ַ�����ɶ���д
#define EXT_ADDR_RUN_AUTO								0x1102      //�豸�������е�ַ�����ɶ���д
#define EXT_ADDR_STOP_RUN								0x1103      //�豸ֹͣ���е�ַ�����ɶ���д
#define EXT_ADDR_PAUSE_RUN							0x1104      //�豸��ͣ���е�ַ�����ɶ���д
#define EXT_ADDR_EM_STOP								0x1105      //�豸��ͣ��ַ�����ɶ���д
#define EXT_ADDR_CANCEL_ALARM						0x1106      //ȡ���豸���������ɶ���д

/*��������Ϣ���ʵ�ַ����Χ0x2000~0x29FF��ÿ����ռ��0x40����ַ*/
#define EXT_ADDR_POINT_1_X							0x2000      //����1��X��ַ���ɶ���д
#define EXT_ADDR_POINT_1_Y							0x2002      //����1��Y��ַ���ɶ���д
#define EXT_ADDR_POINT_1_Z							0x2004      //����1��Z��ַ���ɶ���д
#define EXT_ADDR_POINT_1_O							0x2006      //����1��O��ַ���ɶ���д
#define EXT_ADDR_POINT_2_X							0x2040      //����2��X��ַ���ɶ���д
#define EXT_ADDR_POINT_2_Y							0x2042      //����2��Y��ַ���ɶ���д
#define EXT_ADDR_POINT_2_Z							0x2044      //����2��Z��ַ���ɶ���д
#define EXT_ADDR_POINT_2_O							0x2046      //����2��O��ַ���ɶ���д

/**-------�ⲿ485ͨ�Ŷ�ȡ��е��״̬--------------**/
#define	EXT_OLINE_NO						        0x0000	//��������������
#define	EXT_OLINE_YES						        0x0001	//��������������
#define	EXT_RUNNING							        0x0002	//������
#define	EXT_PAUSE								        0x0003	//��ͣ��
#define	EXT_ERR									        0x0004	//����
#define	EXT_RESET								        0x0005	//��λ��
#define	EXT_BACK								        0x0006	//������

/**-------�ⲿ485ͨ�ſ��ƻ�е��״̬--------------**/
#define	EXTEND_RESET			              0x01	//��е�ָ�λ
#define	EXTEND_ORIGIN			              0x02	//����
#define	EXTEND_RUN				              0x03	//����
#define	EXTEND_STOP				              0x04	//ֹͣ
#define	EXTEND_PAUSE			              0x05	//��ͣ


/*******************************************************************************
* Function Name  : ExtendRecDataDeal
* Description    : ������չģ��485���յ������ݣ�����ѭ���е��øú���
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
extern void ExtendRecDataDeal(void);

#endif

/************************************END OF FILE**************************************/
