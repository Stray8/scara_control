/******************** (C) COPYRIGHT 2011 zjutǶ��ʽ������  ********************
* File Name          : out.h
* Author             :  
* Version            :  
* Date               :  
* Description        : This file 
*******************************************************************************/
#ifndef __out_h_
#define __out_h_

/*����ڵĵ�ƽ����*/
#define Y0_OUT_PORT_SET			{(Output_Status[0] = (Output_Status[0]|0x01)); (GPIO_SetBits(GPIOC, GPIO_Pin_15));}	//Y0��λ
#define Y1_OUT_PORT_SET			{(Output_Status[0] = (Output_Status[0]|0x02)); (GPIO_SetBits(GPIOC, GPIO_Pin_14));}	//Y1��λ
#define Y2_OUT_PORT_SET			{(Output_Status[0] = (Output_Status[0]|0x04)); (GPIO_SetBits(GPIOC, GPIO_Pin_13));}	//Y2��λ
#define Y3_OUT_PORT_SET			{(Output_Status[0] = (Output_Status[0]|0x08)); (GPIO_SetBits(GPIOE, GPIO_Pin_6));}	//Y3��λ
#define Y4_OUT_PORT_SET			{(Output_Status[0] = (Output_Status[0]|0x10)); (GPIO_SetBits(GPIOE, GPIO_Pin_5));}	//Y4��λ
#define Y5_OUT_PORT_SET			{(Output_Status[0] = (Output_Status[0]|0x20)); (GPIO_SetBits(GPIOE, GPIO_Pin_4));}	//Y5��λ
#define Y6_OUT_PORT_SET			{(Output_Status[0] = (Output_Status[0]|0x40)); (GPIO_SetBits(GPIOE, GPIO_Pin_3));}	//Y6��λ
#define Y7_OUT_PORT_SET			{(Output_Status[0] = (Output_Status[0]|0x80)); (GPIO_SetBits(GPIOE, GPIO_Pin_1));}	//Y7��λ
#define Y8_OUT_PORT_SET			{(Output_Status[1] = (Output_Status[1]|0x01)); (GPIO_SetBits(GPIOE, GPIO_Pin_0));}	//Y8��λ
#define Y9_OUT_PORT_SET			{(Output_Status[1] = (Output_Status[1]|0x02)); (GPIO_SetBits(GPIOB, GPIO_Pin_5));}	//Y9��λ
#define Y10_OUT_PORT_SET		{(Output_Status[1] = (Output_Status[1]|0x04)); (GPIO_SetBits(GPIOB, GPIO_Pin_4));}	//Y10��λ
#define Y11_OUT_PORT_SET		{(Output_Status[1] = (Output_Status[1]|0x08)); (GPIO_SetBits(GPIOB, GPIO_Pin_3));}	//Y11��λ
#define Y12_OUT_PORT_SET		{(Output_Status[1] = (Output_Status[1]|0x10)); (GPIO_SetBits(GPIOD, GPIO_Pin_7));}	//Y12��λ
#define Y13_OUT_PORT_SET		{(Output_Status[1] = (Output_Status[1]|0x20)); (GPIO_SetBits(GPIOD, GPIO_Pin_3));}	//Y13��λ
#define Y14_OUT_PORT_SET		{(Output_Status[1] = (Output_Status[1]|0x40)); (GPIO_SetBits(GPIOD, GPIO_Pin_2));}	//Y14��λ
#define Y15_OUT_PORT_SET	 	{(Output_Status[1] = (Output_Status[1]|0x80)); (GPIO_SetBits(GPIOC, GPIO_Pin_8));}	//Y15��λ
#define Y16_OUT_PORT_SET   	{(Output_Status[2] = (Output_Status[2]|0x01)); (GPIO_SetBits(GPIOC, GPIO_Pin_7));}	//Y16��λ
#define Y17_OUT_PORT_SET	  {(Output_Status[2] = (Output_Status[2]|0x02)); (GPIO_SetBits(GPIOC, GPIO_Pin_6));}	//Y17��λ

#define Y0_OUT_PORT_RESET			{(Output_Status[0] = (Output_Status[0]&0xfe)); (GPIO_ResetBits(GPIOC, GPIO_Pin_15));}	//Y0��λ
#define Y1_OUT_PORT_RESET			{(Output_Status[0] = (Output_Status[0]&0xfd)); (GPIO_ResetBits(GPIOC, GPIO_Pin_14));}	//Y1��λ
#define Y2_OUT_PORT_RESET			{(Output_Status[0] = (Output_Status[0]&0xfb)); (GPIO_ResetBits(GPIOC, GPIO_Pin_13));}	//Y2��λ
#define Y3_OUT_PORT_RESET			{(Output_Status[0] = (Output_Status[0]&0xf7)); (GPIO_ResetBits(GPIOE, GPIO_Pin_6));}	//Y3��λ
#define Y4_OUT_PORT_RESET			{(Output_Status[0] = (Output_Status[0]&0xef)); (GPIO_ResetBits(GPIOE, GPIO_Pin_5));}	//Y4��λ
#define Y5_OUT_PORT_RESET			{(Output_Status[0] = (Output_Status[0]&0xdf)); (GPIO_ResetBits(GPIOE, GPIO_Pin_4));}	//Y5��λ
#define Y6_OUT_PORT_RESET			{(Output_Status[0] = (Output_Status[0]&0xbf)); (GPIO_ResetBits(GPIOE, GPIO_Pin_3));}	//Y6��λ
#define Y7_OUT_PORT_RESET			{(Output_Status[0] = (Output_Status[0]&0x7f)); (GPIO_ResetBits(GPIOE, GPIO_Pin_1));}	//Y7��λ
#define Y8_OUT_PORT_RESET			{(Output_Status[1] = (Output_Status[1]&0xfe)); (GPIO_ResetBits(GPIOE, GPIO_Pin_0));}	//Y8��λ
#define Y9_OUT_PORT_RESET			{(Output_Status[1] = (Output_Status[1]&0xfd)); (GPIO_ResetBits(GPIOB, GPIO_Pin_5));}	//Y9��λ
#define Y10_OUT_PORT_RESET		{(Output_Status[1] = (Output_Status[1]&0xfb)); (GPIO_ResetBits(GPIOB, GPIO_Pin_4));}	//Y10��λ
#define Y11_OUT_PORT_RESET		{(Output_Status[1] = (Output_Status[1]&0xf7)); (GPIO_ResetBits(GPIOB, GPIO_Pin_3));}	//Y11��λ
#define Y12_OUT_PORT_RESET		{(Output_Status[1] = (Output_Status[1]&0xef)); (GPIO_ResetBits(GPIOD, GPIO_Pin_7));}	//Y12��λ
#define Y13_OUT_PORT_RESET		{(Output_Status[1] = (Output_Status[1]&0xdf)); (GPIO_ResetBits(GPIOD, GPIO_Pin_3));}	//Y13��λ
#define Y14_OUT_PORT_RESET		{(Output_Status[1] = (Output_Status[1]&0xbf)); (GPIO_ResetBits(GPIOD, GPIO_Pin_2));}	//Y14��λ
#define Y15_OUT_PORT_RESET  	{(Output_Status[1] = (Output_Status[1]&0x7f)); (GPIO_ResetBits(GPIOC, GPIO_Pin_8));}	//Y15��λ
#define Y16_OUT_PORT_RESET 		{(Output_Status[2] = (Output_Status[2]&0xfe)); (GPIO_ResetBits(GPIOC, GPIO_Pin_7));}	//Y16��λ
#define Y17_OUT_PORT_RESET 		{(Output_Status[2] = (Output_Status[2]&0xfd)); (GPIO_ResetBits(GPIOC, GPIO_Pin_6));}	//Y17��λ

/**-------0x1A-----IO����-���1--------------**/
#define   O_IODEBUG_OUTPUT_0       0	//
#define   O_IODEBUG_OUTPUT_1       1	//
#define   O_IODEBUG_OUTPUT_2       2	//
#define   O_IODEBUG_OUTPUT_3       3	//
#define   O_IODEBUG_OUTPUT_4       4	//
#define   O_IODEBUG_OUTPUT_5       5	//
#define   O_IODEBUG_OUTPUT_6       6	//
#define   O_IODEBUG_OUTPUT_7       7	//
#define   O_IODEBUG_OUTPUT_8       8	//
#define   O_IODEBUG_OUTPUT_9       9	//
#define   O_IODEBUG_OUTPUT_10      10	//
#define   O_IODEBUG_OUTPUT_11      11	//
#define   O_IODEBUG_OUTPUT_12      12	//
#define   O_IODEBUG_OUTPUT_13      13	//
#define   O_IODEBUG_OUTPUT_14      14	//
#define   O_IODEBUG_OUTPUT_15      15	//
#define   O_IODEBUG_OUTPUT_16      16	//
#define   O_IODEBUG_OUTPUT_17      17	//
#define   O_IODEBUG_OUTPUT_18      18	//
#define   O_IODEBUG_OUTPUT_19      19	//
#define   O_IODEBUG_OUTPUT_20      20	//
#define   O_IODEBUG_OUTPUT_21      21	//
#define   O_IODEBUG_OUTPUT_22      22	//
#define   O_IODEBUG_OUTPUT_23      23	//
#define   O_IODEBUG_OUTPUT_24      24	//
#define   O_IODEBUG_OUTPUT_25      25	//
#define   O_IODEBUG_OUTPUT_26      26	//
#define   O_IODEBUG_OUTPUT_27      27	//
#define   O_IODEBUG_OUTPUT_28      28	//
#define   O_IODEBUG_OUTPUT_29      29	//
#define   O_IODEBUG_OUTPUT_30      30	//
#define   O_IODEBUG_OUTPUT_31      31	//
#define   O_IODEBUG_OUTPUT_32      32	//
#define   O_IODEBUG_OUTPUT_33      33	//
#define   O_IODEBUG_OUTPUT_34      34	//
#define   O_IODEBUG_OUTPUT_35      35	//
#define   O_IODEBUG_OUTPUT_36      36	//
#define   O_IODEBUG_OUTPUT_37      37	//
#define   O_IODEBUG_OUTPUT_38      38	//
#define   O_IODEBUG_OUTPUT_39      39	//

/**-----------����ض���--------------**/
#define O_RUN_GREEN_LIGHT     	O_IODEBUG_OUTPUT_14		//���е�
#define O_WAIT_YELLOW_LIGHT   	O_IODEBUG_OUTPUT_15		//������
#define O_ALARM_RED_LIGHT     	O_IODEBUG_OUTPUT_16		//������
#define O_ALARM_BUZZER					O_IODEBUG_OUTPUT_17		//������

#define O_ORIGINED_LIGHT     O_IODEBUG_OUTPUT_12		//ԭ��״̬
#define O_RESETING_LIGHT     O_IODEBUG_OUTPUT_13		//��λ״̬


/*------------�����������ź�----------------*/
#define X_AXSIS_DIRECTION          40
#define Z_AXSIS_DIRECTION					 41
#define L_AXSIS_DIRECTION          42
#define O_AXSIS_DIRECTION					 43

extern u8 Output_Status[5];							//���ڱ������״̬
extern void OutputInit(void);
extern void SetOutput(u8 IO_Num);
extern void ResetOutput(u8 IO_Num);

#endif

/******************* (C) COPYRIGHT 2011 zjutǶ��ʽ������ *****END OF FILE****/
