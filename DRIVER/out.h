/******************** (C) COPYRIGHT 2011 zjut嵌入式开发组  ********************
* File Name          : out.h
* Author             :  
* Version            :  
* Date               :  
* Description        : This file 
*******************************************************************************/
#ifndef __out_h_
#define __out_h_

/*输出口的电平设置*/
#define Y0_OUT_PORT_SET			{(Output_Status[0] = (Output_Status[0]|0x01)); (GPIO_SetBits(GPIOC, GPIO_Pin_15));}	//Y0置位
#define Y1_OUT_PORT_SET			{(Output_Status[0] = (Output_Status[0]|0x02)); (GPIO_SetBits(GPIOC, GPIO_Pin_14));}	//Y1置位
#define Y2_OUT_PORT_SET			{(Output_Status[0] = (Output_Status[0]|0x04)); (GPIO_SetBits(GPIOC, GPIO_Pin_13));}	//Y2置位
#define Y3_OUT_PORT_SET			{(Output_Status[0] = (Output_Status[0]|0x08)); (GPIO_SetBits(GPIOE, GPIO_Pin_6));}	//Y3置位
#define Y4_OUT_PORT_SET			{(Output_Status[0] = (Output_Status[0]|0x10)); (GPIO_SetBits(GPIOE, GPIO_Pin_5));}	//Y4置位
#define Y5_OUT_PORT_SET			{(Output_Status[0] = (Output_Status[0]|0x20)); (GPIO_SetBits(GPIOE, GPIO_Pin_4));}	//Y5置位
#define Y6_OUT_PORT_SET			{(Output_Status[0] = (Output_Status[0]|0x40)); (GPIO_SetBits(GPIOE, GPIO_Pin_3));}	//Y6置位
#define Y7_OUT_PORT_SET			{(Output_Status[0] = (Output_Status[0]|0x80)); (GPIO_SetBits(GPIOE, GPIO_Pin_1));}	//Y7置位
#define Y8_OUT_PORT_SET			{(Output_Status[1] = (Output_Status[1]|0x01)); (GPIO_SetBits(GPIOE, GPIO_Pin_0));}	//Y8置位
#define Y9_OUT_PORT_SET			{(Output_Status[1] = (Output_Status[1]|0x02)); (GPIO_SetBits(GPIOB, GPIO_Pin_5));}	//Y9置位
#define Y10_OUT_PORT_SET		{(Output_Status[1] = (Output_Status[1]|0x04)); (GPIO_SetBits(GPIOB, GPIO_Pin_4));}	//Y10置位
#define Y11_OUT_PORT_SET		{(Output_Status[1] = (Output_Status[1]|0x08)); (GPIO_SetBits(GPIOB, GPIO_Pin_3));}	//Y11置位
#define Y12_OUT_PORT_SET		{(Output_Status[1] = (Output_Status[1]|0x10)); (GPIO_SetBits(GPIOD, GPIO_Pin_7));}	//Y12置位
#define Y13_OUT_PORT_SET		{(Output_Status[1] = (Output_Status[1]|0x20)); (GPIO_SetBits(GPIOD, GPIO_Pin_3));}	//Y13置位
#define Y14_OUT_PORT_SET		{(Output_Status[1] = (Output_Status[1]|0x40)); (GPIO_SetBits(GPIOD, GPIO_Pin_2));}	//Y14置位
#define Y15_OUT_PORT_SET	 	{(Output_Status[1] = (Output_Status[1]|0x80)); (GPIO_SetBits(GPIOC, GPIO_Pin_8));}	//Y15置位
#define Y16_OUT_PORT_SET   	{(Output_Status[2] = (Output_Status[2]|0x01)); (GPIO_SetBits(GPIOC, GPIO_Pin_7));}	//Y16置位
#define Y17_OUT_PORT_SET	  {(Output_Status[2] = (Output_Status[2]|0x02)); (GPIO_SetBits(GPIOC, GPIO_Pin_6));}	//Y17置位

#define Y0_OUT_PORT_RESET			{(Output_Status[0] = (Output_Status[0]&0xfe)); (GPIO_ResetBits(GPIOC, GPIO_Pin_15));}	//Y0复位
#define Y1_OUT_PORT_RESET			{(Output_Status[0] = (Output_Status[0]&0xfd)); (GPIO_ResetBits(GPIOC, GPIO_Pin_14));}	//Y1复位
#define Y2_OUT_PORT_RESET			{(Output_Status[0] = (Output_Status[0]&0xfb)); (GPIO_ResetBits(GPIOC, GPIO_Pin_13));}	//Y2复位
#define Y3_OUT_PORT_RESET			{(Output_Status[0] = (Output_Status[0]&0xf7)); (GPIO_ResetBits(GPIOE, GPIO_Pin_6));}	//Y3复位
#define Y4_OUT_PORT_RESET			{(Output_Status[0] = (Output_Status[0]&0xef)); (GPIO_ResetBits(GPIOE, GPIO_Pin_5));}	//Y4复位
#define Y5_OUT_PORT_RESET			{(Output_Status[0] = (Output_Status[0]&0xdf)); (GPIO_ResetBits(GPIOE, GPIO_Pin_4));}	//Y5复位
#define Y6_OUT_PORT_RESET			{(Output_Status[0] = (Output_Status[0]&0xbf)); (GPIO_ResetBits(GPIOE, GPIO_Pin_3));}	//Y6复位
#define Y7_OUT_PORT_RESET			{(Output_Status[0] = (Output_Status[0]&0x7f)); (GPIO_ResetBits(GPIOE, GPIO_Pin_1));}	//Y7复位
#define Y8_OUT_PORT_RESET			{(Output_Status[1] = (Output_Status[1]&0xfe)); (GPIO_ResetBits(GPIOE, GPIO_Pin_0));}	//Y8复位
#define Y9_OUT_PORT_RESET			{(Output_Status[1] = (Output_Status[1]&0xfd)); (GPIO_ResetBits(GPIOB, GPIO_Pin_5));}	//Y9复位
#define Y10_OUT_PORT_RESET		{(Output_Status[1] = (Output_Status[1]&0xfb)); (GPIO_ResetBits(GPIOB, GPIO_Pin_4));}	//Y10复位
#define Y11_OUT_PORT_RESET		{(Output_Status[1] = (Output_Status[1]&0xf7)); (GPIO_ResetBits(GPIOB, GPIO_Pin_3));}	//Y11复位
#define Y12_OUT_PORT_RESET		{(Output_Status[1] = (Output_Status[1]&0xef)); (GPIO_ResetBits(GPIOD, GPIO_Pin_7));}	//Y12复位
#define Y13_OUT_PORT_RESET		{(Output_Status[1] = (Output_Status[1]&0xdf)); (GPIO_ResetBits(GPIOD, GPIO_Pin_3));}	//Y13复位
#define Y14_OUT_PORT_RESET		{(Output_Status[1] = (Output_Status[1]&0xbf)); (GPIO_ResetBits(GPIOD, GPIO_Pin_2));}	//Y14复位
#define Y15_OUT_PORT_RESET  	{(Output_Status[1] = (Output_Status[1]&0x7f)); (GPIO_ResetBits(GPIOC, GPIO_Pin_8));}	//Y15复位
#define Y16_OUT_PORT_RESET 		{(Output_Status[2] = (Output_Status[2]&0xfe)); (GPIO_ResetBits(GPIOC, GPIO_Pin_7));}	//Y16复位
#define Y17_OUT_PORT_RESET 		{(Output_Status[2] = (Output_Status[2]&0xfd)); (GPIO_ResetBits(GPIOC, GPIO_Pin_6));}	//Y17复位

/**-------0x1A-----IO调试-输出1--------------**/
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

/**-----------输出重定义--------------**/
#define O_RUN_GREEN_LIGHT     	O_IODEBUG_OUTPUT_14		//运行灯
#define O_WAIT_YELLOW_LIGHT   	O_IODEBUG_OUTPUT_15		//待机灯
#define O_ALARM_RED_LIGHT     	O_IODEBUG_OUTPUT_16		//报警灯
#define O_ALARM_BUZZER					O_IODEBUG_OUTPUT_17		//蜂鸣器

#define O_ORIGINED_LIGHT     O_IODEBUG_OUTPUT_12		//原点状态
#define O_RESETING_LIGHT     O_IODEBUG_OUTPUT_13		//复位状态


/*------------电机方向控制信号----------------*/
#define X_AXSIS_DIRECTION          40
#define Z_AXSIS_DIRECTION					 41
#define L_AXSIS_DIRECTION          42
#define O_AXSIS_DIRECTION					 43

extern u8 Output_Status[5];							//用于保存输出状态
extern void OutputInit(void);
extern void SetOutput(u8 IO_Num);
extern void ResetOutput(u8 IO_Num);

#endif

/******************* (C) COPYRIGHT 2011 zjut嵌入式开发组 *****END OF FILE****/
