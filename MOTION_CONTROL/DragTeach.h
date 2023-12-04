#ifndef __DRAGTEACH_H_
#define __DRAGTEACH_H_

#include "JDZ.h"
#include "CANopen.h"
#include "EtherCAT_App.h"
#include "Parameter.h"

#define TEACH_AXIS_NUM  1
#define ZERO_ERR  1
#define ZERO_CHANGE_CYCLE  2  //�ٶ�Ϊ0ʱ�����л����� ��λΪ10ms
#define ZERO_CHANGE_COE  0.1  //�ٶ�Ϊ0ʱ�����л����� ��λΪ10ms
#define AXIS_01_TORQUE_START  55   //%0.1
#define AXIS_02_TORQUE_START  30







void TorqueProcess();
void TorqueProcess_T();






#endif