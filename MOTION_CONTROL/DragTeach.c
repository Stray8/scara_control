#include "DragTeach.h"

double K[2] = {0.532,0.232};  //摩擦力斜率
double B[2] = {5,5};	//摩擦力截距

s32 Vel = 0;

void TorqueProcess()
{
	int i = 0;
	s16 tempTorque = 0;
	s32 tempVel = 0;
	
	
	
	for(i=1; i<TEACH_AXIS_NUM; i++)
	{
		
		tempVel = ServoMoveCurSpeed(i);
		if (tempVel != ServoMoveCurSpeed(i) )
		{
			if(tempVel > 0)
				tempTorque = ( K[i]*tempVel + B[i] ) * 10; //单位为0.1%
			else
				tempTorque = ( K[i]*tempVel - B[i] ) * 10;
			
			ServoCST_PDO(i,tempTorque);
		}
		
	}
	

}

void TorqueProcess_T()
{
	int i = 0;
	s16 tempTorque = 0;
	s32 tempVel = 0;
	
	
	
	for(i = 1; i<= TEACH_AXIS_NUM; i++){	
		tempVel = ServoMoveCurSpeed(i);
		
		if (1){			//(tempVel != ServoMoveCurSpeed(i) )
			if(tempVel > ZERO_ERR){
				tempTorque = AXIS_01_TORQUE_START; //单位为0.1%
			}
			else if(tempVel <= ZERO_ERR && tempVel >= 0){
				tempTorque = AXIS_01_TORQUE_START * tempVel * 0.5; //单位为0.1%
			}
			else if (tempVel <= 0 && tempVel >= (-1) * ZERO_ERR){
				tempTorque = (-1) * AXIS_01_TORQUE_START * tempVel * 0.5; //单位为0.1%
			}
			else
				tempTorque = AXIS_01_TORQUE_START*(-1);
			
			ServoCST_PDO(i,tempTorque);
		}
	}
}

// void TorqueProcess_T()
// {
// 	int i = 0;
// 	s16 tempTorque = 0;
// 	s32 tempVel = 0;
	
// 	for(i = 1; i<= TEACH_AXIS_NUM; i++){	
// 		tempVel = ServoMoveCurSpeed(i);

// 		if (1){			//(tempVel != ServoMoveCurSpeed(i) )
// 			if(tempVel > ZERO_ERR){
// 				tempTorque = AXIS_01_TORQUE_START; //单位为0.1%
// 			}
// 			else if(tempVel <= ZERO_ERR && tempVel >= (-1*ZERO_ERR)){
// 				if(Torque_T_count%(2*ZERO_CHANGE_CYCLE) <= ZERO_CHANGE_CYCLE)
// 					tempTorque = AXIS_01_TORQUE_START*ZERO_CHANGE_COE;
// 				else
// 					tempTorque = AXIS_01_TORQUE_START*(-1)*ZERO_CHANGE_COE;
// 			}
// 			else
// 				tempTorque = AXIS_01_TORQUE_START*(-1);
			
// 			ServoCST_PDO(i,tempTorque);
// 		}
// 	}
// }
