#include "Function.h"
#include "main.h"

#include "string.h"
#include "stdio.h"
#include <stdlib.h>
#include "stdbool.h"
#include <math.h>
//#include <FLASH_SECTOR_F4.h>
extern bool POSReach;
void Stop() // Stop motor function
{
		HAL_GPIO_WritePin(Stop_PC5_43_GPIO_Port,Stop_PC5_43_Pin,GPIO_PIN_SET);//(STOP) turn on pin 43
		HAL_GPIO_WritePin(SerVoReset_PC4_18_GPIO_Port, SerVoReset_PC4_18_Pin, GPIO_PIN_SET); // Disable Servo Enable
}

void AlarmReset() // reset alarm function
{
	  	HAL_GPIO_WritePin(EStop_Not_PB0_17_GPIO_Port, EStop_Not_PB0_17_Pin, GPIO_PIN_SET);//Pull Estop pin to 1 (24V)

		HAL_GPIO_WritePin(ArlarmRST_PB1_42_GPIO_Port, ArlarmRST_PB1_42_Pin, GPIO_PIN_RESET); // trig Alarm Reset Pin
		HAL_Delay(500);
		HAL_GPIO_WritePin(ArlarmRST_PB1_42_GPIO_Port, ArlarmRST_PB1_42_Pin, GPIO_PIN_SET);
	
		HAL_GPIO_WritePin(Stop_PC5_43_GPIO_Port,Stop_PC5_43_Pin,GPIO_PIN_SET);//(STOP) turn on pin 43
		HAL_GPIO_WritePin(SerVoReset_PC4_18_GPIO_Port, SerVoReset_PC4_18_Pin, GPIO_PIN_RESET); // Enable Servo Enable 
		// Turn on the brake
}

void Estop() // Estop function
{
		HAL_GPIO_WritePin(EStop_Not_PB0_17_GPIO_Port, EStop_Not_PB0_17_Pin, GPIO_PIN_RESET);//Pull Estop pin to GND
}

void DisableSTOP()
{
	HAL_GPIO_WritePin(Stop_PC5_43_GPIO_Port,Stop_PC5_43_Pin,GPIO_PIN_RESET);//START , disable stop IO
}

void JogMoveUp() // Move up by pressing the JOG button on the UI
{
		HAL_GPIO_WritePin(Dir_Not_PE10_14_GPIO_Port, Dir_Not_PE10_14_Pin, GPIO_PIN_SET); // Choose the direction
		HAL_GPIO_WritePin(Stop_PC5_43_GPIO_Port,Stop_PC5_43_Pin,GPIO_PIN_RESET);//START , disable stop IO
}

void JogMoveDown()
{
		HAL_GPIO_WritePin(Dir_Not_PE10_14_GPIO_Port, Dir_Not_PE10_14_Pin, GPIO_PIN_RESET); // Choose the direction
		HAL_GPIO_WritePin(Stop_PC5_43_GPIO_Port,Stop_PC5_43_Pin,GPIO_PIN_RESET);//START , disable stop IO
}

void SetPositionMode()
{
	HAL_GPIO_WritePin(Type_Not_PE8_40_GPIO_Port, Type_Not_PE8_40_Pin, GPIO_PIN_SET); // Position Mode
}

void SetSpeedMode()
{
	HAL_GPIO_WritePin(Type_Not_PE8_40_GPIO_Port, Type_Not_PE8_40_Pin, GPIO_PIN_RESET); // Speed Mode
}

bool DriverInit()
{	
		HAL_GPIO_WritePin(Dir_Not_PE10_14_GPIO_Port, Dir_Not_PE10_14_Pin, GPIO_PIN_RESET); // CN1-14 - PLSCLR	
		HAL_GPIO_WritePin(Speed2_Not_PE7_15_GPIO_Port,Speed2_Not_PE7_15_Pin,GPIO_PIN_RESET);//CN1-15 SPDLIM/TLIM
		HAL_GPIO_WritePin(CCWLIM_Not_PE12_39_GPIO_Port,CCWLIM_Not_PE12_39_Pin,GPIO_PIN_RESET);//CN1-39 PLSINH
	
		HAL_GPIO_WritePin(SPDLIM_Not_PE11_38_GPIO_Port, SPDLIM_Not_PE11_38_Pin, GPIO_PIN_SET);// CN-38 - CWLIM
		HAL_GPIO_WritePin(CWLIM_Not_PE14_13_GPIO_Port,CWLIM_Not_PE14_13_Pin,GPIO_PIN_SET);//CN1-13 CCWLIM
	
		HAL_GPIO_WritePin(EStop_Not_PB0_17_GPIO_Port, EStop_Not_PB0_17_Pin, GPIO_PIN_RESET);// First, the driver will be in Emergency Stop
		HAL_GPIO_WritePin(SerVoReset_PC4_18_GPIO_Port, SerVoReset_PC4_18_Pin, GPIO_PIN_RESET); // Servo enable OFF
		
		HAL_GPIO_WritePin(PA12_LINE_DRV_EN_GPIO_Port, PA12_LINE_DRV_EN_Pin, GPIO_PIN_RESET);// Enable U14 DriveLine IC for generating pulses
		HAL_GPIO_WritePin(Type_Not_PE8_40_GPIO_Port, Type_Not_PE8_40_Pin, GPIO_PIN_RESET); // DIR	
	
		return true;
}
uint16_t ReadLogicF7000Out(void)
{ 
	uint16_t OuputState = 0;
	uint8_t i=0;
	if (HAL_GPIO_ReadPin(CN1_23_TYPEOUT_GPIO_Port,CN1_23_TYPEOUT_Pin)) // Read CN1-23-TYPEOUT
	{
		OuputState = OuputState | (1 << i); // Set ith bit		
	}
	i++;	
	if (HAL_GPIO_ReadPin(CN1_48_BRAKE_GPIO_Port,CN1_48_BRAKE_Pin)) // Read CN1-48-BRAKE
	{
		OuputState = OuputState | (1 << i); // Set ith bit
	}
	i++;
	if (HAL_GPIO_ReadPin(CN1_22_RDY_GPIO_Port,CN1_22_RDY_Pin)) // Read CN1-22-RDY
	{
		OuputState = OuputState | (1 << i); // Set ith bit
	}		
	i++;
	POSReach = HAL_GPIO_ReadPin(CN1_47_INSPD_INPOS_GPIO_Port,CN1_47_INSPD_INPOS_Pin);	// Check if the position is reached or not
//	if (HAL_GPIO_ReadPin(CN1_47_INSPD_INPOS_GPIO_Port,CN1_47_INSPD_INPOS_Pin)) // Read CN1-47
	if(POSReach)
	{		
		OuputState = OuputState | (1 << i); // Set ith bit
	}
	i++;	
	if (HAL_GPIO_ReadPin(CN1_21_SPDOUT_TRQOUT_GPIO_Port,CN1_21_SPDOUT_TRQOUT_Pin)) // Read CN1-21
	{
		OuputState = OuputState | (1 << i); // Set ith bit
	}		
	i++;
	if (HAL_GPIO_ReadPin(CN1_46_ALARM_GPIO_Port,CN1_46_ALARM_Pin)) // Read CN1-22-
	{
		OuputState = OuputState | (1 << i); // Set ith bit
	}
	i++;	
	if (HAL_GPIO_ReadPin(CN1_20_PCWOUT_PTQOUT_GPIO_Port,CN1_20_PCWOUT_PTQOUT_Pin)) // Read CN1-20
	{
		OuputState = OuputState | (1 << i); // Set ith bit
	}
	i++;	
	if (HAL_GPIO_ReadPin(CN1_45_NCWOUT_NTQOUT_GPIO_Port,CN1_45_NCWOUT_NTQOUT_Pin)) // Read CN1-45
	{
		OuputState = OuputState | (1 << i); // Set ith bit
	}
	i++;
	if (HAL_GPIO_ReadPin(CN1_19_ZSPD_GPIO_Port,CN1_19_ZSPD_Pin)) // Read CN1-19-ZSPD
	{
		OuputState = OuputState | (1 << i); // Set ith bit
	}	
	return OuputState;
}

void PulseClear()
{
	HAL_GPIO_WritePin(CN1_16_PulseCCLR_GPIO_Port, CN1_16_PulseCCLR_Pin, GPIO_PIN_RESET); // trig Alarm Reset Pin
	HAL_Delay(500);
	HAL_GPIO_WritePin(CN1_16_PulseCCLR_GPIO_Port, CN1_16_PulseCCLR_Pin, GPIO_PIN_SET);
}
