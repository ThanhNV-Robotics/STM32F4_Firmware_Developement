#include "Function.h"
#include "main.h"

#include "string.h"
#include "stdio.h"
#include <stdlib.h>
#include "stdbool.h"
#include <math.h>
//#include <FLASH_SECTOR_F4.h>

void Stop() // Stop motor function
{
		HAL_GPIO_WritePin(Stop_PC5_43_GPIO_Port,Stop_PC5_43_Pin,GPIO_PIN_SET);//(STOP) turn on pin 43
		HAL_GPIO_WritePin(SerVoReset_PC4_18_GPIO_Port, SerVoReset_PC4_18_Pin, GPIO_PIN_SET); // Disable Servo Enable
}

void AlarmReset() // reset alarm function
{
	  HAL_GPIO_WritePin(EStop_Not_PB0_17_GPIO_Port, EStop_Not_PB0_17_Pin, GPIO_PIN_SET);//Pull Estop pin to 1 (24V)
		HAL_GPIO_WritePin(ArlarmRST_PB1_42_GPIO_Port, ArlarmRST_PB1_42_Pin, GPIO_PIN_RESET); // trig Alarm Reset Pin
		HAL_Delay(20);
		HAL_GPIO_WritePin(ArlarmRST_PB1_42_GPIO_Port, ArlarmRST_PB1_42_Pin, GPIO_PIN_SET);
	
		HAL_GPIO_WritePin(Stop_PC5_43_GPIO_Port,Stop_PC5_43_Pin,GPIO_PIN_SET);//(STOP) turn on pin 43
		HAL_GPIO_WritePin(SerVoReset_PC4_18_GPIO_Port, SerVoReset_PC4_18_Pin, GPIO_PIN_SET); // Enable Servo Enable
}

void Estop() // Estop function
{
		HAL_GPIO_WritePin(EStop_Not_PB0_17_GPIO_Port, EStop_Not_PB0_17_Pin, GPIO_PIN_RESET);//Pull Estop pin to GND
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

}

void SetSpeedMode()
{

}

bool DriverInit(bool SpeedControl,bool DirCW)
{	
		HAL_GPIO_WritePin(EStop_Not_PB0_17_GPIO_Port, EStop_Not_PB0_17_Pin, GPIO_PIN_RESET);// First, the driver will be in Emergency Stop
		
		HAL_GPIO_WritePin(SerVoReset_PC4_18_GPIO_Port, SerVoReset_PC4_18_Pin, GPIO_PIN_SET); // Servo enable on
		
		if (SpeedControl)
			HAL_GPIO_WritePin(Type_Not_PE8_40_GPIO_Port, Type_Not_PE8_40_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(Type_Not_PE8_40_GPIO_Port, Type_Not_PE8_40_Pin, GPIO_PIN_RESET);
		if (DirCW)
			HAL_GPIO_WritePin(Dir_Not_PE10_14_GPIO_Port, Dir_Not_PE10_14_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(Dir_Not_PE10_14_GPIO_Port, Dir_Not_PE10_14_Pin, GPIO_PIN_RESET);
		
		HAL_GPIO_WritePin(SPDLIM_Not_PE11_38_GPIO_Port, SPDLIM_Not_PE11_38_Pin, GPIO_PIN_RESET);//2022.01.24
		return true;
}
