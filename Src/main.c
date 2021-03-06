/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
	#include "string.h"
	#include "stdio.h"
	#include <stdlib.h>
	#include "stdbool.h"
	#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
		uint8_t RxPCBuff[30]; // buffer to read the command from the PC
		uint8_t RxDriverBuff[20]; // buffer to read the data from the driver
		char DataRegion[20]; // array to filt the data from the PC
		uint8_t RxPCData; // variable to save 1 byte received from the PC
		uint8_t RxDriverData; // variable to save 1 byte received from the driver
		char TxPCBuff[30]; // buffer to send message to the PC
		uint8_t TxPCLen;
		//char Message2PC[25];
		char SpeedValueRegion[4];
		
		uint8_t TimeTick;
		
		uint8_t _rxIndex; // index for receiving data from the driver
		volatile  bool RxUart6_Cpl_Flag= false; // uart6 receiving complete flag (from the PC)
		volatile  bool RxUart5_Cpl_Flag= false; // uart5 receiving complete flag (from the Driver)

		volatile  bool TimerSpeedDataFlag = false; // timer flag
		volatile  bool TimerOutputDataFlag = false; // timer flag
		volatile 	bool OneEpisodeCompleted = false; 
		volatile  float MotorSpeed; // variable to save motor speed, it's value is changed in uart5 interrupt => volatile type
		volatile  bool StartDropping = false; // bit to check start or not
							float MaxSpeed;
		volatile  float SpeedCommand;

		volatile bool TimerFlag = false;
		volatile bool WritingCompleted = false;
		
		uint8_t EndChar = '$'; // Character to determine the ending of a data frame from the PC (GUI)
		int MotionCode[5]; // array to save the command from the PC
		uint16_t RegisterAddress; // Register of the address want to read/write
		volatile float P402RegisterValue;
		bool StartReceiveDriverData = false; // to check if start saving the driver data to the buffer RxDriverBuff or not
		bool UISpeedDataRequest = false; // to check if the GUI request data or not
		bool OutputDataRequest = false;
		
		
		
		// Parameter for Experiment, tam thoi chua dung
		
		uint8_t TotalEpisode;

		
		bool StartPulling = false;
		bool StopPulling = false;
		bool StartBraking  = false;
		float MinSpeed = 0;
		
		uint8_t TimeCount;

		float AccelerationTime = 4; // Default value
		float DeccelerationTime = 4; // Default value
		float Acceleration;
		float Deceleration;
		
		uint8_t CountTimerDriverOutput = 0;
		uint16_t DriverOutput; // to save the driver output
		uint8_t PullingSpeed = 50; // rpm, pulling speed
		
		
		#define DriverID 1
		#define P402_SpeedCommand 401 // Register address of the parameter P402
		#define StE03 12 // Register for motor speed in rpm
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_UART5_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

		
void Stop() // Stop motor function
{
		HAL_GPIO_WritePin(Stop_PC5_43_GPIO_Port,Stop_PC5_43_Pin,GPIO_PIN_SET);//(STOP) turn on pin 43
		HAL_GPIO_WritePin(SerVoReset_PC4_18_GPIO_Port, SerVoReset_PC4_18_Pin, GPIO_PIN_SET); // Disable Servo Enable
	
//		uint8_t u8_TxPCBuff[20]="< Stop Motor"; // Send a message to UI
//		HAL_UART_Transmit(&huart6,u8_TxPCBuff,sizeof(u8_TxPCBuff),100);
}

void AlarmReset() // reset alarm function
{
	  HAL_GPIO_WritePin(EStop_Not_PB0_17_GPIO_Port, EStop_Not_PB0_17_Pin, GPIO_PIN_SET);//Pull Estop pin to 1 (24V)
		HAL_GPIO_WritePin(ArlarmRST_PB1_42_GPIO_Port, ArlarmRST_PB1_42_Pin, GPIO_PIN_RESET); // trig Alarm Reset Pin
		HAL_Delay(20);
		HAL_GPIO_WritePin(ArlarmRST_PB1_42_GPIO_Port, ArlarmRST_PB1_42_Pin, GPIO_PIN_SET);
	
		HAL_GPIO_WritePin(Stop_PC5_43_GPIO_Port,Stop_PC5_43_Pin,GPIO_PIN_SET);//(STOP) turn on pin 43
		HAL_GPIO_WritePin(SerVoReset_PC4_18_GPIO_Port, SerVoReset_PC4_18_Pin, GPIO_PIN_SET); // Enable Servo Enable
	
//		uint8_t u8_TxPCBuff[20]="< Reset Alarm"; // Send a message to UI
//		HAL_UART_Transmit(&huart6,u8_TxPCBuff,sizeof(u8_TxPCBuff),100); 
}

void Estop() // Estop function
{
		HAL_GPIO_WritePin(EStop_Not_PB0_17_GPIO_Port, EStop_Not_PB0_17_Pin, GPIO_PIN_RESET);//Pull Estop pin to GND
	
//		uint8_t u8_TxPCBuff[20]="< Emergency Stop"; // Send Message to UI
//		HAL_UART_Transmit(&huart6,u8_TxPCBuff,sizeof(u8_TxPCBuff),100);
}

void JogMoveUp() // Move up by pressing the JOG button on the UI
{

		HAL_GPIO_WritePin(Dir_Not_PE10_14_GPIO_Port, Dir_Not_PE10_14_Pin, GPIO_PIN_SET); // Choose the direction
		HAL_GPIO_WritePin(Stop_PC5_43_GPIO_Port,Stop_PC5_43_Pin,GPIO_PIN_RESET);//START , disable stop IO
	
//		uint8_t u8_TxPCBuff[20]="< Jog Move up"; // Send out a message
//		HAL_UART_Transmit(&huart6,u8_TxPCBuff,sizeof(u8_TxPCBuff),100);
}

void JogMoveDown()
{

		HAL_GPIO_WritePin(Dir_Not_PE10_14_GPIO_Port, Dir_Not_PE10_14_Pin, GPIO_PIN_RESET); // Choose the direction
		HAL_GPIO_WritePin(Stop_PC5_43_GPIO_Port,Stop_PC5_43_Pin,GPIO_PIN_RESET);//START , disable stop IO
	
//		uint8_t u8_TxPCBuff[20]="< Jog Move down"; // Send out a message
//		HAL_UART_Transmit(&huart6,u8_TxPCBuff,sizeof(u8_TxPCBuff),100);
}


void DefaultMessage() // send a default message
{
//		uint8_t u8_TxPCBuff[20]="< Defaulte";
//	
//		HAL_UART_Transmit(&huart6,u8_TxPCBuff,sizeof(u8_TxPCBuff),100);
}
void SetPositionMode()
{
	
//		uint8_t u8_TxPCBuff[20]="< Position Mode";		
//		HAL_UART_Transmit(&huart6,u8_TxPCBuff,sizeof(u8_TxPCBuff),100);
}

void SetSpeedMode()
{
	
//		uint8_t u8_TxPCBuff[20]="< Speed Mode";		
//		HAL_UART_Transmit(&huart6,u8_TxPCBuff,sizeof(u8_TxPCBuff),100);
}
void ExtractMotionCode () // Extract command from the UI
{
	memset (MotionCode, '\0', sizeof (MotionCode)); // reset MotionCode
	memset (DataRegion, '\0', sizeof (DataRegion)); // reset DataRegion
	uint8_t j = 0;
	for(uint8_t i = 0; i< sizeof(RxPCBuff); i++)
	{
		if (RxPCBuff[i] != 0) // coppy the command from UI to another array, remove the null character at the beginning
		{
			DataRegion[j] = RxPCBuff[i]; // coppy to DataRegion
			j++;
		}
	}
	j = 0;
	char *token;
	token = strtok((char *)DataRegion, "/");	// Split the command ~ remove the / character
	while (token != NULL)
    {
				//MotionCode[j] = (atof(token)); // covert to float type 
			  MotionCode[j] = (atoi(token)); // covert to int type 
        token = strtok(NULL, "/");
				j++;
    }
	memset (RxPCBuff, '\0', sizeof (RxPCBuff)); // reset
}
void WriteFloatData (uint16_t RegisterAddress, float value) // Write a float value to the driver
{
	// Prepare data frame -- BEGIN
	uint8_t TxDataToDriver[10]; // 10 bytes of data
	
	// Data preparation
	TxDataToDriver[0] = DriverID;//SerialID of the driver, 1
	TxDataToDriver[1] = 6;//Write Regis, function code	
	TxDataToDriver[2] = RegisterAddress / 256; // Register Address High byte
  TxDataToDriver[3] = RegisterAddress % 256; // Register Address LOW byte
	
	char FloatValue[sizeof(float)];
	memcpy(FloatValue, &value, sizeof(float)); // convert float to 4 bytes of char
	
	TxDataToDriver[4] = FloatValue[3];
	TxDataToDriver[5] = FloatValue[2];
	TxDataToDriver[6] = FloatValue[1];
	TxDataToDriver[7] = FloatValue[0];
	
	//CRC BEGIN=======
	uint16_t crc = 0xFFFF;
	for (int pos = 0; pos < 8; pos++) //for (int pos = 0; pos < raw_msg_data.length()/2; pos++) 
	{	crc ^= (uint16_t)TxDataToDriver[pos];          // XOR byte into least sig. byte of crc
		for (int i = 8; i != 0; i--) 
		{    // Loop over each bit
			if ((crc & 0x0001) != 0) 
			{      // If the LSB is set
				crc >>= 1;                    // Shift right and XOR 0xA001
				crc ^= 0xA001;
			}
			else                            // Else LSB is not set
				crc >>= 1;                    // Just shift right
		}
	}
	TxDataToDriver[8]= (uint8_t)(crc&0x00FF);;//(uint8_t)(TemDat16&0xFF);
	TxDataToDriver[9]=(uint8_t)((crc>>8)&0x00FF);				
	//CRC=====END*/
	// Prepare data frame -- END
	
	// Send data use UART5
	HAL_GPIO_WritePin(PE0_485_MCU_DRV_DIR_GPIO_Port, PE0_485_MCU_DRV_DIR_Pin, GPIO_PIN_RESET); //Switch to transmit mode
	HAL_UART_Transmit(&huart5,TxDataToDriver,sizeof(TxDataToDriver),200); // use UART5 to send	
	HAL_GPIO_WritePin(PE0_485_MCU_DRV_DIR_GPIO_Port, PE0_485_MCU_DRV_DIR_Pin, GPIO_PIN_SET);	//Switch back to receive mode
	HAL_UART_Receive_IT(&huart5,&RxDriverData,1); // Receive 1 byte each time
	/// For debug only, should be comment out latter
//	uint8_t u8_TxPCBuff[10]="<Sete";
//	HAL_UART_Transmit(&huart6,u8_TxPCBuff,sizeof(u8_TxPCBuff),100);//*/	
	//*/ Dubug END
}

void WriteIntData (uint16_t RegisterAddress, int value) // Write an int value to the driver
{
	// Prepare data frame -- BEGIN
	uint8_t TxDataToDriver[10]; // 10 bytes of data
	
	// Data preparation
	TxDataToDriver[0] = DriverID;//SerialID of the driver, 1
	TxDataToDriver[1] = 6;//Write Regis, function code	
	TxDataToDriver[2] = RegisterAddress / 256; // Register Address High byte
  TxDataToDriver[3] = RegisterAddress % 256; // Register Address LOW byte
	
	TxDataToDriver[4] = (char)((value >> 24) & 0xFF); // convert to 4 bytes data
	TxDataToDriver[5] = (char)((value >> 16) & 0xFF);
	TxDataToDriver[6] = (char)((value >> 8) & 0xFF);
	TxDataToDriver[7] =  (char)(value & 0xFF);
	
	//CRC BEGIN=======
	uint16_t crc = 0xFFFF;
	for (int pos = 0; pos < 8; pos++) //for (int pos = 0; pos < raw_msg_data.length()/2; pos++) 
	{	crc ^= (uint16_t)TxDataToDriver[pos];          // XOR byte into least sig. byte of crc
		for (int i = 8; i != 0; i--) 
		{    // Loop over each bit
			if ((crc & 0x0001) != 0) 
			{      // If the LSB is set
				crc >>= 1;                    // Shift right and XOR 0xA001
				crc ^= 0xA001;
			}
			else                            // Else LSB is not set
				crc >>= 1;                    // Just shift right
		}
	}
	TxDataToDriver[8]= (uint8_t)(crc&0x00FF);;//(uint8_t)(TemDat16&0xFF);
	TxDataToDriver[9]=(uint8_t)((crc>>8)&0x00FF);				
	//CRC=====END*/
	// Prepare data frame -- END
	// Send data use UART5
	HAL_GPIO_WritePin(PE0_485_MCU_DRV_DIR_GPIO_Port, PE0_485_MCU_DRV_DIR_Pin, GPIO_PIN_RESET); //Switch to transmit mode
	HAL_UART_Transmit(&huart5,TxDataToDriver,sizeof(TxDataToDriver),200); // use UART5 to send
	
	/// For debug only, should be comment out latter
//	uint8_t u8_TxPCBuff[20]="< Send Int\n";
//	HAL_UART_Transmit(&huart6,u8_TxPCBuff,sizeof(u8_TxPCBuff),100);//*/	
	//*/ Dubug END

}

void ReadDriverData(uint16_t RegisterAddress) // Read data from the Driver
{
	// Prepare data frame -- BEGIN
	uint8_t TxDataToDriver[8]; // 8 bytes of data frame
	
	// Data preparation
	TxDataToDriver[0] = DriverID;//SerialID = 1 of the driver
	TxDataToDriver[1] = 3;//Read Regis, function code	
	TxDataToDriver[2] = RegisterAddress / 256; // Register Address High byte
  TxDataToDriver[3] = RegisterAddress % 256; // Register Address LOW byte
	TxDataToDriver[4] = 0; // Number of Register HIGH byte
	TxDataToDriver[5] = 1; // Number of Register LOW byte, read 1 Register only
	
	//CRC BEGIN=======
				uint16_t crc = 0xFFFF;
				for (int pos = 0; pos < 6; pos++) //for (int pos = 0; pos < raw_msg_data.length()/2; pos++) 
				{	crc ^= (uint16_t)TxDataToDriver[pos];          // XOR byte into least sig. byte of crc
					for (int i = 8; i != 0; i--) 
					{    // Loop over each bit
						if ((crc & 0x0001) != 0) 
						{      // If the LSB is set
							crc >>= 1;                    // Shift right and XOR 0xA001
							crc ^= 0xA001;
						}
						else                            // Else LSB is not set
							crc >>= 1;                    // Just shift right
					}
				}
	TxDataToDriver[6]= (uint8_t)(crc&0x00FF);;//(uint8_t)(TemDat16&0xFF);
	TxDataToDriver[7]=(uint8_t)((crc>>8)&0x00FF);				
	//CRC=====END*/
	// Prepare data frame -- END
	// Send data use UART5
	HAL_GPIO_WritePin(PE0_485_MCU_DRV_DIR_GPIO_Port, PE0_485_MCU_DRV_DIR_Pin, GPIO_PIN_RESET); //Switch to transmit mode
//	HAL_Delay(1);
	HAL_UART_Transmit(&huart5,TxDataToDriver,sizeof(TxDataToDriver),200); // use UART5 to send
	HAL_GPIO_WritePin(PE0_485_MCU_DRV_DIR_GPIO_Port, PE0_485_MCU_DRV_DIR_Pin, GPIO_PIN_SET);	//Switch back to receive mode
	HAL_UART_Receive_IT(&huart5,&RxDriverData,1); // Receive 1 byte each time
	
	/// For debug only, should be comment out latter
	//uint8_t u8_TxPCBuff[20]="< Read Data";
	//HAL_UART_Transmit(&huart6,u8_TxPCBuff,sizeof(u8_TxPCBuff),1000);	
	//*/ Dubug END
}

/*void Running ()
{
	if (Accelerating) // Acceleration Stage
	{
		SpeedCommand = Acceleration*RunningTime;
		if (SpeedCommand > MaxSpeed) 
		{
			SpeedCommand = MaxSpeed;
			Accelerating = false;
		}
	}
	if (!Accelerating) // Deceleration Stage
	{
		SpeedCommand = MaxSpeed - fabs(Deceleration)*(RunningTime-AccelerationTime);
		if (SpeedCommand <= 0) 
		{
			SpeedCommand = 0;
			StartDropping = false; // Stop running
			Accelerating = true;
			RunningTime = 0;
			TimeCount = 0;
		}
	}
}*/

void Dropping() // currently dont use
{
		HAL_GPIO_WritePin(Dir_Not_PE10_14_GPIO_Port, Dir_Not_PE10_14_Pin, GPIO_PIN_SET); // Choose the direction
		HAL_GPIO_WritePin(Stop_PC5_43_GPIO_Port,Stop_PC5_43_Pin,GPIO_PIN_RESET);//START , disable stop IO
		if (MotorSpeed >= MaxSpeed)		
		{
			Stop();
			StartDropping = false;
			StartBraking = true;
		}
}
void Braking ()
{
		Stop();
		if (HAL_GPIO_ReadPin(CN1_19_ZSPD_GPIO_Port,CN1_19_ZSPD_Pin)) // If the motor is stopped, read CN1-19 to know
			{
				WriteFloatData(P402_SpeedCommand, PullingSpeed); // Pulling Speed = 50 rpm;
				//while (!WritingCompleted) {}; // Wait for complete writing
				//WritingCompleted = false;
				HAL_GPIO_WritePin(Dir_Not_PE10_14_GPIO_Port, Dir_Not_PE10_14_Pin, GPIO_PIN_RESET); // Choose the direction
				HAL_GPIO_WritePin(Stop_PC5_43_GPIO_Port,Stop_PC5_43_Pin,GPIO_PIN_RESET);//START , disable stop IO
				StartPulling = true;
				StartBraking = false;
			}
}
void ProcessReceivedCommand () // Proceed the command from the UI
{
			//ExtractMotionCode(); // Extract data to MotionCode
			switch ((int)MotionCode[0])
			{
				case 0:
					if ((int)MotionCode[1] == 0) // 0/0
					{
						Estop(); // Estop button on the UI
					}
					else {AlarmReset();}  // 0/1, alarm button
					break;
				case 1:
					if ((int)MotionCode[1] == 1) // 1/1
					{
						Stop(); // Stop button
					}
					break;
				case 2:
					if ((int)MotionCode[1] == 0) {SetPositionMode();} // 2/0 position mode
					else {SetSpeedMode();} // 2/1 speed mode
					break;
				case 3:
					if ((int)MotionCode[1] == 1) {JogMoveUp();} // 3/1 move up button
					else {JogMoveDown();} // 3/0 move down button
					break;
				case 4: // Start Running START button
					if ((int)MotionCode[1] == 1) // Start runing free-fall
						{
							WriteFloatData(P402_SpeedCommand, MaxSpeed);
//							while (!WritingCompleted) {}; // Wait for complete writing
//							WritingCompleted = false;
							StartDropping = true;
							StartPulling = false;
						}
					else // Stop running
						{
							StartDropping = false;
							StartPulling = false;
							SpeedCommand = 0;
							TimeCount = 0;
						}
					break;
				case 5: // 5 is the function code for writing to a register
								// 5/data type/adress /value
					if ((int)MotionCode[1] == 0) // Write int Value
					{
						WriteIntData((uint16_t)MotionCode[2], (uint16_t)MotionCode[3]);
					}
					else
					{
						MaxSpeed = roundf(MotionCode[3] * 100)/100;	
						WriteFloatData(P402_SpeedCommand, MaxSpeed);						
					}// Write speed command to P402;} // Write float data
					break;
					
				case 6: // 6 request read speed data
					if((int)MotionCode[1] == 1) {UISpeedDataRequest = true;} // 6/1 If the UI request data
					else {UISpeedDataRequest = false;}
					break;
							
				case 7: // Set running parameter
					AccelerationTime = MotionCode[1]; //
					DeccelerationTime = MotionCode[2];
					MinSpeed = MotionCode[3];
					MaxSpeed = MotionCode[4];
					Acceleration = (MaxSpeed - MinSpeed)/AccelerationTime; // rpm/s
					Deceleration = (MinSpeed - MaxSpeed)/DeccelerationTime; 
					// Send back to the UI to check					
					char buffer[30];
					TxPCLen = sprintf(buffer,"r%.1f/%.1f/%.1f/%.1fe",AccelerationTime,DeccelerationTime,MinSpeed,MaxSpeed);
					// r stands for running param, e is for the ending character.
					HAL_UART_Transmit(&huart6,(uint8_t *)buffer,TxPCLen,200); // Send to uart6 to check the params are set or not
					break;
				case 8: // Request reading output data or not
					if((int)MotionCode[1] == 1) {OutputDataRequest = true;} // 8/1 = request
					else OutputDataRequest = false; // 8/0 = stop request
					break;
				case 9:
					TotalEpisode = (uint8_t)MotionCode[1];
				default:
					DefaultMessage();
					break;		
			}
			//memset (MotionCode, '\0', sizeof (MotionCode)); // reset RxPCBuff
//			memset (RxPCBuff, '\0', sizeof (RxPCBuff)); // reset RxPCBuff
//			memset (DataRegion, '\0', sizeof (DataRegion)); // reset RxPCBuff
}
bool SetupF7000Ready(bool SpeedControl,bool DirCW)
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
		//Check Stt
		return true;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) // Callback function when a receiving complete
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
		if (huart->Instance==USART6) // If it is uart6, UI communication
		{
			if(RxPCData!=EndChar) // read up to the ending char
			{
				if (RxPCData != NULL) // remove the null character
				{
					RxPCBuff[_rxIndex]=RxPCData;// Copy the data to buffer
				  _rxIndex++;
				}		
			}
			else //if(RxPCData==EndChar)
			{
				ExtractMotionCode();
				_rxIndex=0;
				RxUart6_Cpl_Flag=true; // reading completed
			}
			HAL_UART_Receive_IT(&huart6,&RxPCData,1);
		}
		//END USART3 = HAL_UART_Receive_IT============================================*/
		
		//BEGIN UART5 = HAL_UART_Receive_IT============================================
		/// Use this part
		if (huart->Instance==UART5) // If it is uart5, driver communication
		{			
			if (_rxIndex >= 9) // Complete receiving the data
				{
					switch (RxDriverBuff[1])
					{
						case 3: // Read driver data
							SpeedValueRegion[0] = RxDriverBuff[6];
							SpeedValueRegion[1] = RxDriverBuff[5];
							SpeedValueRegion[2] = RxDriverBuff[4];
							SpeedValueRegion[3] = RxDriverBuff[3];	

							MotorSpeed = *(float *)&SpeedValueRegion;					
							//memcpy(&MotorSpeed, SpeedValueRegion, sizeof(MotorSpeed)); // extract the motor speed
							RxUart5_Cpl_Flag = true; // Complete Receive
							break;
						case 6: // Feedback frame due to writing data, fcn code 06
							RegisterAddress =  (RxDriverBuff[2] << 8) + RxDriverBuff[3];							
						
							SpeedValueRegion[0] = RxDriverBuff[7];
							SpeedValueRegion[1] = RxDriverBuff[6];
							SpeedValueRegion[2] = RxDriverBuff[5];
							SpeedValueRegion[3] = RxDriverBuff[4];
						
							P402RegisterValue = *(float *)&SpeedValueRegion;
							char buffer[15];
							TxPCLen = sprintf(buffer,"<%d/%.1fe",RegisterAddress,P402RegisterValue);
							HAL_UART_Transmit(&huart6,(uint8_t *)buffer,TxPCLen,200); // use uart6 to send						
							//memset (RxDriverBuff, '\0', sizeof (RxDriverBuff)); // reset RxPCBuff
							WritingCompleted = true;
							break;
						default: break;
					}					
					StartReceiveDriverData = false;
					_rxIndex = 0;
					HAL_UART_Receive_IT(&huart5,&RxDriverData,1); // Receive 1 byte each time
					//return;
				}
			if ((_rxIndex == 0)&&(RxDriverData == DriverID)) // If byte 0 is the Driver ID
			{
				StartReceiveDriverData = true; 
			}
			if (StartReceiveDriverData) //
			{
				RxDriverBuff[_rxIndex]=RxDriverData;// Copy the data to buffer
				_rxIndex++;	
			}
			HAL_UART_Receive_IT(&huart5,&RxDriverData,1); // Receive 1 byte each time ///*/			
		}
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
	if (HAL_GPIO_ReadPin(CN1_47_INSPD_INPOS_GPIO_Port,CN1_47_INSPD_INPOS_Pin)) // Read CN1-47
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) // Timer 2 interrupt, 50ms
{
	TimerFlag = true;
	if(OutputDataRequest)
	{
		CountTimerDriverOutput++;
		if (CountTimerDriverOutput >= 10) // 500 ms
		{
			TimerOutputDataFlag = true;
			CountTimerDriverOutput = 0;
		}
	}	
	
	if (UISpeedDataRequest) {TimerSpeedDataFlag = true;}	

	if (StartPulling)
	{
		TimeTick++;
		if (TimeTick >= 60) // Pulling in 3 secs
		{
			Stop(); // Stop or Estop?
			TimeTick = 0;
		}
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
		
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_UART5_Init();
  MX_USART6_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(PE15_RELAY1_GPIO_Port, PE15_RELAY1_Pin, GPIO_PIN_SET);
	HAL_Delay(5000);
	
	HAL_TIM_Base_Start_IT(&htim2); // Enable Timer 2 interrupt
	HAL_UART_Receive_IT(&huart6,&RxPCData,1);
	SetupF7000Ready(true,true);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(RxUart6_Cpl_Flag) // If data is completely received, a command is sent from the UI
			{
				ProcessReceivedCommand (); // Proceed the command
				RxUart6_Cpl_Flag=false;
			}
		// Running an episode		
		if (StartDropping)
		{
			Dropping();
		}
		if (StartBraking) // Braking stage
		{			
			Braking();
		}
		if (TimerFlag)
			{
				TimerFlag = false;
				if(TimerSpeedDataFlag && TimerOutputDataFlag) // if the UI request Speed and driver output
						{
							TxPCLen = sprintf(TxPCBuff,"s3/%.1f/%de",MotorSpeed,DriverOutput); // s means speed, 3 means both driver output and motor speed
							HAL_UART_Transmit(&huart6,(uint8_t *)TxPCBuff,TxPCLen,200); // use uart6 to send							
							DriverOutput = ReadLogicF7000Out(); // Read Driver Output
							TimerSpeedDataFlag = false;
							TimerOutputDataFlag = false;
						}
				if(TimerOutputDataFlag && !TimerSpeedDataFlag) // Send only the Driver Outputs
						{
							TxPCLen = sprintf(TxPCBuff,"s1/%de",DriverOutput); // 1 means only the driver outputs
							HAL_UART_Transmit(&huart6,(uint8_t *)TxPCBuff,TxPCLen,200); // use uart6 to send
							DriverOutput = ReadLogicF7000Out(); // Read Driver Output
							TimerOutputDataFlag = false;
						}
				if(!TimerOutputDataFlag && TimerSpeedDataFlag) // Send only the Motor speed.
						{
							TxPCLen = sprintf(TxPCBuff,"s2/%.1fe",MotorSpeed); // s means speed, 2 means only the motor speed
							HAL_UART_Transmit(&huart6,(uint8_t *)TxPCBuff,TxPCLen,200); // use uart6 to send
							DriverOutput = ReadLogicF7000Out(); // Read Driver Output
							TimerSpeedDataFlag = false;
						}
				ReadDriverData(StE03); // StE03 = 12 = Motor Speed, Read motor speed
			}		
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART6_IRQn);
  /* UART5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UART5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(UART5_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 500;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 8400;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, PE4_ZIGBEE_PA_EN_Pin|Type_Not_PE8_40_Pin|PE9_TIM1_CH1_PFIN_Pin|Speed1_not_PE13_41_Pin
                          |PE15_RELAY1_Pin|PE0_485_MCU_DRV_DIR_Pin|PE1_ZIGBEE_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, PC3_ZIGBEE_nRST_Pin|PC9_ZIGBEE_HGM_EN_Pin|PC10_SPI3_SCK_SPARE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PA3_LCD_RST_Pin|PA8_LINE_DRV_SELFTEST2_Pin|PA10_LINE_DRV_SELFTEST1_Pin|PA11_ENC_RECEIV_EN_Pin
                          |PA12_LINE_DRV_EN_Pin|PA15_SPI3_NSS_SPARE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SerVoReset_PC4_18_Pin|Stop_PC5_43_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EStop_Not_PB0_17_Pin|Tor1_PB2_16_Pin|PB13_Output_JP7_Pin|PB14_POS_CMD_OPC_EN_Pin
                          |PB5_SPI3_MOSI_SPARE_Pin|PB6_RELAY2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ArlarmRST_PB1_42_Pin|PB15_485_MCU_PC_DIR_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, Speed2_Not_PE7_15_Pin|Dir_Not_PE10_14_Pin|SPDLIM_Not_PE11_38_Pin|CCWLIM_Not_PE12_39_Pin
                          |CWLIM_Not_PE14_13_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PD10_ESP32_EN_GPIO_Port, PD10_ESP32_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, PD11_ESP32_BOOT_SEL_Pin|PD13_MON1_2_EN_Pin|PD15_SPDIN_TRQIN_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CN1_22_RDY_Pin CN1_21_SPDOUT_TRQOUT_Pin PE5_BLE_GPIO_Pin CN1_48_BRAKE_Pin */
  GPIO_InitStruct.Pin = CN1_22_RDY_Pin|CN1_21_SPDOUT_TRQOUT_Pin|PE5_BLE_GPIO_Pin|CN1_48_BRAKE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE4_ZIGBEE_PA_EN_Pin Speed2_Not_PE7_15_Pin Type_Not_PE8_40_Pin PE9_TIM1_CH1_PFIN_Pin
                           Dir_Not_PE10_14_Pin SPDLIM_Not_PE11_38_Pin CCWLIM_Not_PE12_39_Pin Speed1_not_PE13_41_Pin
                           CWLIM_Not_PE14_13_Pin PE15_RELAY1_Pin PE0_485_MCU_DRV_DIR_Pin PE1_ZIGBEE_EN_Pin */
  GPIO_InitStruct.Pin = PE4_ZIGBEE_PA_EN_Pin|Speed2_Not_PE7_15_Pin|Type_Not_PE8_40_Pin|PE9_TIM1_CH1_PFIN_Pin
                          |Dir_Not_PE10_14_Pin|SPDLIM_Not_PE11_38_Pin|CCWLIM_Not_PE12_39_Pin|Speed1_not_PE13_41_Pin
                          |CWLIM_Not_PE14_13_Pin|PE15_RELAY1_Pin|PE0_485_MCU_DRV_DIR_Pin|PE1_ZIGBEE_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC13_USER_BT_MID_Pin CN1_46_ALARM_Pin CN1_47_INSPD_INPOS_Pin CN1_45_NCWOUT_NTQOUT_Pin */
  GPIO_InitStruct.Pin = PC13_USER_BT_MID_Pin|CN1_46_ALARM_Pin|CN1_47_INSPD_INPOS_Pin|CN1_45_NCWOUT_NTQOUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC3_ZIGBEE_nRST_Pin SerVoReset_PC4_18_Pin Stop_PC5_43_Pin PC9_ZIGBEE_HGM_EN_Pin
                           PC10_SPI3_SCK_SPARE_Pin */
  GPIO_InitStruct.Pin = PC3_ZIGBEE_nRST_Pin|SerVoReset_PC4_18_Pin|Stop_PC5_43_Pin|PC9_ZIGBEE_HGM_EN_Pin
                          |PC10_SPI3_SCK_SPARE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0_UART4_TX_ESP32_RX_Pin */
  GPIO_InitStruct.Pin = PA0_UART4_TX_ESP32_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(PA0_UART4_TX_ESP32_RX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2_USER_BT_UP_Pin CN1_20_PCWOUT_PTQOUT_Pin CN1_23_TYPEOUT_Pin PA9_LINE_RECV_SELFTEST_Pin */
  GPIO_InitStruct.Pin = PA2_USER_BT_UP_Pin|CN1_20_PCWOUT_PTQOUT_Pin|CN1_23_TYPEOUT_Pin|PA9_LINE_RECV_SELFTEST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3_LCD_RST_Pin PA8_LINE_DRV_SELFTEST2_Pin PA10_LINE_DRV_SELFTEST1_Pin PA11_ENC_RECEIV_EN_Pin
                           PA12_LINE_DRV_EN_Pin PA15_SPI3_NSS_SPARE_Pin */
  GPIO_InitStruct.Pin = PA3_LCD_RST_Pin|PA8_LINE_DRV_SELFTEST2_Pin|PA10_LINE_DRV_SELFTEST1_Pin|PA11_ENC_RECEIV_EN_Pin
                          |PA12_LINE_DRV_EN_Pin|PA15_SPI3_NSS_SPARE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6_TIM3_CH1_ENC_PZO_Pin */
  GPIO_InitStruct.Pin = PA6_TIM3_CH1_ENC_PZO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(PA6_TIM3_CH1_ENC_PZO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EStop_Not_PB0_17_Pin ArlarmRST_PB1_42_Pin Tor1_PB2_16_Pin PB13_Output_JP7_Pin
                           PB14_POS_CMD_OPC_EN_Pin PB15_485_MCU_PC_DIR_Pin PB5_SPI3_MOSI_SPARE_Pin PB6_RELAY2_Pin */
  GPIO_InitStruct.Pin = EStop_Not_PB0_17_Pin|ArlarmRST_PB1_42_Pin|Tor1_PB2_16_Pin|PB13_Output_JP7_Pin
                          |PB14_POS_CMD_OPC_EN_Pin|PB15_485_MCU_PC_DIR_Pin|PB5_SPI3_MOSI_SPARE_Pin|PB6_RELAY2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10_I2C2_SCL_LCD_IOEXP_Pin PB11_I2C2_SDA_LCD_IOEXP_Pin */
  GPIO_InitStruct.Pin = PB10_I2C2_SCL_LCD_IOEXP_Pin|PB11_I2C2_SDA_LCD_IOEXP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12_RELAY3_Pin CN1_19_ZSPD_Pin PB4_SPI3_MISO_SPARE_Pin Input_JP7_Pin */
  GPIO_InitStruct.Pin = PB12_RELAY3_Pin|CN1_19_ZSPD_Pin|PB4_SPI3_MISO_SPARE_Pin|Input_JP7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8_USART3_TX_485_MCU_PC_Pin PD9_USART3_RX_485_MCU_PC_Pin */
  GPIO_InitStruct.Pin = PD8_USART3_TX_485_MCU_PC_Pin|PD9_USART3_RX_485_MCU_PC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD10_ESP32_EN_Pin */
  GPIO_InitStruct.Pin = PD10_ESP32_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PD10_ESP32_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD11_ESP32_BOOT_SEL_Pin PD13_MON1_2_EN_Pin PD15_SPDIN_TRQIN_EN_Pin */
  GPIO_InitStruct.Pin = PD11_ESP32_BOOT_SEL_Pin|PD13_MON1_2_EN_Pin|PD15_SPDIN_TRQIN_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12_Input_J6_Pin PD14_Input_J6_Pin Input0_J6_DAC_ADC_Pin Input1_J6_DAC_ADC_Pin
                           PD7_A_CODE2_Pin */
  GPIO_InitStruct.Pin = PD12_Input_J6_Pin|PD14_Input_J6_Pin|Input0_J6_DAC_ADC_Pin|Input1_J6_DAC_ADC_Pin
                          |PD7_A_CODE2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8_TIM8_CH3_PRIN_Pin */
  GPIO_InitStruct.Pin = PC8_TIM8_CH3_PRIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
  HAL_GPIO_Init(PC8_TIM8_CH3_PRIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB7_I2C1_SDA_DAC_ADC_Pin PB8_I2C1_SCL_DAC_ADC_Pin */
  GPIO_InitStruct.Pin = PB7_I2C1_SDA_DAC_ADC_Pin|PB8_I2C1_SCL_DAC_ADC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

