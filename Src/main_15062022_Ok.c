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
	#include "Function.h"
	#include "string.h"
	#include "stdio.h"
	#include <stdlib.h>
	#include "stdbool.h"
	#include <math.h>
	#include <FLASH_SECTOR_F4.h>
//	#include <pid.h>
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
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
		volatile uint8_t RxPCBuff[40]; // buffer to read the command from the PC
		volatile uint8_t RxDriverBuff[10]; // buffer to read the data from the driver
		
		char DataRegion[40]; // array to filt the data from the PC
		uint8_t RxPCData; // variable to save 1 byte received from the PC
		uint8_t RxDriverData; // variable to save 1 byte received from the driver
		char TxPCBuff[30]; // buffer to send message to the PC
		uint8_t TxPCLen;

		char SpeedValueRegion[4];		
		
		uint8_t _rxIndex; // index for receiving data from the driver
		
		// Flag variables
		volatile  bool RxUart6_Cpl_Flag= false; // uart6 receiving complete flag (from the PC)
		volatile  bool RxUart5_Cpl_Flag= false; // uart5 receiving complete flag (from the Driver)

		//volatile bool IsReadTachometer = false;
		volatile bool IsReadMotorSpeed = false;
		volatile bool IsReadEncoderPulse = true;
		
//		volatile bool ReadingEncoderCompleted = false;
//		volatile bool ReadingSpeedCompleted = false;
//		volatile  bool TimerSpeedDataFlag = false; // timer flag
//		volatile  bool TimerOutputDataFlag = false; // timer flag
//		volatile  bool DataSendingFlag  = false;
		
		volatile  bool StartDropping = false; // bit to check start or not
		volatile 	bool StartRunning;
		volatile bool Direction; // false = move up, true = move down
		volatile bool Timer2SampleTimeInterrupt;
		
		volatile bool StartReceiveDriverData = false; // to check if start saving the driver data to the buffer RxDriverBuff or not
		bool UIDataRequest = false; // to check if the GUI request data or not
		bool OutputDataRequest = false;
		bool PositionControlMode = true; // Position Mode is default
		
		bool StartPulling = false;
		bool StartBraking  = false;
		bool PulseGenerationFlag = false;
		bool IsStepPulseCmd = false; //
		bool POSReach =  false; // position reach flag
		bool StartAccleratePulling = false;
		
		uint8_t ExperimentMode = 1; // 1: Dropping Mode, 2: Pulling Mode, 3: Pulling->Dropping Mode
		bool RunningMode = false; // false = Manual; true = automatic
		

		bool ToggleReadingData = false;
		
		bool PRIsToggled; // to handle the pulse generation
		
		uint16_t RunningTime = 0;
		
		uint8_t Timer2Count;
		uint8_t DataSendingTimeCount;
		
		uint16_t CountTimerDriverOutput = 0;
		uint16_t DriverOutput; // to save the driver output
		uint16_t JogSpeed = 30;     // Control the Jog Speed 30 rpm is default
		//volatile uint16_t TachometerCount;
		
		uint16_t Timer3CountPeriod; //
		uint16_t Timer3Count;
		uint16_t StoppingTimeCount;
		
		//General Parameters
		float DrumRadius;
		uint8_t SampleTime; // sample time
		uint8_t kbrake = 2;
		uint8_t PullingSpeed; // rpm, pulling speed, and going down Speed
		uint16_t StoppingTime ;
		uint8_t EgearRatio = 8;
		
		// Dropping Experiment Mode			
		uint16_t DroppingDistance;
		float AccRefDropping; // Reference acceleration		
		float EpsilonDropping; // Angular acceleration		
		uint16_t DroppingMaxSpeed;
		
		// Pulling Experiment Mode	
		uint16_t PullingDistance;
		float AccRefPulling;
		float EpsilonPulling;
		uint16_t PullingMaxSpeed;
		int BotomPulseCmdPosition;
		uint16_t TotalPullingDistance;
		// Pulling and Dropping Mode
		uint16_t PD_Distance;
		float PD_PullAccRef;
		float PD_DropAccRef;
		uint16_t PD_DroppingMaxSpeed;
		uint16_t PD_PullingMaxSpeed;
		float PD_EpsilonDrop;
		float PD_EpsilonPull;
		int PulseCmd;		
		
		float SpeedCmd;
		
//	PID_TypeDef TPID; // PID controller

		uint8_t numofwords = 12; // The number of words To save to flash memory
		float Params[11] = {0, 0, 0, 0, 0, 0,0,0,0,0,0}; // DrumRadius, DroppingDistance, PullingSpeed, StoppingTime, AccRefDropping, SampleTime

		
//		volatile float AccZ ; // Feedback acceleration
		
		
		
		
		float MotorSpeed; // variable to save motor speed, it's value is changed in uart5 interrupt => volatile type
		volatile int CurrentEncPulse;
		volatile int PositionPulseCmd; 
		int StepPulseCmd;	
		
		const uint8_t EndChar = '$'; // Character to determine the ending of a data frame from the PC (GUI)
		float MotionCode[8]; // array to save the command from the PC
		uint16_t RegisterAddress; // Register of the address want to read/write
		
		
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_UART5_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

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
				MotionCode[j] = (atof(token)); // covert to float type 
			  //MotionCode[j] = (atoi(token)); // covert to int type 
        token = strtok(NULL, "/");
				j++;
    }
	memset (RxPCBuff, '\0', sizeof (RxPCBuff)); // reset
}
void WriteFloatData (uint16_t RegisterAddress, float value, bool GetReceived) // Write a float value to the driver
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
	//CRC=====END/
	// Prepare data frame -- END
	
	// Send data use UART5
	HAL_GPIO_WritePin(PE0_485_MCU_DRV_DIR_GPIO_Port, PE0_485_MCU_DRV_DIR_Pin, GPIO_PIN_RESET); //Switch to transmit mode
	HAL_UART_Transmit(&huart5,TxDataToDriver,sizeof(TxDataToDriver),200); // use UART5 to send
	
	if (GetReceived) // Get Received feedback if GetReceived = true
	{
		HAL_GPIO_WritePin(PE0_485_MCU_DRV_DIR_GPIO_Port, PE0_485_MCU_DRV_DIR_Pin, GPIO_PIN_SET);	//Switch back to receive mode
	}
}

void WriteIntData (uint16_t RegisterAddress, int value, bool GetReceived) // Write an int value to the driver
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
	//CRC=====END/
	// Prepare data frame -- END
	// Send data use UART5
	HAL_GPIO_WritePin(PE0_485_MCU_DRV_DIR_GPIO_Port, PE0_485_MCU_DRV_DIR_Pin, GPIO_PIN_RESET); //Switch to transmit mode
	HAL_UART_Transmit(&huart5,TxDataToDriver,sizeof(TxDataToDriver),200); // use UART5 to send
	if (GetReceived) // Get Received feedback if GetReceived = true
	{
		HAL_GPIO_WritePin(PE0_485_MCU_DRV_DIR_GPIO_Port, PE0_485_MCU_DRV_DIR_Pin, GPIO_PIN_SET);	//Switch back to receive mode
	}
}

void ReadDriverData(uint16_t RegisterAddress) // Read 1 register from the Driver
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
	//CRC=====END/
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
	/// Dubug END
}
///////////
void ReadMultiRegister(uint16_t StartingAddress, uint8_t NoOfRegister) // Read data from the Driver
{
	// Prepare data frame -- BEGIN
	uint8_t TxDataToDriver[8]; // 8 bytes of data frame
	
	// Data preparation
	TxDataToDriver[0] = DriverID;//SerialID = 1 of the driver
	TxDataToDriver[1] = 3;//Read Regis, function code	
	TxDataToDriver[2] = RegisterAddress / 256; // Register Address High byte
  TxDataToDriver[3] = RegisterAddress % 256; // Register Address LOW byte
	TxDataToDriver[4] = 0; // Number of Register HIGH byte
	TxDataToDriver[5] = NoOfRegister; // Number of Register LOW byte
	
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
	//CRC=====END/
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
	/// Dubug END
}
void ReadSlaveData (uint8_t SlaveID, uint16_t RegisterAddress, uint8_t NoRegister)
{
		// Prepare data frame -- BEGIN
	uint8_t TxDataToDriver[8]; // 8 bytes of data frame
	
	// Data preparation
	TxDataToDriver[0] = SlaveID; // Byte 0: slave ID
	TxDataToDriver[1] = 3;//Read Regis, function code	
	TxDataToDriver[2] = RegisterAddress / 256; // Register Address High byte
  TxDataToDriver[3] = RegisterAddress % 256; // Register Address LOW byte
	TxDataToDriver[4] = 0; // Number of Registers HIGH byte
	TxDataToDriver[5] = NoRegister; // Number of Register LOW byte
	
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
	//CRC=====END/
	// Prepare data frame -- END
	// Send data use UART5
	HAL_GPIO_WritePin(PE0_485_MCU_DRV_DIR_GPIO_Port, PE0_485_MCU_DRV_DIR_Pin, GPIO_PIN_RESET); //Switch to transmit mode
//	HAL_Delay(1);
	HAL_UART_Transmit(&huart5,TxDataToDriver,sizeof(TxDataToDriver),200); // use UART5 to send
	HAL_GPIO_WritePin(PE0_485_MCU_DRV_DIR_GPIO_Port, PE0_485_MCU_DRV_DIR_Pin, GPIO_PIN_SET);	//Switch back to receive mode
	HAL_UART_Receive_IT(&huart5,&RxDriverData,1); // Receive 1 byte each time
}

void LoadSavedParam (uint32_t StartSectorAddress, float *_Param)
{
	uint8_t LoadDataBuff[50];
	
	Flash_Read_Data(StartSectorAddress, (uint32_t *)LoadDataBuff, numofwords);
	
	uint8_t	j = 0;
	char *token;
	token = strtok((char *)LoadDataBuff, "/");	// Split the command ~ remove the / character
	while (token != NULL)
    {
				_Param[j] = (atof(token)); // covert to float type 
        token = strtok(NULL, "/");
				j++;
    }
}

void SaveParams(float _DrumRadius, float _DroppingDistance, int _PullingSpeed, uint16_t _StoppingTime, float _AccRefDropping, uint8_t _SampleTime)
{
	char buffer[40];
	TxPCLen = sprintf(buffer,"%.2f/%.1f/%d/%d/%.2f/%d",_DrumRadius, _DroppingDistance, PullingSpeed, _StoppingTime, _AccRefDropping,_SampleTime); // Combine to a string
	numofwords = (strlen(buffer)/4)+((strlen(buffer)%4)!=0);
  Flash_Write_Data(MemoryAddress , (uint32_t *)buffer, numofwords);	
}
void SaveSystemParams ()
{
	char SaveBuffer[60];
	TxPCLen = sprintf(SaveBuffer,"%.2f/%d/%d/%d/%.2f/%d/%d/%.2f/%d/%.2f/%.2f"
	                   ,DrumRadius, DroppingDistance, PullingSpeed, StoppingTime, AccRefDropping, SampleTime,
                    	PullingDistance, AccRefPulling,
											PD_Distance, PD_PullAccRef, PD_DropAccRef); // Combine to a string
	numofwords = (strlen(SaveBuffer)/4)+((strlen(SaveBuffer)%4)!=0);
  Flash_Write_Data(MemoryAddress , (uint32_t *)SaveBuffer, numofwords);	
}
float LinearSpeedGeneration (int Time, float _epsilon, int InitialSpeed, int Min, int Max)
{
	float LinearSpeed;
	LinearSpeed = (InitialSpeed + (float)(_epsilon*Time*0.01)); // to rpm
	if (LinearSpeed >= Max)
	{
		LinearSpeed = Max;
	}
	if (LinearSpeed <= Min)
	{
		LinearSpeed = Min;
	}
	return LinearSpeed;
}

void StopPulseGenerating()
{	
	PulseGenerationFlag = false; // PR phase is 90 deg late
	HAL_TIM_Base_Stop_IT(&htim3); // Disable Timer3
	HAL_GPIO_WritePin(PE9_TIM1_CH1_PFIN_GPIO_Port, PE9_TIM1_CH1_PFIN_Pin,GPIO_PIN_RESET);//Reset Pin status, change RESET to SET if the direction is reverse
	HAL_GPIO_WritePin(PC8_PR_GPIO_Port,PC8_PR_Pin, GPIO_PIN_RESET);//Reset Pin status	
}
void StartPulseGenerating()
{
	HAL_GPIO_WritePin(PC8_PR_GPIO_Port, PC8_PR_Pin, GPIO_PIN_SET); // Set CW direction	
	HAL_GPIO_WritePin(PE9_TIM1_CH1_PFIN_GPIO_Port, PE9_TIM1_CH1_PFIN_Pin,GPIO_PIN_SET);
	PulseGenerationFlag = true; // PR phase is 90 deg late
	HAL_TIM_Base_Start_IT(&htim3); // Enable Timer3		
}
bool WaitBeforeRunning(uint16_t TimeInMiliSecond)
{
	StoppingTimeCount++;
	if (StoppingTimeCount >= (TimeInMiliSecond/SampleTime))
	{
		StoppingTimeCount = 0;
		return true;
	}
	return false;
}
bool CheckGoingDownToBottom() // return true when finish going down, else return false;
{	
	if (8*PositionPulseCmd >= BotomPulseCmdPosition) // Reach the bottom position
		{			
			StopPulseGenerating();				
			return true;			
		}
	return false;
}
bool PullingExperiment ()
{
	if (!StartAccleratePulling) 
	{
		if (CheckGoingDownToBottom()) // if at the bottom position, then wait for some seconds
		{
			if (WaitBeforeRunning(StoppingTime)) // Wait for some seconds
			{
				StartAccleratePulling = true; // turn on flag to start acclerating pulling
				Direction = false;
				StartBraking = false;
				
				PRIsToggled = true; // true = pulling up.
				DisableSTOP(); // Disable the stop
				StartPulseGenerating();			
			}
			else return false;
		}
		else return false;
	}
	else // Start accelerate pulling
	{
		if (!StartBraking) // Accelerating Stage
		{
			// Calculate speed cmd
			RunningTime += SampleTime;
			SpeedCmd = LinearSpeedGeneration(RunningTime,-EpsilonPulling,0,-PullingMaxSpeed,0); //-EpsilonPulling means the spd is negative
			if (SpeedCmd != 0)
			{									
				Timer3CountPeriod = (int)((float)(120000000.0/(fabs(SpeedCmd)*(float)EncoderResolution)) + 0.5);
			}
			else 
			{
				StopPulseGenerating();
			}
			if (fabs(SpeedCmd) >= PullingMaxSpeed)
			{
				RunningTime = 0;
				StartBraking = true;
			}
		}
		else // Braking Stage
		{
			RunningTime += SampleTime;
			SpeedCmd = LinearSpeedGeneration(RunningTime,kbrake*EpsilonPulling,-PullingMaxSpeed,-PullingMaxSpeed,0);
			if (SpeedCmd != 0)
			{									
				Timer3CountPeriod = (int)((float)(120000000.0/(fabs(SpeedCmd)*(float)EncoderResolution)) + 0.5);
			}
			else 
			{
				StopPulseGenerating();
			}	
			if (SpeedCmd >= 0) // Stop braking
			{
				RunningTime = 0;
				SpeedCmd = 0; // reset/ stop
					
				StartBraking = false;
				StartAccleratePulling = false;
				StopPulseGenerating();
				return true;
			}
		}	
	}	
	return false;		
}


bool Dropping(uint16_t StoppingDelayTime) // Dropping Program
// Mode = false -> Manual Running
// Mode = true -> Automatic Running
// return true if finishing, else return false while running
// StoppingDelayTime (ms): the time duration of stopping before pulling
{	
	if (StartDropping && !StartPulling) // Dropping Stage
	{
		if (!StartBraking) // Accelerating Stage
		{
			// Calculate speed cmd
			RunningTime += SampleTime;
			SpeedCmd = LinearSpeedGeneration(RunningTime,EpsilonDropping,0,0,DroppingMaxSpeed); //
			if (SpeedCmd != 0)
			{									
				Timer3CountPeriod = (int)((float)(120000000.0/(fabs(SpeedCmd)*(float)EncoderResolution)) + 0.5);
			}
			else 
			{
				StopPulseGenerating();
			}
			if (SpeedCmd >= DroppingMaxSpeed)
			{
				RunningTime = 0;
				StartBraking = true;
			}
		}
		else // Braking Stage
		{
			RunningTime += SampleTime;
			SpeedCmd = LinearSpeedGeneration(RunningTime,-kbrake*EpsilonDropping,DroppingMaxSpeed,0,DroppingMaxSpeed);
			if (SpeedCmd != 0)
			{									
				Timer3CountPeriod = (int)((float)(120000000.0/(fabs(SpeedCmd)*(float)EncoderResolution)) + 0.5);
			}
			else 
			{
				StopPulseGenerating();
			}			
			if (SpeedCmd <= 0) // Stop braking
			{
				RunningTime = 0;
				SpeedCmd = 0; // reset/ stop

				StartDropping = false; //	
				StartBraking = false;
				StopPulseGenerating();
			}
		}		
	}

	if (!StartDropping && !StartPulling) //
	{
//		POSReach = HAL_GPIO_ReadPin(CN1_47_INSPD_INPOS_GPIO_Port,CN1_47_INSPD_INPOS_Pin);					
//		if (POSReach) // Check if position is reached or not
//		{
			StoppingTimeCount++;
			if (StoppingTimeCount >= (StoppingDelayTime/SampleTime))
			{
				StoppingTimeCount = 0;
				
				// Change to pulling stage
				StartPulling = true;
				Timer3CountPeriod = (int)((float)(120000000.0/((float)PullingSpeed*(float)EncoderResolution)) + 0.5);
				// Start pulling to the home position
//				StepPulseCmd = (int)CurrentEncPulse/8; // calculate # of pulse cmd to return to the top postion
//				IsStepPulseCmd = true;
				PRIsToggled = true;	// true = Pulling	
				
				//Start Running
				Direction = false; // pulling up direction
				StartPulseGenerating();
//				DisableSTOP();
			}											
//		}
	}						

	if (!StartDropping && StartPulling) // Pulling Stage
	{	
		if (PositionPulseCmd <= 0)
		{
//			POSReach = HAL_GPIO_ReadPin(CN1_47_INSPD_INPOS_GPIO_Port,CN1_47_INSPD_INPOS_Pin);
//			if (POSReach) // Reaching to the top/home postion
//			{
				StartPulling = false;
				StopPulseGenerating();
				return true;
//			}			
		}
	}
	return false;
}
void InitializeRunning ()
{
	switch (ExperimentMode)
	{
		case 1: // Dropping Mode
			StartRunning = true;
			StartDropping = true;
			Direction = true; // false = move up, true = move down
			StartPulling = false;		
		
			PRIsToggled = false; // false = Dropping Down. change to true/false to change the direction: pulling or dropping
			DisableSTOP(); // Disable the stop
			StartPulseGenerating();
			break;
		case 2: // Pulling Mode
			StartRunning = true;
			if (PositionPulseCmd < BotomPulseCmdPosition) // Then going down to the bottom
			{
				StartAccleratePulling = false;
				Direction = true; // false = move up, true = move down
				// Start going down to the bottom position
				PRIsToggled = false; // false = Dropping Down
				DisableSTOP(); // Disable the stop
				Timer3CountPeriod = (int)((float)(120000000.0/((PullingSpeed)*(float)EncoderResolution)) + 0.5); // Set going down speed
				StartPulseGenerating();
			}
			else // Object is at the bottom, then start pulling up
			{
				StartAccleratePulling = true;
				StartBraking = false; // go to Accerlerating Stage
				Direction = false;
				
				PRIsToggled = true; // true = pulling up.
				DisableSTOP(); // Disable the stop
				StartPulseGenerating();
			}			
			break;
		default:
			break;
	}
}
void StopExperiment ()
{
	// Reset all the flag and state
	StartRunning = false;
	StartDropping = false;
	StartPulling = false;
	StopPulseGenerating();
	RunningTime = 0;
	Timer3CountPeriod = 0;
	SpeedCmd = 0;
}
void ProcessReceivedCommand () // Proceed the command from the UI
{
	//ExtractMotionCode(); // Extract data to MotionCode
	switch ((int)MotionCode[0])
	{
		case 0: //Emergency Stop
			if ((int)MotionCode[1] == 0) // 0/0
			{
				Estop(); // Estop button on the UI
				PulseGenerationFlag = false; // Stop generating pulses
				StartDropping = false;
				StartPulling = false;									
			}
			else {AlarmReset();}  // 0/1, alarm button
			break;
		case 1: // Stop button;
			if ((int)MotionCode[1] == 1) // 1/1
			{
				Stop();				
			}
			break;
		case 2: // Set Control Mode
			if ((int)MotionCode[1] == 1) // 2/1 position mode
					{
						PositionControlMode = true;
						DriverInit(); // Init Position Mode
						//SetPositionMode(); // Set to Position Mode
					} 
			else // 2/0 speed mode
					{
						PositionControlMode = false;
						//SetSpeedMode(); // Set to Speed Mode
					} 
			break;
		case 3: // Jog Control
			if ((int)MotionCode[1] == 1) // 3/1 move up button
				 {
					if (PositionControlMode) // If the control Mode is Position Mode
					{								
						Timer3CountPeriod = (int)((float)(120000000.0/((float)JogSpeed*(float)EncoderResolution)) + 0.5);						
						PRIsToggled = true; // PR phase is 90 deg late
						Direction = false; // false = move up
						StartPulseGenerating(); // Reset PF, PR + Enable Timer + PulseGeneratingFlag = true						
						DisableSTOP(); // Turn off STOP to run
					}
					else // Speed Mode
					{
						JogMoveUp(); // Disable the stop
					}					
				 } 
			else  // 3/0 move down button
				 {
					if (PositionControlMode) // If the control Mode is Position Mode
					{					
						Timer3CountPeriod = (int)((float)(120000000.0/((float)JogSpeed*(float)EncoderResolution)) + 0.5);	
						PRIsToggled = false; //
						Direction = true;
						StartPulseGenerating(); // Reset PF, PR + Enable Timer + PulseGeneratingFlag = true
						DisableSTOP();	// Turn off STOP to run					
					}
					else // Speed Mode
					{
						JogMoveDown(); // Disable the stop
					}					
				 }
			break;
		case 4: // Start Dropping Buton
			if ((int)MotionCode[1] == 1) // Start runing 
				{
					InitializeRunning ();			
				}
			else // Stop running
				{
					StopExperiment();					
				}
			break;
				
		case 5: // Set Jog Speed						
			if (PositionControlMode) // If it is the position control mode, then change the JogSpeed
			{
				JogSpeed = (int)(MotionCode[1]); // unit: rpm
				Timer3CountPeriod = (int)((float)(120000000.0/((float)JogSpeed*(float)EncoderResolution)) + 0.5);
				char JogSpeedBuff[10];
				TxPCLen = sprintf(JogSpeedBuff,"j%.de",JogSpeed);
				HAL_UART_Transmit(&huart6,(uint8_t *)JogSpeedBuff,TxPCLen,200); // Send to uart6 to check the params are set or not
				// = (60*10e6)/(JogSpeed*EncoderRelsolution*Timer3Period)
				// Where JogSpeed in rpm; EcoderRelsolution in pulses, Timer3Period in us
				// Timer3 period in us = 15 us
			}
			else // Speed control mode
			{
					if ((int)MotionCode[1] == 0) // Write int Value
					{
						WriteIntData((uint16_t)MotionCode[2], (uint16_t)MotionCode[3], true);
					}
					else // Write float Value
					{
						DroppingDistance = roundf(MotionCode[3] * 10)/10;
						WriteFloatData((uint16_t)MotionCode[2], DroppingDistance, true);						
					}
			}					
			break;
			
		case 6: // 6 request read speed data
			if((int)MotionCode[1] == 1) {UIDataRequest = true;} // 6/1 If the UI request data
			else {UIDataRequest = false; DataSendingTimeCount = 0;}
			break;
					
		case 7: // Set running parameter, save params			
			// Save to the memory
			//SaveParams(DrumRadius, DroppingDistance, PullingSpeed, StoppingTime, AccRefDropping, SampleTime);				
			SaveSystemParams();
			// Send back to the UI to check					
			char MessageBuffer[10];
			TxPCLen = sprintf(MessageBuffer,"r7/1e");
			HAL_UART_Transmit(&huart6,(uint8_t *)MessageBuffer,TxPCLen,100); // Send to uart6 to check the params are set or not
			break;
		
		case 8: // Request reading output data or not
			if((int)MotionCode[1] == 1) {OutputDataRequest = true;} // 8/1 = request
			else OutputDataRequest = false; // 8/0 = stop request
			break;
			
		case 10: // Load saved parameters					
			LoadSavedParam(MemoryAddress,Params);
			DrumRadius = Params[0];
			DroppingDistance = Params[1];
			PullingSpeed = Params[2];
			StoppingTime = Params[3];
			AccRefDropping = Params[4];
			SampleTime = Params[5];
		
			PullingDistance = Params[6];
			AccRefPulling = Params[7];
	
			PD_Distance = Params[8];
			PD_PullAccRef = Params[9];
			PD_DropAccRef = Params[10];
		
			char ParamBuffer[60];
			TxPCLen = sprintf(ParamBuffer,"p%.2f/%d/%d/%d/%.2f/%d/%d/%.2f/%d/%.2f/%.2fe"
	                   ,DrumRadius, DroppingDistance, PullingSpeed, StoppingTime, AccRefDropping, SampleTime,
                    	PullingDistance, AccRefPulling,
											PD_Distance, PD_PullAccRef, PD_DropAccRef); // Combine to a string
			HAL_UART_Transmit(&huart6,(uint8_t *)ParamBuffer,TxPCLen,200); // Send to uart6 to check the params are set or not
			break;
		
		case 11: // Set Drum Radius
			DrumRadius = MotionCode[1];
			DroppingMaxSpeed = (uint16_t)(10*sqrt(2*AccRefDropping*DroppingDistance)/(DrumRadius));
			char AccTimeBuffer[10];
			TxPCLen = sprintf(AccTimeBuffer,"r11/%.2fe",DrumRadius);
			HAL_UART_Transmit(&huart6,(uint8_t *)AccTimeBuffer,TxPCLen,200); // Send to uart6 to check the params are set or not
			break;
		
		case 12: // Set DroppingDistance
			DroppingDistance = MotionCode[1];
			DroppingMaxSpeed = (uint16_t)(10*sqrt(2*AccRefDropping*DroppingDistance)/(DrumRadius));
			char DroppingDistanceBuffer[10];
			TxPCLen = sprintf(DroppingDistanceBuffer,"r12/%de",DroppingDistance);
			HAL_UART_Transmit(&huart6,(uint8_t *)DroppingDistanceBuffer,TxPCLen,200); // Send to uart6 to check the params are set or not
			break;
		
		case 13: // Set PullingSpeed
			PullingSpeed = MotionCode[1];
			char PullingSpeedBuffer[10];
			TxPCLen = sprintf(PullingSpeedBuffer,"r13/%de",PullingSpeed);
			HAL_UART_Transmit(&huart6,(uint8_t *)PullingSpeedBuffer,TxPCLen,200); // Send to uart6 to check the params are set or not
			break;
		
		case 14: // also start running
			InitializeRunning ();	
			break;
		case 15: // Set AccRefDropping
			AccRefDropping = MotionCode[1];
			EpsilonDropping = AccRefDropping/DrumRadius;
			DroppingMaxSpeed = (uint16_t)(10*sqrt(2*AccRefDropping*DroppingDistance)/(DrumRadius));
			char AccRefDroppingBuffer[10];
			TxPCLen = sprintf(AccRefDroppingBuffer,"r15/%.3fe",AccRefDropping);
			HAL_UART_Transmit(&huart6,(uint8_t *)AccRefDroppingBuffer,TxPCLen,200); // Send to uart6 to check the params are set or not
			break;
		
		case 16: // Set SampleTime
			SampleTime = MotionCode[1];
		if (SampleTime<= 20) // ms Set value range, 20:100ms
			{
				SampleTime = 20;
			}
			if (SampleTime >= 100) // ms
			{
				SampleTime = 100;
			}
			char SammpleTimeBuffer[10];
			TxPCLen = sprintf(SammpleTimeBuffer,"r16/%de",SampleTime);
			HAL_UART_Transmit(&huart6,(uint8_t *)SammpleTimeBuffer,TxPCLen,200); // Send to uart6 to check the params are set or not
			break;
		
		case 17: // Reset MCU
			HAL_NVIC_SystemReset();
			break;
		case 18: // Servo Enable on/off
			if (MotionCode[1] == 1) // Servo Enable ON
				HAL_GPIO_WritePin(SerVoReset_PC4_18_GPIO_Port, SerVoReset_PC4_18_Pin, GPIO_PIN_SET); // Servo enable on
			else
				HAL_GPIO_WritePin(SerVoReset_PC4_18_GPIO_Port, SerVoReset_PC4_18_Pin, GPIO_PIN_RESET); // Servo enable OFF
			break;
		case 19: // PF on/off
			if (MotionCode[1] == 1) // PF ON
				HAL_GPIO_WritePin(PE9_TIM1_CH1_PFIN_GPIO_Port, PE9_TIM1_CH1_PFIN_Pin, GPIO_PIN_SET); //
			else
				HAL_GPIO_WritePin(PE9_TIM1_CH1_PFIN_GPIO_Port, PE9_TIM1_CH1_PFIN_Pin, GPIO_PIN_RESET); // 
			break;
		case 20: // Set Step Pulse Cmd
			StepPulseCmd = MotionCode[2];
			PulseCmd = 0;
			if (MotionCode[1] == 1)
			{				
				PRIsToggled = false;
				IsStepPulseCmd = true;
				Timer3CountPeriod = (int)((float)(120000000.0/((JogSpeed)*(float)EncoderResolution)) + 0.5);
				//Start Running
				StartPulseGenerating();
				DisableSTOP();
			}
			else
			{				
				PRIsToggled = true;
				IsStepPulseCmd = true;
				Timer3CountPeriod = (int)((float)(120000000.0/((JogSpeed)*(float)EncoderResolution)) + 0.5);
				//Start Running
				StartPulseGenerating();
				DisableSTOP();
			}		  
			break;
		case 21: // PLSCLR on/off - CN1 14
			if (MotionCode[1] == 1) // ON
				HAL_GPIO_WritePin(Dir_Not_PE10_14_GPIO_Port, Dir_Not_PE10_14_Pin, GPIO_PIN_RESET); // CN1-14 - PLSCLR
			else
				HAL_GPIO_WritePin(Dir_Not_PE10_14_GPIO_Port, Dir_Not_PE10_14_Pin, GPIO_PIN_SET); // CN1-14 - PLSCLR
			break;
		case 22: // SPDLIM on/off CN1 - 15
			if (MotionCode[1] == 1) // ON
				HAL_GPIO_WritePin(Speed2_Not_PE7_15_GPIO_Port,Speed2_Not_PE7_15_Pin,GPIO_PIN_RESET);//CN1-15 SPDLIM/TLIM
			else
				HAL_GPIO_WritePin(Speed2_Not_PE7_15_GPIO_Port,Speed2_Not_PE7_15_Pin,GPIO_PIN_SET);//CN1-15 SPDLIM/TLIM
			break;
		case 23: // PLSINH CN1 - 39
			if (MotionCode[1] == 1) // ON
				HAL_GPIO_WritePin(CCWLIM_Not_PE12_39_GPIO_Port,CCWLIM_Not_PE12_39_Pin,GPIO_PIN_RESET);//CN1-39 PLSINH
			else
				HAL_GPIO_WritePin(CCWLIM_Not_PE12_39_GPIO_Port,CCWLIM_Not_PE12_39_Pin,GPIO_PIN_SET);//CN1-39 PLSINH
			break;
		case 24: // CWLIM CN1 - 38
			if (MotionCode[1] == 1) // ON
				HAL_GPIO_WritePin(SPDLIM_Not_PE11_38_GPIO_Port, SPDLIM_Not_PE11_38_Pin, GPIO_PIN_RESET);// CN-38 - CWLIM
			else
				HAL_GPIO_WritePin(SPDLIM_Not_PE11_38_GPIO_Port, SPDLIM_Not_PE11_38_Pin, GPIO_PIN_SET);// CN-38 - CWLIM
			break;
		case 25: // CCWLIM CN1 - 13
			if (MotionCode[1] == 1) // ON
				 HAL_GPIO_WritePin(CWLIM_Not_PE14_13_GPIO_Port,CWLIM_Not_PE14_13_Pin,GPIO_PIN_RESET);//CN1-13 CCWLIM
			else
				 HAL_GPIO_WritePin(CWLIM_Not_PE14_13_GPIO_Port,CWLIM_Not_PE14_13_Pin,GPIO_PIN_SET);//CN1-13 CCWLIM
			break;
		case 26: // DIR CN1 - 40
			if (MotionCode[1] == 1) // ON
				 HAL_GPIO_WritePin(Type_Not_PE8_40_GPIO_Port, Type_Not_PE8_40_Pin, GPIO_PIN_RESET); // DIR	
			else
				 HAL_GPIO_WritePin(Type_Not_PE8_40_GPIO_Port, Type_Not_PE8_40_Pin, GPIO_PIN_SET); // DIR	
			break;
		case 27: // Set Running Mode
			if (MotionCode[1] == 1) // RunningMode = true => Automatic Running
				 RunningMode = true; // Automatic
			else
				 RunningMode = false; // Manual
			break;
		case 28: // Stop jog move up/down in Position Jog control;
			StopPulseGenerating();			
			//HAL_TIM_IC_Stop()
			break;
		case 30: // Set Driver parameters
			       // 30/data type/adress/value
						 // datatype: 1-> float; 0-> int
			if (MotionCode[1] == 1) // Write float value
			{
				WriteFloatData(MotionCode[2], MotionCode[3], true);
			}
			else // Write Int Value
			{
				WriteIntData(MotionCode[2], MotionCode[3], true);
			}
			break;
		case 31: // Set Experiment Mode
			ExperimentMode = MotionCode[1]; // 1=Dropping Mode;2 = Pulling; 3= Pulling->Dropping
			char SetModeBuff[8];
			TxPCLen = sprintf(SetModeBuff,"m%de",ExperimentMode);
			HAL_UART_Transmit(&huart6,(uint8_t *)SetModeBuff,TxPCLen,100); // Send to uart6 to check the params are set or not	
			break;
		case 32: // Set Pulling Distance, Pulling Mode
			PullingDistance = MotionCode[1];
			PullingMaxSpeed = (uint16_t)(10*sqrt(2*AccRefPulling*PullingDistance)/(DrumRadius));
			char PullingDistanceBuffer[10];
			TxPCLen = sprintf(PullingDistanceBuffer,"r32/%de",PullingDistance);
			HAL_UART_Transmit(&huart6,(uint8_t *)PullingDistanceBuffer,TxPCLen,100); // Send to uart6 to check the params are set or not			
			break;
		case 33: // Set Pulling AccRef; Pulling Mode
			AccRefPulling = MotionCode[1];
			EpsilonPulling = AccRefPulling/DrumRadius;
			PullingMaxSpeed = (uint16_t)(10*sqrt(2*AccRefPulling*PullingDistance)/(DrumRadius));
			char AccRefPullingBuffer[10];
			TxPCLen = sprintf(AccRefPullingBuffer,"r33/%.3fe",AccRefPulling);
			HAL_UART_Transmit(&huart6,(uint8_t *)AccRefPullingBuffer,TxPCLen,100); // Send to uart6 to check the params are set or not
			break;
		case 34: // Set PD Distance
			PD_Distance = MotionCode[1];
			PD_DroppingMaxSpeed = (uint16_t)(10*sqrt(2*PD_DropAccRef*PD_Distance)/(DrumRadius));
			PD_PullingMaxSpeed = (uint16_t)(10*sqrt(2*PD_PullAccRef*PD_Distance)/(DrumRadius));
			char PDDistanceBuffer[10];
			TxPCLen = sprintf(PDDistanceBuffer,"r34/%de",PD_Distance);
			HAL_UART_Transmit(&huart6,(uint8_t *)PDDistanceBuffer,TxPCLen,100); // Send to uart6 to check the params are set or not			
			break;
		case 35: // Set PD PullingAccRef
			PD_PullAccRef = MotionCode[1];
			PD_PullingMaxSpeed = (uint16_t)(10*sqrt(2*AccRefPulling*PD_Distance)/(DrumRadius));
			PD_EpsilonPull = PD_PullAccRef/DrumRadius;
			char PD_AccRefPullingBuffer[10];
			TxPCLen = sprintf(PD_AccRefPullingBuffer,"r35/%.3fe",PD_PullAccRef);
			HAL_UART_Transmit(&huart6,(uint8_t *)PD_AccRefPullingBuffer,TxPCLen,100); // Send to uart6 to check the params are set or not
			break;
		case 36: // Set PD DroppingAccRef
			PD_DropAccRef = MotionCode[1];
			PD_DroppingMaxSpeed = (uint16_t)(10*sqrt(2*AccRefDropping*PD_Distance)/(DrumRadius));
			PD_EpsilonDrop = PD_DropAccRef/DrumRadius;
			char PD_AccRefDroppingBuffer[10];
			TxPCLen = sprintf(PD_AccRefDroppingBuffer,"r36/%.3fe",PD_DropAccRef);
			HAL_UART_Transmit(&huart6,(uint8_t *)PD_AccRefDroppingBuffer,TxPCLen,100); // Send to uart6 to check the params are set or not
			break;
		case 37: // Set Stopping Time
			StoppingTime = MotionCode[1];
			if (StoppingTime <= 1000) // ms
			{
				StoppingTime = 1000;
			}
			if (StoppingTime >= 10000)
			{
				StoppingTime = 10000;
			}
			char StoppingTimeBuffer[10];
			TxPCLen = sprintf(StoppingTimeBuffer,"r37/%de",StoppingTime);
			HAL_UART_Transmit(&huart6,(uint8_t *)StoppingTimeBuffer,TxPCLen,200); // Send to uart6 to check the params are set or not
			break;
		default:
			break;		
	}
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
			if (_rxIndex >= 9) // 25 byte =  reading 5 registers, if only 1 register then 9 Complete receiving the data from the driver
				{					
					RxUart5_Cpl_Flag = true; // Complete Receive
					StartReceiveDriverData = false;
					_rxIndex = 0;
					HAL_UART_Receive_IT(&huart5,&RxDriverData,1); // Receive 1 byte for the next time
					return;					
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


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) // Timer 2 interrupt, 1ms
{
	if (htim->Instance == TIM3)	// TIMER 3 interrupt for pulse generation, period: 2us
	{
		if (PulseGenerationFlag) // Only generating pulse when the flag is ON. Otherwise, do nothing
		{
			if (Timer3CountPeriod>0)
			{
				Timer3Count++;
				if (Timer3Count >= Timer3CountPeriod) // Generate pulse
				{
					Timer3Count = 0;
					if (PRIsToggled)
					{
						HAL_GPIO_TogglePin(PE9_TIM1_CH1_PFIN_GPIO_Port, PE9_TIM1_CH1_PFIN_Pin); // Generate pulses on PF by tonggling this input
						PRIsToggled = false;					
						if (Direction) // Direction = true: dropping down
						{
							PositionPulseCmd++; // Increase the pulse cmd
						}
						else // false: pulling up
						{
							PositionPulseCmd--; // Decrease the pulse cmd
							if(PositionPulseCmd<=0 && StartPulling)
							{							
								StopPulseGenerating();
							}
						}										
						if (IsStepPulseCmd == true)
						{
							PulseCmd++;
							if (PulseCmd >= abs(StepPulseCmd)) // Pulse cmd is reached
							{
								StopPulseGenerating();
								IsStepPulseCmd = false;
								return;
							}
						}					
						return; // exit the function
					}				
					else
					{
						HAL_GPIO_TogglePin(PC8_PR_GPIO_Port, PC8_PR_Pin); // Generate pulses on PF by tonggling this input
						PRIsToggled = true;
						if (Direction) // Direction = true: dropping down
						{
							PositionPulseCmd++; // Increase the pulse cmd
						}
						else // false: pulling up
						{
							PositionPulseCmd--; // Decrease the pulse cmd
							if(PositionPulseCmd<=0 && StartPulling) 
							{
								StopPulseGenerating();
							}					
						}					
						if (IsStepPulseCmd == true)
						{
							PulseCmd++;
							if (PulseCmd >= abs(StepPulseCmd)) // Pulse cmd is reached
							{
								StopPulseGenerating();
								IsStepPulseCmd = false;
								return;
							}
						}
						return;
					}
				}				
			}
		}
	}

	if (htim->Instance == TIM2) // Timer 2 interrupt, for the main control function
		{
				Timer2Count++;
				if (Timer2Count >= SampleTime) // turn on the flag when the sample time reaches
				{		
					Timer2SampleTimeInterrupt = true;
					Timer2Count = 0;					
				}				
				if(OutputDataRequest)
					{
						CountTimerDriverOutput++;
						if (CountTimerDriverOutput >= 500) // 500 ms
						{
							DriverOutput = ReadLogicF7000Out(); // Read Driver Output
							
							memset (TxPCBuff, '\0', sizeof (TxPCBuff)); // reset
							TxPCLen = sprintf(TxPCBuff,"o%de",DriverOutput); // 1 means only the driver outputs
							HAL_UART_Transmit(&huart6,(uint8_t *)TxPCBuff,TxPCLen,200); // use uart6 to send
							
							CountTimerDriverOutput = 0;
						}
					}
//				if (UIDataRequest) // If the UI request the data, speed and encoder data
//					{
//						DataSendingTimeCount++;
//						if (DataSendingTimeCount >= SampleTime) // send the data each 50ms
//						{
//							DataSendingTimeCount = 0; // reset counter
//													
//						}
//					}
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
  MX_TIM3_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(PE15_RELAY1_GPIO_Port, PE15_RELAY1_Pin, GPIO_PIN_SET);
	HAL_Delay(5000);
	
//	// Load Parameters from the memory
	LoadSavedParam(MemoryAddress,Params);
	DrumRadius = Params[0]; 
	DroppingDistance = Params[1];
	PullingSpeed = Params[2];
	StoppingTime = Params[3];
	AccRefDropping  = Params[4];
	EpsilonDropping = AccRefDropping/DrumRadius;
	SampleTime = Params[5];
	DroppingMaxSpeed = (uint16_t)(10*sqrt(2*AccRefDropping*DroppingDistance)/(DrumRadius)); // in rpm
	
	PullingDistance = Params[6];
	TotalPullingDistance = (uint16_t)((float)PullingDistance + (float)(PullingDistance/kbrake))*(float)1.25;
	BotomPulseCmdPosition = (int)((float)EncoderResolution*(float)TotalPullingDistance/((float)(2*3.14*DrumRadius)));
	AccRefPulling = Params[7];
	EpsilonPulling = AccRefPulling/DrumRadius;
	PullingMaxSpeed = (uint16_t)(10*sqrt(2*AccRefPulling*DroppingDistance)/(DrumRadius)); // in rpm
	
	PD_Distance = Params[8];
	PD_PullAccRef = Params[9];
	PD_DropAccRef = Params[10];
	PD_EpsilonPull = PD_PullAccRef/DrumRadius;
	PD_EpsilonDrop = PD_DropAccRef/DrumRadius;
// End load data

// Init PID controller
	// Assume that FeedForwardPulse is the feedback signal to check the PID calculations
	//PID(&TPID, &AccFb, &PIPulseCmd, &AccRefDropping, Kp, StoppingTime, 0, _PID_P_ON_E, _PID_CD_DIRECT);
	
	//PID(&TPID, &AccFb, &PIPulseCmd, &AccRefDropping, Kp, StoppingTime, 0, _PID_P_ON_E, _PID_CD_DIRECT); // Kd = 0, use PI controller	
//  PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);
//  PID_SetSampleTime(&TPID, Timer2Period); // the sample time is 50ms = Timer2 time interval
//  PID_SetOutputLimits(&TPID, -2000, 2000); // min PID: -2000rpm, max: 2000rpm
	

	HAL_TIM_Base_Start_IT(&htim2); // Enable Timer 2 interrupt
// Not turn on timer3 at the start
//	HAL_TIM_Base_Start_IT(&htim3); // Enable Timer 3 interrupt
	HAL_UART_Receive_IT(&huart6,&RxPCData,1);
	DriverInit();
	//ReadDriverData(StE07); // Read speed

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(RxUart6_Cpl_Flag) // If data is completely received, a command is sent from the UI
			{
				ExtractMotionCode();				
				ProcessReceivedCommand (); // Proceed the command
				RxUart6_Cpl_Flag=false;
			}
			
		if (Timer2SampleTimeInterrupt)
		{
			Timer2SampleTimeInterrupt = false;
			if (StartRunning)
			{
				switch (ExperimentMode)
				{
					case 1: // Dropping Mode
						if (Dropping(StoppingTime)) // Dropping() return true when it finishing
						{
							StopExperiment();
							if (RunningMode) // Running Mode = false = manual, true=Automatic
							{
								memset (TxPCBuff, '\0', sizeof (TxPCBuff)); // reset
								TxPCLen = sprintf(TxPCBuff,"$"); // $ means finish running one episode
								HAL_UART_Transmit(&huart6,(uint8_t *)TxPCBuff,TxPCLen,200); // use uart6 to send
								HAL_Delay(20);
							}
						}
						break;
					case 2: // Pulling Mode
						if (PullingExperiment()) // PullingExperiment() return true when it finishing
						{
							StopExperiment();
							if (RunningMode) // Running Mode = false = manual, true=Automatic
							{
								memset (TxPCBuff, '\0', sizeof (TxPCBuff)); // reset
								TxPCLen = sprintf(TxPCBuff,"$"); // $ means finish running one episode
								HAL_UART_Transmit(&huart6,(uint8_t *)TxPCBuff,TxPCLen,200); // use uart6 to send
							}
						}
						break;
					case 3: // Pulling -> Dropping Mode
						break;
					default:
						break;
				}
			}
			if (UIDataRequest)
			{
				if (PositionControlMode) // Position Mode, read both Position and Speed, Send both Position and Speed
				{
					memset (TxPCBuff, '\0', sizeof (TxPCBuff)); // reset
					TxPCLen = sprintf(TxPCBuff,"s%.1f/%.1f/%d/%de",MotorSpeed,SpeedCmd,CurrentEncPulse,PositionPulseCmd*EgearRatio); // s means speed
					//TxPCLen = sprintf(TxPCBuff,"s%.1f/%.1f/%de",MotorSpeed,SpeedCmd,PositionPulseCmd*EgearRatio); // 8 is the Egear ratio 
					//TxPCLen = sprintf(TxPCBuff,"s2/%de",PulseCmd);
					HAL_UART_Transmit(&huart6,(uint8_t *)TxPCBuff,TxPCLen,200); // use uart6 to send
					//ReadMultiRegister(StE03,5); // Read from StE03 -> StE07
					
//					IsReadMotorSpeed = false;
//					IsReadEncoderPulse = true;
//					ReadDriverData(StE07); // Read the motor position	

					IsReadMotorSpeed = true;
					IsReadEncoderPulse = false;						
					ReadDriverData(StE03); // Read the motor speed.					
								
//					if(ToggleReadingData)
//					{
//						IsReadMotorSpeed = true;
//						IsReadEncoderPulse = false;						
//						ReadDriverData(StE03); // Read the motor speed.
//					}
//					else
//					{						
//						IsReadEncoderPulse = true; // Switch to reading pulse
//						IsReadMotorSpeed = false;
//						ReadDriverData(StE07); // Read the motor position.						
//					}
				}
				else // Speed Mode
				{
					memset (TxPCBuff, '\0', sizeof (TxPCBuff)); // reset
					TxPCLen = sprintf(TxPCBuff,"s%.1f/%.1fe",MotorSpeed, SpeedCmd); // s means speed, 2 means only the motor speed
					//TxPCLen = sprintf(TxPCBuff,"s2/%de",PulseCmd);
					HAL_UART_Transmit(&huart6,(uint8_t *)TxPCBuff,TxPCLen,200); // use uart6 to send
					
					IsReadMotorSpeed = true;
					IsReadEncoderPulse = false;
					ReadDriverData(StE03); // Read the motor speed.	
				}
			}
			
			if (RxUart5_Cpl_Flag) // Complete receive data from the driver
			{
				RxUart5_Cpl_Flag = false;
				if (RxDriverBuff[1] == 3) // Read Holding register
				{						
					if (IsReadEncoderPulse)
						{
							CurrentEncPulse = (RxDriverBuff[3] << 24) | (RxDriverBuff[4] << 16) | (RxDriverBuff[5] << 8) | RxDriverBuff[6];																
							memset (RxDriverBuff, '\0', sizeof (RxDriverBuff)); // reset buffer
							ToggleReadingData = !ToggleReadingData;								
						}
					if (IsReadMotorSpeed)
						{
							SpeedValueRegion[0] = RxDriverBuff[6];
							SpeedValueRegion[1] = RxDriverBuff[5];
							SpeedValueRegion[2] = RxDriverBuff[4];
							SpeedValueRegion[3] = RxDriverBuff[3];	

							//MotorSpeed = *(float *)&SpeedValueRegion;	
							memcpy(&MotorSpeed, SpeedValueRegion, 4);
							memset (RxDriverBuff, '\0', sizeof (RxDriverBuff)); // reset buffer
							ToggleReadingData = !ToggleReadingData;
						}						
				}
				if (RxDriverBuff[1] == 6) // Writing to a register
				{
					// Send to PC to check the writing result
					memset (TxPCBuff, '\0', sizeof (TxPCBuff)); // reset
					TxPCLen = sprintf(TxPCBuff,"w%d/%d/%d/%d/%d/%d/%d/%d/%d/e",RxDriverBuff[0],RxDriverBuff[1],RxDriverBuff[2],RxDriverBuff[3],RxDriverBuff[4],RxDriverBuff[5],RxDriverBuff[6],RxDriverBuff[7],RxDriverBuff[8]); 
					HAL_UART_Transmit(&huart6,(uint8_t *)TxPCBuff,TxPCLen,200); // use uart6 to send
				}
			}
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
  htim2.Init.Prescaler = 10;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 168;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, PC3_ZIGBEE_nRST_Pin|PC8_PR_Pin|PC9_ZIGBEE_HGM_EN_Pin|PC10_SPI3_SCK_SPARE_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : PC3_ZIGBEE_nRST_Pin SerVoReset_PC4_18_Pin Stop_PC5_43_Pin PC8_PR_Pin
                           PC9_ZIGBEE_HGM_EN_Pin PC10_SPI3_SCK_SPARE_Pin */
  GPIO_InitStruct.Pin = PC3_ZIGBEE_nRST_Pin|SerVoReset_PC4_18_Pin|Stop_PC5_43_Pin|PC8_PR_Pin
                          |PC9_ZIGBEE_HGM_EN_Pin|PC10_SPI3_SCK_SPARE_Pin;
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
