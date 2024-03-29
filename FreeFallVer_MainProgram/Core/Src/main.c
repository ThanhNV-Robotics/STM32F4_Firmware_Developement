/* USER CODE BEGIN Header */
/**
 *
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
		volatile uint8_t RxDriverBuff[30]; // buffer to read the data from the driver
		//volatile uint8_t RxUart3Buff[10];

		//uint8_t AccZDataRegion[10];

		char DataRegion[40]; // array to filt the data from the PC
		uint8_t RxPCData; // variable to save 1 byte received from the PC
		uint8_t RxDriverData; // variable to save 1 byte received from the driver
		char TxPCBuff[90]; // buffer to send message to the PC
		uint8_t TxDataToDriver[8];
		uint8_t TxPCLen;
		uint8_t i;
		char SpeedValueRegion[4];
		char ResponseMess[15];

		uint8_t _rxPCIndex; // index for receiving data from the driver, uart6
		uint8_t _rxDriverIndex; // uart5

		uint8_t NoOfBytes = 29; // FDA7000, read 6 registers

		// Flag variables
		volatile  bool RxUart6_Cpl_Flag= false; // uart6 receiving complete flag (from the PC)
		volatile  bool RxUart5_Cpl_Flag= false; // uart5 receiving complete flag (from the Driver)

		volatile bool RxUart3_Cpl_Flag = false;
		volatile bool StartReceiveDriverData = false; // to check if start saving the driver data to the buffer RxDriverBuff or not

		volatile bool Timer2SampleTimeInterrupt;
		volatile bool Timer2ControlInterrupt;

		volatile bool direction = false;

		bool StartRunning;
		bool StartMainRun = false;
		bool StartSimulating = false;
		bool Direction; // false = move up, true = move down

		bool UIDataRequest = false; // to check if the GUI request data or not
		bool OutputDataRequest = true; // default is true
		bool PositionControlMode = true; // Position Mode is default

		bool PulseGenerationFlag = false;
		bool POSReach =  false; // position reach flag

		volatile bool IsReachTargetPosition = false;

		bool CompleteDropping;
		bool CompletePulling;

		bool MotorDriver = true; // true = FDA7000 (15kw); false = ASDA-A3 (200W)

		bool IsHoming = false;

		bool Initialized = false; // indicate that if the experiment is initialized or not, false = no


		bool SoftWareLimit = false; // the software limit is turned on in default


		bool PRIsToggled; // false = down, true = up

		bool IsGoingToBottom = false;


		volatile bool IsPulseCheck = false;
		volatile bool IsGoingToInitPoint = false;

		uint16_t DropCurveDataIndex = 0;
		int DropRefPosDeltaPulse[6000];

		uint8_t ExperimentMode = 0; // 1: Dropping Mode, 2: Pulling Mode, 3: Pulling->Dropping Mode
		uint8_t Timer2Count;
		uint8_t Timer2SampleTimeControlCount;
		uint16_t WaitingTime; // waiting time in pull and Drop experiment

		uint16_t CountTimerDriverOutput = 0;
		uint16_t DriverOutput; // to save the driver output
		uint16_t JogSpeed = 30; // Control the Jog Speed 30 rpm is default

		uint16_t Timer3CountPeriod; //
		uint16_t Timer3Count;
		uint16_t StoppingTimeCount;

		uint32_t TotalPullingPulse;
		uint32_t TotalDroppingPulse;

		uint8_t EgearRatio = 8;

		uint16_t Index;

		float DrumRadius;
		uint8_t SampleTime; // sample time in ms
		uint8_t PullingSpeed; // rpm, pulling speed, and going down Speed
		uint16_t StoppingTime = 5000; // ms

		uint16_t EncoderResolution = HigenEncoderResolution;

		float GoingAcceleration;

		float PullingTotalDistance; // m
		uint32_t PullingBotomPulseCmdPosition ; // pulses

		int OriginPulse; // To save the postion of the origin
		int AbsoluteTargetMotorPosition;
		volatile int TargetPosition;
		volatile int PulseSimuCount;
		volatile int PositionPulseCmd;
		volatile int PositionSimCmd;

		uint8_t numofwords = 24; // The number of words To save to flash memory
		float Params[19] = {0, 0, 0, 0, 0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0};

		float MotorSpeed; // variable to save motor speed, it's value is changed in uart5 interrupt => volatile type
		float MotorSpeedBuff;
		int MotorEncPulse;
		float PositionCmd;
		float SpeedCmd;

		const uint8_t EndChar = '$'; // Character to determine the ending of a data frame from the PC (GUI)
		const float GravityConst = -9.8;
		float MotionCode[8]; // array to save the command from the PC

		float DistCoeff; // To estimate the bottom position in pulling task
		uint8_t DropDecelSlope;
		uint8_t DropAccelSlope;
		//float AccZ;
		float AccRef = GravityConst; // Reference accleraion, initial value is GravityConst
		float ObjectPosition;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_UART5_Init(void);
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


void ReadMultiRegister(uint16_t StartingAddress, uint8_t NoOfRegister) // Read data from the Driver
{
	// Prepare data frame -- BEGIN
	// Data preparation
	TxDataToDriver[0] = DriverID;//SerialID = 1 of the driver
	TxDataToDriver[1] = 3;//Read Regis, function code
	TxDataToDriver[2] = StartingAddress / 256; // Register Address High byte
    TxDataToDriver[3] = StartingAddress % 256; // Register Address LOW byte
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
	HAL_Delay(1);
	HAL_UART_Transmit(&huart5,TxDataToDriver,8,200); // use UART5 to send
	HAL_GPIO_WritePin(PE0_485_MCU_DRV_DIR_GPIO_Port, PE0_485_MCU_DRV_DIR_Pin, GPIO_PIN_SET);	//Switch back to receive mode
	HAL_UART_Receive_IT(&huart5,&RxDriverData,1); // Receive 1 byte each time
}

void WriteFloatData (uint16_t RegisterAddress, float value) // Write a float value to the driver
{
	// Prepare data frame -- BEGIN
	uint8_t _TxDataToDriver[10]; // 10 bytes of data

	// Data preparation
	_TxDataToDriver[0] = DriverID;//SerialID of the driver, 1
	_TxDataToDriver[1] = 6;//Write Regis, function code
	_TxDataToDriver[2] = RegisterAddress / 256; // Register Address High byte
	_TxDataToDriver[3] = RegisterAddress % 256; // Register Address LOW byte

	char FloatValue[sizeof(float)];
	memcpy(FloatValue, &value, sizeof(float)); // convert float to 4 bytes of char

	_TxDataToDriver[4] = FloatValue[3];
	_TxDataToDriver[5] = FloatValue[2];
	_TxDataToDriver[6] = FloatValue[1];
	_TxDataToDriver[7] = FloatValue[0];

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
	_TxDataToDriver[8]= (uint8_t)(crc&0x00FF);;//(uint8_t)(TemDat16&0xFF);
	_TxDataToDriver[9]=(uint8_t)((crc>>8)&0x00FF);
	//CRC=====END/
	// Prepare data frame -- END

	// Send data use UART5
	HAL_GPIO_WritePin(PE0_485_MCU_DRV_DIR_GPIO_Port, PE0_485_MCU_DRV_DIR_Pin, GPIO_PIN_RESET); //Switch to transmit mode
	HAL_Delay(1);
	HAL_UART_Transmit(&huart5,_TxDataToDriver,10,200); // use UART5 to send
	HAL_GPIO_WritePin(PE0_485_MCU_DRV_DIR_GPIO_Port, PE0_485_MCU_DRV_DIR_Pin, GPIO_PIN_SET);	//Switch back to receive mode
	HAL_UART_Receive_IT(&huart5,&RxDriverData,1); // Receive 1 byte each time
}


void LoadSavedParam (uint32_t StartSectorAddress, float *_Param)
{
	uint8_t LoadDataBuff[100];

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

void SaveSystemParams (uint8_t *NoOfWords)
{
	char SaveBuffer[80];
	TxPCLen = sprintf(SaveBuffer,"%.2f/%d/%d/%d"
	                   ,DrumRadius, PullingSpeed, StoppingTime, SampleTime); // Combine to a string
	*NoOfWords = (strlen(SaveBuffer)/4)+((strlen(SaveBuffer)%4)!=0);
	Flash_Write_Data(MemoryAddress , (uint32_t *)SaveBuffer, *NoOfWords);
}

void LinearGeneration (float *var, float slope, float Amplitude)
// Amplitude is the limit of the value
{

//	float ReturnValue;
//	ReturnValue = (InitialValue + (float)(slope*Time*0.001)); //
	if (slope >= 0)
	{
		if (*var >= Amplitude)
			{
				*var = Amplitude;
				return;
			}
	}
	else
	{
		if (*var <= Amplitude)
			{
				*var = Amplitude;
				return;
			}
	}
	*var += slope*SampleTime*0.001; // *0.001 to convert to second
}


void StopPulseGenerating()
{
	PulseGenerationFlag = false; //
	HAL_TIM_Base_Stop_IT(&htim3); // Disable Timer3
	HAL_GPIO_WritePin(PE9_TIM1_CH1_PFIN_GPIO_Port, PE9_TIM1_CH1_PFIN_Pin,GPIO_PIN_RESET);//Reset Pin status
	HAL_GPIO_WritePin(PC8_PR_GPIO_Port,PC8_PR_Pin, GPIO_PIN_RESET);//Reset Pin status
}
void InitPulseGenerating(bool _Direction)
{
	if(_Direction)
	{
		PRIsToggled = true; // go up
	}
	else
	{
		PRIsToggled = false; // go down
	}
	HAL_GPIO_WritePin(PC8_PR_GPIO_Port, PC8_PR_Pin, GPIO_PIN_SET); //
	HAL_GPIO_WritePin(PE9_TIM1_CH1_PFIN_GPIO_Port, PE9_TIM1_CH1_PFIN_Pin,GPIO_PIN_SET);
	HAL_TIM_Base_Start_IT(&htim3); // Enable Timer3
}
bool WaitingMiliSecond(uint16_t TimeInMiliSecond)
{
	StoppingTimeCount++;
	if (StoppingTimeCount >= (uint16_t)(TimeInMiliSecond/SampleTime))
	{
		StoppingTimeCount = 0;
		return true;
	}
	return false;
}

int CalculateTimer3Period (bool DriverType, float speed)
{
	// DriverType = true ->  Higen FDA7000 Driver
	// DriverType = false -> ASDA A3 Driver
	// Speed in rpm
	if (DriverType) // Higen FDA7000 Driver
	{
		return (int)((float)(120000000.0/(fabs(speed)*(float)EncoderResolution)) + 0.5); // Set going speed
	}
	else
	{
		return (int)((float)(15000000.0/(fabs(speed)*(float)EncoderResolution)) + 0.5); // Set going speed
	}
}

void PulseGenerating(int DeltaPulse, uint8_t PulseCountPeriod)
{
	if (DeltaPulse == 0)
	{
		HAL_TIM_Base_Stop_IT(&htim3); // Disable Timer3
		PulseGenerationFlag = false;
		Timer3Count = 0;
		return;
	}

	TargetPosition = DeltaPulse; // Relative Position value

	if(StartSimulating)
	{
		if (DeltaPulse < 0) // Move up
		{
			Direction = false;
		}
		else
		{
			if (DeltaPulse > 0) // Move down
			{
				Direction = true;
			}
		}
	}
	else // Running
	{
		if (DeltaPulse < 0) // Move up
		{
			if(Direction) //false = move up, true = move down
			{
				PRIsToggled = !PRIsToggled;
			}
		}
		else
		{
			if (DeltaPulse > 0) // Move down
			{
				if(!Direction) //false = move up, true = move down
				{
					PRIsToggled = !PRIsToggled;
				}
			}
		}
	}


	Timer3CountPeriod = PulseCountPeriod;
	IsReachTargetPosition = false;
	PulseGenerationFlag = true; //Enable Pulse Generation
	HAL_TIM_Base_Start_IT(&htim3); // Enable Timer3
}

bool GoingToRefPosition(int RefDeltaPulse) // return true when finish going down, else return false;
{
	if (RefDeltaPulse == 0)
		return true;

	if (IsReachTargetPosition)
	{
		Timer3CountPeriod = 0;
		SpeedCmd = 0;
		StopPulseGenerating();
		return true;
	}

	if (MotorDriver) // FDA7000 Driver, PosCmd based
	{
		// Start reducing the speed
		if (abs(AbsoluteTargetMotorPosition - MotorEncPulse + OriginPulse) <= (EncoderResolution*PullingSpeed*RampingGoingSpdTime/60))
		{
			if (RefDeltaPulse > 0) // go down
			{
				LinearGeneration(&SpeedCmd,-GoingAcceleration*9.549,20); //-EpsilonPulling means the spd is negative
			}
			else // go up
			{
				LinearGeneration(&SpeedCmd,GoingAcceleration*9.549,-20); //-EpsilonPulling means the spd is negative
			}
			if (SpeedCmd != 0)
			{
				Timer3CountPeriod = CalculateTimer3Period (MotorDriver, SpeedCmd);
				PulseGenerationFlag = true;
			}
		}
		else // Acclerate going
		{
			// Ramping the speed cmd
			if (RefDeltaPulse > 0) // go down
			{
				LinearGeneration(&SpeedCmd,GoingAcceleration*9.549,PullingSpeed);
			}
			else // go up
			{
				LinearGeneration(&SpeedCmd,-GoingAcceleration*9.549,-PullingSpeed); //-EpsilonPulling means the spd is negative
			}

			if (SpeedCmd != 0)
			{
				// Calculate Timer3CountPeriod to generate pulse
				Timer3CountPeriod = CalculateTimer3Period (MotorDriver, SpeedCmd);
				PulseGenerationFlag = true;
			}
		}
	}
	else // ASDA A3, Actual Encoder based
	{
		if (abs(AbsoluteTargetMotorPosition - MotorEncPulse + OriginPulse) <= (EncoderResolution*PullingSpeed*RampingGoingSpdTime/90)) // Start reducing the speed
		{
			//RunningTime2 += SampleTime;
			if (RefDeltaPulse > 0) // go down
			{
				LinearGeneration(&SpeedCmd,-GoingAcceleration*9.549,20);
			}
			else // go up
			{
				LinearGeneration(&SpeedCmd,GoingAcceleration*9.549,-20);
			}

			if (SpeedCmd != 0)
			{
				Timer3CountPeriod = CalculateTimer3Period (MotorDriver, SpeedCmd);
				PulseGenerationFlag = true;
			}
		}

		else
		{
			// Ramping the speed cmd
			if (RefDeltaPulse > 0) // go down
			{
				//SpeedCmd = LinearGeneration(RunningTime,GoingAcceleration*9.549,0,0,PullingSpeed); //-EpsilonPulling means the spd is negative
				LinearGeneration(&SpeedCmd,GoingAcceleration*9.549,PullingSpeed);
			}
			else // go up
			{
				LinearGeneration(&SpeedCmd,-GoingAcceleration*9.549,-PullingSpeed);
			}

			if (SpeedCmd != 0)
			{
				Timer3CountPeriod = CalculateTimer3Period (MotorDriver, SpeedCmd);
				PulseGenerationFlag = true;
			}
		}
	}
	return false;
}


// Init variable for running
void InitializeRunning (uint8_t Mode)
{
	switch (Mode)
	{
		case 1: // Dropping Mode, go to home position first
			TargetPosition = 0 - MotorEncPulse + OriginPulse; // 0 means home position;
			break;
		case 2: // Pulling Mode, go to bottom position first
			TargetPosition = PullingBotomPulseCmdPosition*EgearRatio - MotorEncPulse + OriginPulse;
			break;
		case 3: // Pull and Drop, go to bottom position first
			TargetPosition = PullingBotomPulseCmdPosition*EgearRatio - MotorEncPulse + OriginPulse;
			break;
		default:
			break;
	}

	if (TargetPosition > 0) // Go down
	{
		InitPulseGenerating(false);
	}
	if (TargetPosition < 0) // Go up
	{
		InitPulseGenerating(true);
	}
	AbsoluteTargetMotorPosition = TargetPosition + MotorEncPulse - OriginPulse;
	TargetPosition = TargetPosition/EgearRatio;
	// TargetPosition is the number of pulses generated from the controller
	// the actual pulses received by the driver is gained by the EgearRatio.

	IsGoingToInitPoint = true; // go to init poit first
	StartRunning = true; //
	StartMainRun = false;
	StartSimulating = false;

	Timer3CountPeriod = 0;
	PositionPulseCmd = 0;
	Index = 0;
}

void StopExperiment ()
{
	StopPulseGenerating(); // Stop pulse generation
	IsReachTargetPosition = false;
	IsPulseCheck = false;

	// Reset all the flag and state
	StartRunning = false;
	StartSimulating = false;
	StartMainRun = false;
	CompleteDropping = false;
	CompletePulling = false;

	TargetPosition = 0;
	Index = 0;
	Timer3CountPeriod = 0;
	PositionPulseCmd = 0;
	SpeedCmd = 0;
}
void CalculateRunningSpec () // Calculate running parameters
{
	GoingAcceleration = 0.1*PullingSpeed/RampingGoingSpdTime; // to rad/s2
}


void InitParams ()
{
	// Load Parameters from the memory
	LoadSavedParam(MemoryAddress,Params);

	// General Params
	DrumRadius = Params[0];
	PullingSpeed = Params[1];
	StoppingTime = Params[2];
	SampleTime = Params[3];

	//DistCoeff = Params[13];

	CalculateRunningSpec ();
}
void InitSimulating()
{
	StartSimulating = true;
	PulseSimuCount = 0;
	TargetPosition = 0;
	Timer3CountPeriod = 0;
	//HAL_TIM_Base_Start_IT(&htim3);
	Index = 0;
}
void StopSimulating()
{
	IsReachTargetPosition = false;

	StartSimulating = false;
	StopPulseGenerating(); // Stop pulse generation

	Index = 0;
	Timer3CountPeriod = 0;
	PulseSimuCount = 0; // Reset PulseCmd
	PositionSimCmd = 0;
}

void Simulating()
{
	if (Index >= DropCurveDataIndex)
	{
		StopSimulating();
		Initialized = true;
		return;
	}
	if(DropRefPosDeltaPulse[Index] > 0) // Move down
	{
		 Direction = true;
	}
	else
	{
		if(DropRefPosDeltaPulse[Index] > 0) // Move up
		{
			 Direction = false;
		}
	}
	PulseGenerating(DropRefPosDeltaPulse[Index], 4);
	// 4 is a fixed value to ensure generating enough pulse during the SampleTime
	// The smaller value, the faster of the pulse generating
	Index++;
}

void RunningExperiment()
{
	if (IsGoingToInitPoint)
	{
		if (GoingToRefPosition(TargetPosition))
		{
			if(WaitingMiliSecond(StoppingTime))
			{
				IsGoingToInitPoint = false;
				StartMainRun = true;
				PositionPulseCmd = 0;
				TargetPosition = 0;
				switch (ExperimentMode)
				{
					case 1: // Dropping
						PRIsToggled = false; // go down
						break;
					case 2: // Pulling
						PRIsToggled = true; // go up
						Direction = false; //up
						break;
					case 3: //Pull and Drop
						PRIsToggled = true; // go up
						Direction = false; //up
						break;
					default:
						break;
				}
			}
			else
			{
				return;
			}

		}
		else
		{
			return;
		}
	}

	if (StartMainRun)
	{
		if (Index > DropCurveDataIndex)
		{
			StopExperiment();
			return;
		}
		PulseGenerating(DropRefPosDeltaPulse[Index],4);
		if(DropRefPosDeltaPulse[Index] > 0) // move down
		{
			Direction = true; //down
		}
		if(DropRefPosDeltaPulse[Index] < 0) // move up
		{
			Direction = false; //up
		}
		Index++;
	}
}

void ProcessReceivedCommand () // Proceed the command from the UI
{
	switch ((int)MotionCode[0])
	{
		case 44: //Emergency Stop Change to 44 to avoid data confusion
			if ((int)MotionCode[1] == 0) // 44/0
			{
				Estop(); // Estop button on the UI
				StopPulseGenerating();
				//EMO = true;
			}
			else {AlarmReset();}  // 44/1, alarm button
			break;
		case 1: // Stop button;
			if ((int)MotionCode[1] == 1) // 1/1
			{
				if (MotorDriver) // FDA 7000
				{
					Stop();
					StopExperiment();
					StopPulseGenerating();
					IsHoming = false;
				}
				else // ASDA A3
				{
					StopPulseGenerating();
					StopExperiment();
					IsHoming = false;
				}
			}
			break;
		case 2: // Receive Position Data
			DropCurveDataIndex = (uint16_t)MotionCode[1];
			DropRefPosDeltaPulse[DropCurveDataIndex] = (int)MotionCode[2];
			break;

		case 3: // Jog Control

			if ((int)MotionCode[1] == 1) // 3/1 move up button
			{
					if (PositionControlMode) // If the control Mode is Position Mode
					{
						// Calculate Timer3CountPeriod to generate pulse
						Timer3CountPeriod = CalculateTimer3Period(MotorDriver,JogSpeed);
						PRIsToggled = true; // PR phase is 90 deg late

						InitPulseGenerating(true); // Reset PF, PR + Enable Timer + PulseGeneratingFlag = true
						PulseGenerationFlag = true;
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
						// Calculate Timer3CountPeriod to generate pulse
						Timer3CountPeriod = CalculateTimer3Period(MotorDriver,JogSpeed);
						PRIsToggled = false; //

						InitPulseGenerating(false); // Reset PF, PR + Enable Timer + PulseGeneratingFlag = true
						PulseGenerationFlag = true;
						DisableSTOP();	// Turn off STOP to run
					}
					else // Speed Mode
					{
						JogMoveDown(); // Disable the stop
					}
			}
			break;
		case 4: // Start Running Buton (Start Running Experiment)
			if ((int)MotionCode[1] == 1) // Start runing
				{
					if ( Initialized )
					{
						TxPCLen = sprintf(ResponseMess,"g4/1e"); // Respond that the experiment started
						HAL_UART_Transmit(&huart6,(uint8_t *)ResponseMess,TxPCLen,200); // Send to uart6 to check the params are set or not
						HAL_Delay(200);
						memset(ResponseMess, '\0', sizeof(ResponseMess));

						InitializeRunning (ExperimentMode);
					}
					else
					{
						TxPCLen = sprintf(ResponseMess,"g4/0e"); // Respond that the experiment can not start
						HAL_UART_Transmit(&huart6,(uint8_t *)ResponseMess,TxPCLen,200); // Send to uart6 to check the params are set or not
						HAL_Delay(200);
						memset(ResponseMess, '\0', sizeof(ResponseMess));
					}
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
				// Calculate Timer3CountPeriod to generate pulse
				Timer3CountPeriod = CalculateTimer3Period(MotorDriver,JogSpeed);

				TxPCLen = sprintf(ResponseMess,"j%de",JogSpeed);
				HAL_UART_Transmit(&huart6,(uint8_t *)ResponseMess,TxPCLen,200);
				memset(ResponseMess, '\0', sizeof(ResponseMess)); // Clear the array
			}
			break;

		case 6: // 6 request driver data
			if((int)MotionCode[1] == 1)
			{
				UIDataRequest = true;
				ReadMultiRegister(StE03,5);
			} // 6/1 If the UI request data
			else
			{
				UIDataRequest = false;
			}
			break;

		case 7: // Save System Params
			// Save to the flash memory
			SaveSystemParams(&numofwords);
			HAL_Delay(500);
			SaveSystemParams(&numofwords); // Do twice times
			// Send back to the UI to notify
			char MessageBuffer[10];
			TxPCLen = sprintf(MessageBuffer,"r7/1e");
			HAL_UART_Transmit(&huart6,(uint8_t *)MessageBuffer,TxPCLen,100); // Send to uart6 to check the params are set or not
			break;

		case 8: // Request reading digital driver output
			if((int)MotionCode[1] == 1) {OutputDataRequest = true;} // 8/1 = request
			else OutputDataRequest = false; // 8/0 = stop request
			break;

		case 9: // Get Total Pulling Pull
			PullingBotomPulseCmdPosition = (uint32_t)(MotionCode[1]);
			break;

		case 10: // Get Running Mode
			ExperimentMode = (uint8_t)(MotionCode[1]);
			break;

		case 45: // Load saved parameters

			// Send to the GUI
			TxPCLen = sprintf(TxPCBuff,"p%.2f/%d/%d/%de"
			                   ,DrumRadius, SampleTime, PullingSpeed,StoppingTime); // Combine to a string
			HAL_UART_Transmit(&huart6,(uint8_t *)TxPCBuff,TxPCLen,200); // Send to uart6 to check the params are set or not
			break;

		case 11: // Set Drum Radius
			if (StartRunning) // Setting is not available while running
			{
				InitializeRunning (ExperimentMode);
				break;
			}
			else
			{
				DrumRadius = MotionCode[1];
				Initialized = false;

				TxPCLen = sprintf(ResponseMess,"r11/%.2fe",DrumRadius);
				HAL_UART_Transmit(&huart6,(uint8_t *)ResponseMess,TxPCLen,200);
				memset(ResponseMess, '\0', sizeof(ResponseMess));
				break;
			}
			break;

		case 13: // Set Going Speed
			if (StartRunning) // Setting is not available while running
			{
				break;
			}
			else
			{
				PullingSpeed = MotionCode[1];
				GoingAcceleration = 0.1*PullingSpeed/RampingGoingSpdTime; // to rad/s2

				//char PullingSpeedBuffer[10];

				TxPCLen = sprintf(ResponseMess,"r13/%de",PullingSpeed);
				HAL_UART_Transmit(&huart6,(uint8_t *)ResponseMess,TxPCLen,200); // Send to uart6 to check the params are set or not
				memset(ResponseMess, '\0', sizeof(ResponseMess));
				break;
			}

		case 14: // Start Simulating
			if (StartRunning) // Setting is not available while running
			{
				break;
			}
			else
			{
				if (MotionCode[1] == 1) // Start Simulation
				{
					if (DropCurveDataIndex == 0) // There is no data
					{
						TxPCLen = sprintf(ResponseMess,"g14/0e"); // Respond that the experiment can not start
						HAL_UART_Transmit(&huart6,(uint8_t *)ResponseMess,TxPCLen,200); // Send to uart6 to check the params are set or not
						HAL_Delay(200);
						memset(ResponseMess, '\0', sizeof(ResponseMess));
					}
					else
					{
						InitSimulating();

						TxPCLen = sprintf(ResponseMess,"g14/1e"); // Respond that the experiment can not start
						HAL_UART_Transmit(&huart6,(uint8_t *)ResponseMess,TxPCLen,200); // Send to uart6 to check the params are set or not
						HAL_Delay(200);
						memset(ResponseMess, '\0', sizeof(ResponseMess));
					}

					break;
				}
				else // Stop Simulation
				{
					StopSimulating();
				}
			}
			break;

		case 16: // Set SampleTime
			if (StartRunning) // When the experiment is running, disable this fcn
			{
				break;
			}
			else
			{
				SampleTime = MotionCode[1];
				if (SampleTime<= 0) // ms Set value range, 1:100ms
				{
					SampleTime = 1;
				}
				if (SampleTime >= 100) // ms
				{
					SampleTime = 100;
				}
				// Reset the Input data for safety
				DropCurveDataIndex = 0;
				memset(DropRefPosDeltaPulse, 0, sizeof(DropRefPosDeltaPulse));
				Initialized = false; // Request to initializing again

				TxPCLen = sprintf(ResponseMess,"r16/%de",SampleTime);
				HAL_UART_Transmit(&huart6,(uint8_t *)ResponseMess,TxPCLen,200); // Send to uart6 to check the params are set or not
				memset(ResponseMess, '\0', sizeof(ResponseMess));
				break;
			}

		case 17: // Reset MCU
			HAL_NVIC_SystemReset();
			break;
		case 18: // Servo Enable on/off
			if (MotionCode[1] == 1) // Servo Enable ON
			{
				HAL_GPIO_WritePin(SerVoReset_PC4_18_GPIO_Port, SerVoReset_PC4_18_Pin, GPIO_PIN_SET); // Servo enable on

				//HAL_GPIO_WritePin(CCWLIM_Not_PE12_39_GPIO_Port, CCWLIM_Not_PE12_39_Pin, GPIO_PIN_RESET); // SPD LIM OFF
				//HAL_GPIO_WritePin(CWLIM_Not_PE14_13_GPIO_Port, CWLIM_Not_PE14_13_Pin, GPIO_PIN_RESET); // SPD LIM OFF
				//HAL_GPIO_WritePin(SPDLIM_Not_PE11_38_GPIO_Port, SPDLIM_Not_PE11_38_Pin, GPIO_PIN_RESET); // SPD LIM OFF
			}

			else
				HAL_GPIO_WritePin(SerVoReset_PC4_18_GPIO_Port, SerVoReset_PC4_18_Pin, GPIO_PIN_RESET); // Servo enable OFF
			break;

		case 28: // Stop jog move up/down in Position Jog control;
			if (StartRunning) // Setting is not available while running
			{
				break;
			}
			else
			{
				StopPulseGenerating();
				break;
			}

		case 31: // Set Experiment Mode Pulling/Dropping/PaD
			if (StartRunning) // Setting is not available while running
			{
				break;
			}
			else
			{
				ExperimentMode = MotionCode[1]; // 1=Dropping Mode;2 = Pulling; 3= Pulling->Dropping
				//char SetModeBuff[8];
				TxPCLen = sprintf(ResponseMess,"m%de",ExperimentMode);
				HAL_UART_Transmit(&huart6,(uint8_t *)ResponseMess,TxPCLen,100); // Send to uart6 to check the params are set or not
				memset(ResponseMess, '\0', sizeof(ResponseMess));
				break;
			}

		case 37: // Set Stopping Time
			if (StartRunning)// Setting is not available while running
			{
				break;
			}
			else
			{
				StoppingTime = MotionCode[1];

				TxPCLen = sprintf(ResponseMess,"r37/%de",StoppingTime);
				HAL_UART_Transmit(&huart6,(uint8_t *)ResponseMess,TxPCLen,200); // Send to uart6 to check the params are set or not
				memset(ResponseMess, '\0', sizeof(ResponseMess));
				break;
			}
		case 38: // Homing task
			if (StartRunning)// Setting is not available while running
			{
				break;
			}
			else
			{
				IsHoming = true;
				Direction = false; // false = move up, true = move down
				PRIsToggled = true; // false = Dropping Down. change to true/false to change the direction: pulling or dropping
				DisableSTOP(); // Disable the stop
				InitPulseGenerating(true);
			}
			break;

		case 39: // Set Driver type, FDA7000 or ASDA A3
			if (StartRunning)// Setting is not available while running
			{
				break;
			}
			else
			{
				if (MotionCode[1] == 1) // FDA7000
				{
					MotorDriver = true;
					NoOfBytes = 29; // For FDA7000, read 5 register => receive 25 bytes
					EncoderResolution = HigenEncoderResolution;
					EgearRatio = 8; // Depend on the driver setting, change this value when driver setting change
				}
				else // ASDA A3
				{
					EgearRatio = 1;
					MotorDriver = false;
					NoOfBytes = 17;
					EncoderResolution = AsdaEncoderResolution;
					// For ASDA Drier, read 1 register => receive 9 bytes
					// read 2 registers => receive 13 bytes
				}
				TxPCLen = sprintf(ResponseMess,"g39/%de",MotorDriver);
				HAL_UART_Transmit(&huart6,(uint8_t *)ResponseMess,TxPCLen,200); // Send to uart6 to check the params are set or not
				memset(ResponseMess, '\0', sizeof(ResponseMess));
			}
			break;

		case 42: // Set Distance Coefficient
			if (StartRunning)// Setting is not available while running
			{
				InitializeRunning(ExperimentMode);
			}
			else
			{
				DistCoeff = MotionCode[1];
				Initialized = false;

				TxPCLen = sprintf(ResponseMess,"r42/%.2fe",DistCoeff);
				HAL_UART_Transmit(&huart6,(uint8_t *)ResponseMess,TxPCLen,100); // Send to uart6 to check the params are set or not
				memset(ResponseMess, '\0', sizeof(ResponseMess));
			}
			break;

		case 46: // Set origin (home) position
			OriginPulse = MotorEncPulse;
			PositionPulseCmd = 0;
			PulseSimuCount = 0;
			//PulseClear();
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
  // UNUSED(huart);

	// BEGIN UART6 Receiving
		if (huart->Instance==USART6) // If it is uart6, UI communication
		{
			if(RxPCData!=EndChar) // read up to the ending char
			{
				if (RxPCData != 0) // remove the null character
				//if (RxPCData != NULL) // remove the null character
				{
					RxPCBuff[_rxPCIndex]=RxPCData;// Copy the data to buffer
				  _rxPCIndex++;
					HAL_UART_Receive_IT(&huart6,&RxPCData,1);
				}
			}
			else //if(RxPCData==EndChar)
			{
				_rxPCIndex=0;
				RxUart6_Cpl_Flag=true; // reading completed
			}
		}
	// END UART6

		//BEGIN UART5 = HAL_UART_Receive_IT============================================
		/// Use this part
		if (huart->Instance==UART5) // If it is uart5, driver communication
		{
			if (_rxDriverIndex >= NoOfBytes) //
			{
				RxUart5_Cpl_Flag = true; // Complete Receiving
				StartReceiveDriverData = false;
				_rxDriverIndex = 0;
			}
			if ((_rxDriverIndex == 0)&&(RxDriverData == DriverID)) // If byte 0 is the Driver ID
			{
				StartReceiveDriverData = true;
			}
			if (StartReceiveDriverData) //
			{
				RxDriverBuff[_rxDriverIndex]=RxDriverData;// Copy the data to buffer
				_rxDriverIndex++;
				HAL_UART_Receive_IT(&huart5,&RxDriverData,1); // Receive 1 byte each time ///*/
			}
		}
		// END UART5
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) // Timer 2 interrupt, 1ms
{
	if (htim->Instance == TIM3)	// TIMER 3 interrupt for pulse generation, period: 2us
	{
		if (PulseGenerationFlag) // Only generating pulse when the flag is ON. Otherwise, do nothing
		{
				Timer3Count++;
				if (Timer3Count >= Timer3CountPeriod) // Generate pulse
				{
					Timer3Count = 0;

					if(StartSimulating) // Check the no of pulse generated in Simulating
					{
						if (IsReachTargetPosition)
						{
							return;
						}
						if ( abs(PulseSimuCount) >= abs(TargetPosition))
						{
							IsReachTargetPosition = true;
							PositionSimCmd += PulseSimuCount;
							PulseSimuCount = 0;
							return;
						}
						if (Direction) // go down
						{
							PulseSimuCount++;
						}
						else // move up
						{
							PulseSimuCount--;
						}
						return;
					}

					if (StartRunning)
					{
						if (IsReachTargetPosition)
						{
							return;
						}
						if ( abs(PositionPulseCmd) >= abs(TargetPosition))
						{
							PositionPulseCmd = 0;
							IsReachTargetPosition = true;
							return;
						}
					}

					if (PRIsToggled)
					{
						HAL_GPIO_TogglePin(PE9_TIM1_CH1_PFIN_GPIO_Port, PE9_TIM1_CH1_PFIN_Pin); // Generate pulses on PF by tonggling this input
						PRIsToggled = false;
						PositionPulseCmd++;
						return; // exit the function
					}
					else
					{
						HAL_GPIO_TogglePin(PC8_PR_GPIO_Port, PC8_PR_Pin); // Generate pulses on PF by tonggling this input
						PRIsToggled = true;
						PositionPulseCmd++;
						return;
					}
				}
		}
	}

	if (htim->Instance == TIM2) // Timer 2 interrupt, for the main control function, 1ms
		{
				Timer2SampleTimeControlCount++;
				if (Timer2SampleTimeControlCount >= SampleTime) // turn on the flag when the sample time reaches, fix the data sample time to 50ms
				{
					if (StartSimulating)
					{
						Simulating();
					}
					if (StartRunning)
					{
						RunningExperiment();
					}
					Timer2SampleTimeControlCount = 0;
				}

				// To transmit the data each 50ms
				Timer2Count++;
				if (Timer2Count >= 25) // turn on the flag when the sample time reaches, fix the data sample time to 50
				{
					Timer2SampleTimeInterrupt = true;
					Timer2Count = 0;
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
  MX_USART6_UART_Init();
  MX_TIM3_Init();
  MX_UART5_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	  /* USER CODE BEGIN 2 */

  	//HAL_GPIO_WritePin(SPDLIM_Not_PE11_38_GPIO_Port, SPDLIM_Not_PE11_38_Pin, GPIO_PIN_SET); // SPD LIM OFF
  	//HAL_GPIO_WritePin(Speed2_Not_PE7_15_GPIO_Port, Speed2_Not_PE7_15_Pin, GPIO_PIN_SET); // SPD LIM OFF
  	//DisableSTOP();

	HAL_GPIO_WritePin(PE15_RELAY1_GPIO_Port, PE15_RELAY1_Pin, GPIO_PIN_SET);
	HAL_Delay(5000);

	InitParams (); // Read the saved params from the flash memory

	HAL_TIM_Base_Start_IT(&htim2); // Enable Timer 2 interrupt

	HAL_UART_Receive_IT(&huart6,&RxPCData,1);

	DriverInit();
	ReadMultiRegister(StE03,5);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		// Process Received Cmd from the GUI
		if(RxUart6_Cpl_Flag) // If data is completely received, a command is sent from the UI
			{
				ExtractMotionCode();
				ProcessReceivedCommand (); // Proceed the command
				RxUart6_Cpl_Flag=false;
				HAL_UART_Receive_IT(&huart6,&RxPCData,1);
			}
		// END UART6 Process Cmd

		// Process Timer2 interrupt after a period of Sampletime
		if (Timer2ControlInterrupt)
		{
			Timer2ControlInterrupt = false; // Reset the flag
			// BEGIN running experiment
			if (StartRunning) // Process Running Experiment
			{ }
			// END Running Experiment

			// START SIMULATING EXPERIMENT
			if (StartSimulating) // Process Running Experiment
			{ }
			// END SIMULATION

		}
		// END Timer2ControlInterrupt

		// Process Emergency Stop
		if (StartRunning)
		{
			// Read CN1-22-RDY, Check if Servo Ready or not, or Servo ON of OFF.
			// If servo is not ready or not ON, then turn off all the functions
			//if ( HAL_GPIO_ReadPin(CN1_22_RDY_GPIO_Port,CN1_22_RDY_Pin) || HAL_GPIO_ReadPin(CN1_48_BRAKE_GPIO_Port,CN1_48_BRAKE_Pin))
			if ( HAL_GPIO_ReadPin(CN1_22_RDY_GPIO_Port,CN1_22_RDY_Pin))
			{
			// If the servo is not ON. Then stop all the running function
				StopPulseGenerating();
				StopExperiment();
				IsHoming = false; // Disable Homming
			}
		}
		// END Process Emergency Stop

		// BEGIN Timer2 interrupt for sending the data
		if (Timer2SampleTimeInterrupt)
		{
			Timer2SampleTimeInterrupt = false;

			// BEGIN Software Limit ASDA Driver
			if (!MotorDriver) // Applied for ASDA-A3 Diver since the encoder pulse only can be cleared when cycle the driver
			{
				if (StartRunning) // Always turn on the software limit while running
				{
					if (MotorEncPulse-OriginPulse <= -200) // Software Limit Switch based on actual motor position, 500/2048 pulses
					{
						StopPulseGenerating();
						StopExperiment();
					}
				}
				if (SoftWareLimit) // Software limit is on
				{
					if (MotorEncPulse-OriginPulse <= -100) // Software Limit Switch based on actual motor position, 500/2048 pulses
					{
						StopPulseGenerating();
						StopExperiment();
					}
				}
			}
			else // HIGEN FDA Driver, Software Limit Switch
			{
				if (StartRunning)// Always turn on the software limit while running
				{
					if (MotorEncPulse-OriginPulse <= 0) // Software Limit Switch based on actual motor position, 500/2048 pulses
					{
						StopPulseGenerating();
						StopExperiment();
					}
				}
				if (SoftWareLimit) // Software limit is on
				{
					if (MotorEncPulse-OriginPulse <= -2000) // Software Limit Switch based on actual motor position, 500/2048 pulses
					{
						StopPulseGenerating();
						StopExperiment();
					}
				}
			}
			// END Software Limit ASDA Driver
			//BEGIN Homing

//			if (IsHoming)
//			{
//				if(CheckGoingToRefPosition()) // false = go up, 0 = home position, 100 pulses
//				{
//					IsHoming = false; // finish homing
//				}
//
//				if (MotorEncPulse-OriginPulse <= 0) // Software Limit
//				{
//					StopPulseGenerating();
//					IsHoming = false;
//				}
//			}
			//END Homing

			// BEGIN Send data to the UI
			if (UIDataRequest)
			{
				if (MotorDriver) // FDA7000 Driver
				{
					if (StartSimulating)
					{
						PositionCmd = (float)(8*PositionSimCmd*2*3.14*DrumRadius/EncoderResolution);
						TxPCLen = sprintf(TxPCBuff,"t%.1f/%.1f/%.1fe",MotorSpeed,ObjectPosition,PositionCmd);
						HAL_UART_Transmit(&huart6,(uint8_t *)TxPCBuff,TxPCLen,200); // use uart6 to send
					}
					else
					{
						TxPCLen = sprintf(TxPCBuff,"s%.1f/%.1f/%.1fe",MotorSpeed,ObjectPosition,PositionCmd);
						//TxPCLen = sprintf(TxPCBuff,"s%.1f/%.1f/%.1f/%.1f/%.1fe",MotorSpeed,SpeedCmd,ObjectPosition,AccRef,PositionCmd);
						HAL_UART_Transmit(&huart6,(uint8_t *)TxPCBuff,TxPCLen,200); // use uart6 to send
						ReadMultiRegister(StE03,6); // Read from StE03 -> StE08
					}
				}
				else // ASDA-A3 Driver
				{
					if (StartSimulating)
					{
						PositionCmd = (float)(PositionSimCmd*2*3.14*DrumRadius/EncoderResolution);
						TxPCLen = sprintf(TxPCBuff,"t%.1f/%.1f/%.1fe",MotorSpeed,ObjectPosition,PositionCmd);
						//TxPCLen = sprintf(TxPCBuff,"s%.1f/%.1f/%.1f/%.1f/%.1fe",MotorSpeed,SpeedCmd,ObjectPosition,AccRef, PositionCmd);
						HAL_UART_Transmit(&huart6,(uint8_t *)TxPCBuff,TxPCLen,200); // use uart6 to send
						// Read 4 words start from 0x012 to 0x015
						// Encoder pulse: 0x012 + 0x013 (2 words)
						// Motor Speed: 0x014 + 0x015 (2 words)
						// Pulse Error: (2 word)
						// Total is 6 words, 1word = 4 bytes
						ReadMultiRegister(ASDA_MotorSpeed,6);
					}
					else
					{
						TxPCLen = sprintf(TxPCBuff,"s%.1f/%.1f/%.1fe",MotorSpeed,ObjectPosition,PositionCmd);
						//TxPCLen = sprintf(TxPCBuff,"s%.1f/%.1f/%.1f/%.1f/%.1fe",MotorSpeed,SpeedCmd,ObjectPosition,AccRef, PositionCmd);
						HAL_UART_Transmit(&huart6,(uint8_t *)TxPCBuff,TxPCLen,200); // use uart6 to send
						// Read 4 words start from 0x012 to 0x015
						// Encoder pulse: 0x012 + 0x013 (2 words)
						// Motor Speed: 0x014 + 0x015 (2 words)
						// Pulse Error: (2 word)
						// Total is 6 words, 1word = 4 bytes
						ReadMultiRegister(ASDA_MotorSpeed,6);
					}
				}
				memset (TxPCBuff, '\0', sizeof (TxPCBuff)); // reset
			}
			if(OutputDataRequest)
				{
					CountTimerDriverOutput++;
					if (CountTimerDriverOutput >= 15) // 20*25=500 ms, timer 2 period is 1ms
					{
						DriverOutput = ReadLogicF7000Out(); // Read Driver Output

						memset (TxPCBuff, '\0', sizeof (TxPCBuff)); // reset
						TxPCLen = sprintf(TxPCBuff,"o%de",DriverOutput); // 1 means only the driver outputs
						HAL_UART_Transmit(&huart6,(uint8_t *)TxPCBuff,TxPCLen,200); // use uart6 to send

						CountTimerDriverOutput = 0;
					}
				}
			// END Send data to the UI
		}
		// END 20ms period interrupt

		// BEGIN Process Driver Received Data
		if (RxUart5_Cpl_Flag) // Complete receive data from the driver
			{
				RxUart5_Cpl_Flag = false;

				for (i = 0; i<= sizeof(RxDriverBuff); i++)
				{
					if (RxDriverBuff[i] == DriverID)
					{
						if (RxDriverBuff[1+i] == 3)
						{
							if (MotorDriver) // FDA7000
							{
								SpeedValueRegion[0] = RxDriverBuff[6+i];
								SpeedValueRegion[1] = RxDriverBuff[5+i];
								SpeedValueRegion[2] = RxDriverBuff[4+i];
								SpeedValueRegion[3] = RxDriverBuff[3+i];

								memcpy(&MotorSpeed, SpeedValueRegion, 4);

								MotorEncPulse = (RxDriverBuff[19+i] << 24) | (RxDriverBuff[20+i] << 16) | (RxDriverBuff[21+i] << 8) | RxDriverBuff[22+i];
								//MotorEncPulse = (RxDriverBuff[18+i] << 24) | (RxDriverBuff[19+i] << 16) | (RxDriverBuff[20+i] << 8) | RxDriverBuff[21+i];
							}
							else // ASDA-A3
							{
								MotorSpeedBuff = (float)((RxDriverBuff[5+i] << 24) | (RxDriverBuff[6+i] << 16) | (RxDriverBuff[3+i] << 8) | RxDriverBuff[4+i])/((float)-10.0); // Minus to Reverse

								if (fabs(MotorSpeedBuff) <= 2000)
								{
									MotorSpeed = MotorSpeedBuff;
								}

								MotorEncPulse = -((RxDriverBuff[9+i] << 24) | (RxDriverBuff[10+i] << 16) | (RxDriverBuff[7+i] << 8) | RxDriverBuff[8+i]); // Minus to Reverse
							}

							memset (RxDriverBuff, '\0', sizeof (RxDriverBuff)); // reset buffer
							HAL_UART_Receive_IT(&huart5,&RxDriverData,1); // Receive 1 byte for the next time
						}
						if (RxDriverBuff[1] == 6) // Writing to a register
						{
							// Send to PC to check the writing result
							memset (TxPCBuff, '\0', sizeof (TxPCBuff)); // reset
							TxPCLen = sprintf(TxPCBuff,"w%d/%d/%d/%d/%d/%d/%d/%d/%d/e",RxDriverBuff[0],RxDriverBuff[1],RxDriverBuff[2],RxDriverBuff[3],RxDriverBuff[4],RxDriverBuff[5],RxDriverBuff[6],RxDriverBuff[7],RxDriverBuff[8]);
							HAL_UART_Transmit(&huart6,(uint8_t *)TxPCBuff,TxPCLen,200); // use uart6 to send
						}
						//break;
					}
				}
				ObjectPosition = 2*3.14*DrumRadius*(MotorEncPulse-OriginPulse)/EncoderResolution; // Calculate Object Position in m
			}
			// END Process Driver Received Data
  } // END While(1)
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
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 3, 0);
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
  htim2.Init.Prescaler = 10-1;
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
  huart5.Init.BaudRate = 57600;
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
  huart6.Init.BaudRate = 115200;
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
  HAL_GPIO_WritePin(GPIOB, EStop_Not_PB0_17_Pin|CN1_16_PulseCCLR_Pin|PB13_Output_JP7_Pin|PB14_POS_CMD_OPC_EN_Pin
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

  /*Configure GPIO pins : EStop_Not_PB0_17_Pin ArlarmRST_PB1_42_Pin CN1_16_PulseCCLR_Pin PB13_Output_JP7_Pin
                           PB14_POS_CMD_OPC_EN_Pin PB15_485_MCU_PC_DIR_Pin PB5_SPI3_MOSI_SPARE_Pin PB6_RELAY2_Pin */
  GPIO_InitStruct.Pin = EStop_Not_PB0_17_Pin|ArlarmRST_PB1_42_Pin|CN1_16_PulseCCLR_Pin|PB13_Output_JP7_Pin
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

  /*Configure GPIO pins : PD8 PD9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
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
