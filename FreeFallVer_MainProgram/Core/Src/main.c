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
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
		volatile uint8_t RxPCBuff[40]; // buffer to read the command from the PC
		volatile uint8_t RxDriverBuff[30]; // buffer to read the data from the driver
		volatile uint8_t RxUart3Buff[10];

		uint8_t AccZDataRegion[10];
//		volatile uint8_t RxESPBuff[10]; // buffer to read the data from the driver
//		uint8_t RxESPData;

		char DataRegion[40]; // array to filt the data from the PC
		uint8_t RxPCData; // variable to save 1 byte received from the PC
		uint8_t RxDriverData; // variable to save 1 byte received from the driver
		uint8_t RxUart3Data;
		char TxPCBuff[30]; // buffer to send message to the PC
		uint8_t TxPCLen;
		uint8_t i;
		char SpeedValueRegion[4];

		uint8_t _rxPCIndex; // index for receiving data from the driver, uart6
		uint8_t _rxDriverIndex; // uart5
		uint8_t _rxUart3Index; // uart3
//		uint8_t _rxESPIndex;
		uint8_t NoOfBytes = 29; // FDA7000, read 6 registers

		// Flag variables
		volatile  bool RxUart6_Cpl_Flag= false; // uart6 receiving complete flag (from the PC)
		volatile  bool RxUart5_Cpl_Flag= false; // uart5 receiving complete flag (from the Driver)

		volatile bool RxUart3_Cpl_Flag = false;
		volatile bool StartReceiveDriverData = false; // to check if start saving the driver data to the buffer RxDriverBuff or not
		volatile bool StartUart3ReceiveData = false;

		//volatile bool IsReadTachometer = false;

		bool StartDropping = false; // bit to check start or not
		bool StartRunning;
		bool Direction; // false = move up, true = move down
		volatile bool Timer2SampleTimeInterrupt;
		volatile bool Timer2ControlInterrupt;
		volatile bool StartPulseCmdCounting = false;
		bool UIDataRequest = false; // to check if the GUI request data or not
		bool OutputDataRequest = true; // default is true
		bool PositionControlMode = true; // Position Mode is default

		bool StartPulling = false;
		bool StartBraking  = false;
		bool PulseGenerationFlag = false;
		bool IsStepPulseCmd = false; //
		bool POSReach =  false; // position reach flag
		bool StartWaiting = false; // waiting flag in pull and drop experiment
		volatile bool IsReachTargetPosition = false;

		bool StartAccleratePulling; // To check
		bool CompleteRunning ; // to check if Droppign/Pulling completed or not
		bool MotorDriver = true; // true = FDA7000 (15kw); false = ASDA-A3 (200W)
		//bool OverTopLimit;
		bool JoggingMoveUp;
		bool EMO = false;
		bool IsHoming = false;

		bool IsOpenLoopControl = true; // false = closed loop, true = open loop control, default is Closed loop control

		uint8_t ExperimentMode = 1; // 1: Dropping Mode, 2: Pulling Mode, 3: Pulling->Dropping Mode
		bool RunningMode = false; // false = Manual; true = automatic
		bool SoftWareLimit = true; // the software limit is turned on in default


		bool PRIsToggled; // to handle the pulse generation

		//uint16_t RunningTime = 0;
		//uint16_t RunningTime2 = 0;

		uint8_t Timer2Count;
		uint8_t Timer2SampleTimeControlCount;
		uint16_t WaitingTime; // waiting time in pull and Drop experiment

		uint16_t CountTimerDriverOutput = 0;
		uint16_t DriverOutput; // to save the driver output
		uint16_t JogSpeed = 30; // Control the Jog Speed 30 rpm is default
		//volatile uint16_t TachometerCount;

		uint16_t Timer3CountPeriod; //
		uint16_t Timer3Count;
		uint16_t StoppingTimeCount;

		//General Parameters
		//uint8_t EgearRatio = 8; // Egear ratio of the driver

		float DrumRadius;
		uint8_t SampleTime; // sample time in ms
		uint8_t PullingSpeed; // rpm, pulling speed, and going down Speed
		//int PrePullingSpeed;
		uint16_t StoppingTime; // ms

		uint16_t EncoderResolution = HigenEncoderResolution;

		float GoingAcceleration;

		// Dropping Experiment Stage
		float DroppingAccel; // m/s2
		float DroppingDecel; // m/s2

		float DroppingEpsilonAcc; // rad/s2
		float DroppingEpsilonDec; // rad/s2

		float DroppingAccelTime; //s
		float DroppingDecelTime; //s
		float DroppingDecelDistance;
		int	DroppingDecelPulseCmd;

		uint16_t DroppingMaxSpeed; // rpm


		float DroppingAccelDistance; // m
		uint8_t DroppingTotalDistance; // m

		// Pulling Experiment Stage
		float PullingAccel; // m/s2
		float PullingDecel; // m/s2

		float PullingEpsilonAcc; // rad/s2
		float PullingEpsilonDec; // rad/s2

		float PullingAccelTime; // s
		float PullingDecelTime; // s

		uint16_t PullingMaxSpeed; // rpm

		float  PullingDecelDistance; // m
		float PullingAccelDistance; // m

		float PullingTotalDistance; // m

		int PullingBotomPulseCmdPosition ; // pulses
		int TopPulseCmd;
		int FlyingPosPulseCmd;
		int BottomFreeDropPulseCmd;
		int OriginPulse; // To save the postion of the origin
		int StepPulseCmd;
		int TargetPosition;
		volatile int PositionPulseCmd;

//	PID_TypeDef TPID; // PID controller

		uint8_t numofwords = 16; // The number of words To save to flash memory
		float Params[13] = {0, 0, 0, 0, 0, 0,0,0,0,0}; // DrumRadius, DroppingAccelDistance, PullingSpeed, StoppingTime, DroppingAccel, SampleTime


//		volatile float AccZ ; // Feedback acceleration

		float MotorSpeed; // variable to save motor speed, it's value is changed in uart5 interrupt => volatile type
		float MotorSpeedBuff;
		volatile int MotorEncPulse;



		float SpeedCmd;
		//float TransitionSpeed;
		//int PulseError;
		const uint8_t EndChar = '$'; // Character to determine the ending of a data frame from the PC (GUI)
		float MotionCode[8]; // array to save the command from the PC
//		uint16_t RegisterAddress; // Register of the address want to read/write

		// PID controller gain

		float Kp;
		float Ki;
		float Kd;

		float DistCoeff; // To estimate the bottom position in pulling task
		float BrakeAccSlope;
		float FlyAccSlope;
		float IntergraError;
		float AccZ;
		float AccRef = -9.6; // Reference accleraion, initial value is -9.6
		float PreAccRef;
		float PreError; //
		float ObjectPosition;
		float AccSet;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_UART5_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

float PIDCalculate (float _AccRef, float _AccFeedback, int MinSatuaration, int MaxSaturation, bool _direction)
{
	float Error = _AccRef - _AccFeedback;

	float Pcalculation = Kp*Error;
	IntergraError += Ki*(Error+PreError)*SampleTime*0.005; // Sampletime in second
	float Dcalculation = Kd*(Error - PreError)/SampleTime; // sampletime in ms

	float ReturnValue = Pcalculation + IntergraError + Dcalculation;

	if (ReturnValue <= MinSatuaration)
		ReturnValue = MinSatuaration;
	if (ReturnValue >= MaxSaturation)
		ReturnValue = MaxSaturation;
	if (_direction)
	{
		return ReturnValue;
	}
	else
	{
		return -ReturnValue;
	}
}

void ResetPIDController ()
{
	IntergraError = 0;
	PreError = 0;
	AccRef = -9.6;
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
				MotionCode[j] = (atof(token)); // covert to float type
			  //MotionCode[j] = (atoi(token)); // covert to int type
        token = strtok(NULL, "/");
				j++;
    }
	memset (RxPCBuff, '\0', sizeof (RxPCBuff)); // reset
}
//void WriteFloatData (uint16_t RegisterAddress, float value, bool GetReceived) // Write a float value to the driver
//{
//	// Prepare data frame -- BEGIN
//	uint8_t TxDataToDriver[10]; // 10 bytes of data
//
//	// Data preparation
//	TxDataToDriver[0] = DriverID;//SerialID of the driver, 1
//	TxDataToDriver[1] = 6;//Write Regis, function code
//	TxDataToDriver[2] = RegisterAddress / 256; // Register Address High byte
//  TxDataToDriver[3] = RegisterAddress % 256; // Register Address LOW byte
//
//	char FloatValue[sizeof(float)];
//	memcpy(FloatValue, &value, sizeof(float)); // convert float to 4 bytes of char
//
//	TxDataToDriver[4] = FloatValue[3];
//	TxDataToDriver[5] = FloatValue[2];
//	TxDataToDriver[6] = FloatValue[1];
//	TxDataToDriver[7] = FloatValue[0];
//
//	//CRC BEGIN=======
//	uint16_t crc = 0xFFFF;
//	for (int pos = 0; pos < 8; pos++) //for (int pos = 0; pos < raw_msg_data.length()/2; pos++)
//	{	crc ^= (uint16_t)TxDataToDriver[pos];          // XOR byte into least sig. byte of crc
//		for (int i = 8; i != 0; i--)
//		{    // Loop over each bit
//			if ((crc & 0x0001) != 0)
//			{      // If the LSB is set
//				crc >>= 1;                    // Shift right and XOR 0xA001
//				crc ^= 0xA001;
//			}
//			else                            // Else LSB is not set
//				crc >>= 1;                    // Just shift right
//		}
//	}
//	TxDataToDriver[8]= (uint8_t)(crc&0x00FF);;//(uint8_t)(TemDat16&0xFF);
//	TxDataToDriver[9]=(uint8_t)((crc>>8)&0x00FF);
//	//CRC=====END/
//	// Prepare data frame -- END
//
//	// Send data use UART5
//	HAL_GPIO_WritePin(PE0_485_MCU_DRV_DIR_GPIO_Port, PE0_485_MCU_DRV_DIR_Pin, GPIO_PIN_RESET); //Switch to transmit mode
//	HAL_UART_Transmit(&huart5,TxDataToDriver,sizeof(TxDataToDriver),200); // use UART5 to send
//
//	if (GetReceived) // Get Received feedback if GetReceived = true
//	{
//		HAL_GPIO_WritePin(PE0_485_MCU_DRV_DIR_GPIO_Port, PE0_485_MCU_DRV_DIR_Pin, GPIO_PIN_SET);	//Switch back to receive mode
//	}
//}

//void WriteIntData (uint16_t RegisterAddress, int value, bool GetReceived) // Write an int value to the driver
//{
//	// Prepare data frame -- BEGIN
//	uint8_t TxDataToDriver[10]; // 10 bytes of data
//
//	// Data preparation
//	TxDataToDriver[0] = DriverID;//SerialID of the driver, 1
//	TxDataToDriver[1] = 6;//Write Regis, function code
//	TxDataToDriver[2] = RegisterAddress / 256; // Register Address High byte
//  TxDataToDriver[3] = RegisterAddress % 256; // Register Address LOW byte
//
//	TxDataToDriver[4] = (char)((value >> 24) & 0xFF); // convert to 4 bytes data
//	TxDataToDriver[5] = (char)((value >> 16) & 0xFF);
//	TxDataToDriver[6] = (char)((value >> 8) & 0xFF);
//	TxDataToDriver[7] =  (char)(value & 0xFF);
//
//	//CRC BEGIN=======
//	uint16_t crc = 0xFFFF;
//	for (int pos = 0; pos < 8; pos++) //for (int pos = 0; pos < raw_msg_data.length()/2; pos++)
//	{	crc ^= (uint16_t)TxDataToDriver[pos];          // XOR byte into least sig. byte of crc
//		for (int i = 8; i != 0; i--)
//		{    // Loop over each bit
//			if ((crc & 0x0001) != 0)
//			{      // If the LSB is set
//				crc >>= 1;                    // Shift right and XOR 0xA001
//				crc ^= 0xA001;
//			}
//			else                            // Else LSB is not set
//				crc >>= 1;                    // Just shift right
//		}
//	}
//	TxDataToDriver[8]= (uint8_t)(crc&0x00FF);;//(uint8_t)(TemDat16&0xFF);
//	TxDataToDriver[9]=(uint8_t)((crc>>8)&0x00FF);
//	//CRC=====END/
//	// Prepare data frame -- END
//	// Send data use UART5
//	HAL_GPIO_WritePin(PE0_485_MCU_DRV_DIR_GPIO_Port, PE0_485_MCU_DRV_DIR_Pin, GPIO_PIN_RESET); //Switch to transmit mode
//	HAL_UART_Transmit(&huart5,TxDataToDriver,sizeof(TxDataToDriver),200); // use UART5 to send
//	if (GetReceived) // Get Received feedback if GetReceived = true
//	{
//		HAL_GPIO_WritePin(PE0_485_MCU_DRV_DIR_GPIO_Port, PE0_485_MCU_DRV_DIR_Pin, GPIO_PIN_SET);	//Switch back to receive mode
//	}
//}


void ReadMultiRegister(uint16_t StartingAddress, uint8_t NoOfRegister) // Read data from the Driver
{
	// Prepare data frame -- BEGIN
	uint8_t TxDataToDriver[8]; // 8 bytes of data frame

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
//	HAL_Delay(1);
	HAL_UART_Transmit(&huart5,TxDataToDriver,sizeof(TxDataToDriver),200); // use UART5 to send
	HAL_GPIO_WritePin(PE0_485_MCU_DRV_DIR_GPIO_Port, PE0_485_MCU_DRV_DIR_Pin, GPIO_PIN_SET);	//Switch back to receive mode
	HAL_UART_Receive_IT(&huart5,&RxDriverData,1); // Receive 1 byte each time

	/// For debug only, should be comment out latter
	//uint8_t u8_TxPCBuff[20]="< Read Data";
	//HAL_UART_Transmit(&huart6,u8_TxPCBuff,sizeof(u8_TxPCBuff),1000);
	/// Dubug END
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

void SaveSystemParams ()
{
	char SaveBuffer[80];
	TxPCLen = sprintf(SaveBuffer,"%.2f/%d/%d/%d/%.1f/%.2f/%.2f/%.1f/%.2f/%.2f/%.3f/%.4f/%.4f"
	                   ,DrumRadius, PullingSpeed, StoppingTime, SampleTime,
										 DroppingAccelDistance, DroppingAccel, DroppingDecel,
                     PullingAccelDistance, PullingAccel, PullingDecel,
					 FlyAccSlope, DistCoeff, BrakeAccSlope); // Combine to a string
	numofwords = (strlen(SaveBuffer)/4)+((strlen(SaveBuffer)%4)!=0);
	Flash_Write_Data(MemoryAddress , (uint32_t *)SaveBuffer, numofwords);
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
void StartPulseGenerating()
{
	HAL_GPIO_WritePin(PC8_PR_GPIO_Port, PC8_PR_Pin, GPIO_PIN_SET); // Set CW direction
	HAL_GPIO_WritePin(PE9_TIM1_CH1_PFIN_GPIO_Port, PE9_TIM1_CH1_PFIN_Pin,GPIO_PIN_SET);
	PulseGenerationFlag = true;
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

bool CheckGoingToRefPosition(bool _direction, int RefPulsePosition) // return true when finish going down, else return false;
// direction = true => go down, false => go up
{
	if (MotorDriver) // FDA7000 Driver, PosCmd based
	{
		//if (abs(RefPulsePosition - EgearRatio*PositionPulseCmd) <= (EncoderResolution*PullingSpeed*RampingGoingSpdTime/120)) // Start reducing the speed
		if (abs(RefPulsePosition - MotorEncPulse + OriginPulse) <= (EncoderResolution*PullingSpeed*RampingGoingSpdTime/60))
		{
			//RunningTime2 += SampleTime;
			if (_direction) // go down
			{
				LinearGeneration(&SpeedCmd,-GoingAcceleration*10,20); //-EpsilonPulling means the spd is negative
			}
			else // go up
			{
				//SpeedCmd = LinearGeneration(RunningTime2,GoingAcceleration*10,PrePullingSpeed,-PullingSpeed,-20); //-EpsilonPulling means the spd is negative
				LinearGeneration(&SpeedCmd,GoingAcceleration*10,-20); //-EpsilonPulling means the spd is negative
			}
			Timer3CountPeriod = CalculateTimer3Period (MotorDriver, SpeedCmd);
		}
		else // Acclerate going
		{
			// Ramping the speed cmd
			//RunningTime += SampleTime;
			if (_direction) // go down
			{
				//SpeedCmd = LinearGeneration(RunningTime,GoingAcceleration*10,0,0,PullingSpeed); //-EpsilonPulling means the spd is negative
				LinearGeneration(&SpeedCmd,GoingAcceleration*10,PullingSpeed);
			}
			else // go up
			{
				//SpeedCmd = LinearGeneration(RunningTime,-GoingAcceleration,0,-PullingSpeed,0); //-EpsilonPulling means the spd is negative
				LinearGeneration(&SpeedCmd,-GoingAcceleration*10,-PullingSpeed); //-EpsilonPulling means the spd is negative
			}

			if (SpeedCmd != 0)
			{
				// Calculate Timer3CountPeriod to generate pulse
				Timer3CountPeriod = CalculateTimer3Period (MotorDriver, SpeedCmd);
				//Timer3CountPeriod = (int)((float)(120000000.0/(fabs(SpeedCmd)*(float)EncoderResolution)) + 0.5);
			}
			//PrePullingSpeed = SpeedCmd;
		}
		if ( abs(RefPulsePosition - MotorEncPulse + OriginPulse) <= 1000) // Reach the ref position
		{
//			RunningTime = 0;
//			RunningTime2 = 0;
			Timer3CountPeriod = 0;
			SpeedCmd = 0;
			//PrePullingSpeed = 0;
			StopPulseGenerating();
			return true;
		}
	}
	else // ASDA A3, Actual Encoder based
	{
		if (abs(RefPulsePosition - MotorEncPulse + OriginPulse) <= (EncoderResolution*PullingSpeed*RampingGoingSpdTime/90)) // Start reducing the speed
		{
			//RunningTime2 += SampleTime;
			if (_direction) // go down
			{
				LinearGeneration(&SpeedCmd,-GoingAcceleration*10,20);
				//SpeedCmd = LinearGeneration(RunningTime2,-GoingAcceleration*10,PrePullingSpeed,20,PullingSpeed); //-EpsilonPulling means the spd is negative
			}
			else // go up
			{
				LinearGeneration(&SpeedCmd,GoingAcceleration*10,-20);
				//SpeedCmd = LinearGeneration(RunningTime2,GoingAcceleration*10, PrePullingSpeed,-PullingSpeed,-20); //-EpsilonPulling means the spd is negative
			}
			Timer3CountPeriod = CalculateTimer3Period (MotorDriver, SpeedCmd);
		}

		else
		{
			// Ramping the speed cmd
			//RunningTime += SampleTime;
			if (_direction) // go down
			{
				//SpeedCmd = LinearGeneration(RunningTime,GoingAcceleration*10,0,0,PullingSpeed); //-EpsilonPulling means the spd is negative
				LinearGeneration(&SpeedCmd,GoingAcceleration*10,PullingSpeed);
			}
			else // go up
			{
				//SpeedCmd = LinearGeneration(RunningTime,-GoingAcceleration*10,0,-PullingSpeed,0); //-EpsilonPulling means the spd is negative
				LinearGeneration(&SpeedCmd,-GoingAcceleration*10,-PullingSpeed);
			}

			if (SpeedCmd != 0)
			{
				// Calculate Timer3CountPeriod to generate pulse
				Timer3CountPeriod = CalculateTimer3Period (MotorDriver, SpeedCmd);
				//Timer3CountPeriod = (int)((float)(120000000.0/(fabs(SpeedCmd)*(float)EncoderResolution)) + 0.5);
			}
			//PrePullingSpeed = SpeedCmd;
		}

		if (abs(RefPulsePosition - MotorEncPulse + OriginPulse) <= 100) // Reach the bottom position
		{
			//RunningTime = 0;
			//RunningTime2 = 0;
			Timer3CountPeriod = 0;
			SpeedCmd = 0;
			//PrePullingSpeed = 0;
			StopPulseGenerating();
			return true;
		}
	}
	return false;
}

void InitGoingToStartingPosition ()
{
	if (MotorDriver) // HIGEN FDA7000
	{
		//if (PositionPulseCmd*EgearRatio < PullingBotomPulseCmdPosition) // Then going down to the bottom
		if (MotorEncPulse - OriginPulse < PullingBotomPulseCmdPosition)
		{
			StartAccleratePulling = false;
			Direction = true; // false = move up, true = move down
			// Start going down to the bottom position
			PRIsToggled = false; // false = Dropping Down
			DisableSTOP(); // Disable the stop
			// Calculate Timer3CountPeriod to generate pulse
			// Timer3CountPeriod = CalculateTimer3Period (MotorDriver, PullingSpeed);
			//Timer3CountPeriod = (int)((float)(120000000.0/((PullingSpeed)*(float)EncoderResolution)) + 0.5); // Set going down speed
			StartPulseGenerating();
		}
		if (MotorEncPulse - OriginPulse > PullingBotomPulseCmdPosition) // Then going up to the initial position
		{
			StartAccleratePulling = false;
			Direction = false; // false = move up, true = move down
			// Start going down to the bottom position
			PRIsToggled = true; // false = Dropping Down, true = Going up
			DisableSTOP(); // Disable the stop
			// Calculate Timer3CountPeriod to generate pulse
			// Timer3CountPeriod = CalculateTimer3Period (MotorDriver, PullingSpeed);
			//Timer3CountPeriod = (int)((float)(120000000.0/((PullingSpeed)*(float)EncoderResolution)) + 0.5); // Set going down speed
			StartPulseGenerating();
		}
		if (MotorEncPulse - OriginPulse == PullingBotomPulseCmdPosition)  // Object is at the bottom, then start pulling up
		{
			StartAccleratePulling = true;
			StartBraking = false; // go to Accerlerating Stage
			Direction = false;

			PRIsToggled = true; // true = pulling up.
			DisableSTOP(); // Disable the stop
			StartPulseGenerating();
		}
	}
	else // ASDA A3, small motor
	{
		if (MotorEncPulse - OriginPulse < PullingBotomPulseCmdPosition) // Then going down to the bottom
		{
			StartAccleratePulling = false;
			Direction = true; // false = move up, true = move down
			// Start going down to the bottom position
			PRIsToggled = false; // false = Dropping Down
			DisableSTOP(); // Disable the stop
			StartPulseGenerating();
		}
		if (MotorEncPulse - OriginPulse > PullingBotomPulseCmdPosition) // Then going up to the initial position
		{
			StartAccleratePulling = false;
			Direction = false; // false = move up, true = move down
			// Start going down to the bottom position
			PRIsToggled = true; // false = Dropping Down, true = Going up
			DisableSTOP(); // Disable the stop
			// Calculate Timer3CountPeriod to generate pulse
			// Timer3CountPeriod = CalculateTimer3Period (MotorDriver, PullingSpeed);
			//Timer3CountPeriod = (int)((float)(120000000.0/((PullingSpeed)*(float)EncoderResolution)) + 0.5); // Set going down speed
			StartPulseGenerating();
		}
		if (MotorEncPulse - OriginPulse == PullingBotomPulseCmdPosition)  // Object is at the bottom, then start pulling up
		{
			StartAccleratePulling = true;
			StartBraking = false; // go to Accerlerating Stage
			Direction = false;

			PRIsToggled = true; // true = pulling up.
			DisableSTOP(); // Disable the stop
			StartPulseGenerating();
		}
	}
}
// Init variable for running
void InitializeRunning (uint8_t Mode)
{
	switch (Mode)
	{
		case 1: // Dropping Mode
			StartRunning = true;
			StartDropping = true;
			StartBraking = false;
			Direction = true; // variable to show the direction, false = move up, true = move down
			StartPulling = false;
			CompleteRunning = false;

			StartPulseCmdCounting = false;

			PRIsToggled = false; // false = Dropping Down. change to true/false to change the direction: pulling or dropping
			//PreAccRef = -9.6;
			DisableSTOP(); // Disable the stop
			StartPulseGenerating();
			break;
		case 2: // Pulling Mode
			StartRunning = true;
			StartDropping = false;
			CompleteRunning = false;

			PositionPulseCmd = 0;
			InitGoingToStartingPosition ();

			break;
		case 3: // Pull and Drop mode, Same like Pulling Mode
			StartRunning = true;
			CompleteRunning = false;
			StartPulling = true; // Pulling Stage Firse
			StartDropping = false;
			StartPulseCmdCounting = false;
			InitGoingToStartingPosition ();
			break;
		default:
			break;
	}
}

bool PullingExperiment ()
{
	if (CompleteRunning)
	{
		return true;
	}
	else
	{
		if (!StartAccleratePulling)
		{
			if (CheckGoingToRefPosition(true, PullingBotomPulseCmdPosition)) // if at the bottom position, then wait for some seconds
			{
				if (WaitingMiliSecond(5000)) // Wait for 5 seconds = 5000ms
				{
					StartAccleratePulling = true; // turn on flag to start acclerating pulling
					Direction = false;
					StartBraking = false;

					PRIsToggled = true; // true = pulling up.
					DisableSTOP(); // Disable the stop
					StartPulseGenerating();
					PreAccRef = AccRef;

					TargetPosition = FlyingPosPulseCmd; // First, go to max-speed point

					//RunningTime = 0;
				}
				else return false;
			}
			else return false;
		}
		else // Start accelerate pulling
		{
			if (!StartBraking) // Accelerating Stage
			{
				if (IsReachTargetPosition)
				{
					StartBraking = true;
					PreAccRef = AccRef;

					StartPulseCmdCounting = false;

					PositionPulseCmd = 0; // Reset the pulse count variable
					TargetPosition = (int)(EncoderResolution*PullingDecelDistance/(2*3.14*DrumRadius));

					IsReachTargetPosition = false; // Reset the flag
				}
				//RunningTime += SampleTime;

				//AccRef = LinearGeneration(RunningTime,-8,PreAccRef,-9.6-PullingAccel, PreAccRef); // ramping the reference signal
				//LinearGeneration(&AccRef,-8,-9.6-PullingAccel); // ramping the reference signal
				AccRef = -9.6-PullingAccel;
				// Calculate speed cmd


				if (IsOpenLoopControl) // Use open-loop control
				{
					//SpeedCmd = LinearGeneration(RunningTime,-PullingEpsilonAcc*10,0,-PullingMaxSpeed,0);// Feedforward term
					//SpeedCmd = LinearGeneration(RunningTime,(AccRef+9.6)*10/DrumRadius,0,-PullingMaxSpeed,0);// Feedforward term

					//LinearGeneration(&SpeedCmd,(AccRef+9.6)*10/DrumRadius,-PullingMaxSpeed);
					SpeedCmd += SampleTime*0.001*(AccRef + 9.6)*10/DrumRadius;
					if (SpeedCmd <= -PullingMaxSpeed)
						SpeedCmd = -PullingMaxSpeed;
					if (SpeedCmd >= 0)
						SpeedCmd = 0;
					StartPulseCmdCounting = true;
				}
				else // Use closed loop Control
				{
					AccSet = PIDCalculate(AccRef,AccZ,-5,5, true); // PID term
					AccSet = AccSet + AccRef + 9.6;
					//SpeedCmd = LinearSpeedGeneration(RunningTime,AccSet/DrumRadius,0,-PullingMaxSpeed,0); // Feedforwad
					//SpeedCmd = LinearGeneration(RunningTime,AccSet*10/DrumRadius,0,-PullingMaxSpeed,0);// Feedforward term
					LinearGeneration(&SpeedCmd,AccSet*10/DrumRadius,-PullingMaxSpeed);// Feedforward term
				}

				if (SpeedCmd != 0)
				{
					// Calculate Timer3CountPeriod to generate pulse
					Timer3CountPeriod = CalculateTimer3Period (MotorDriver, SpeedCmd);
					//Timer3CountPeriod = (int)((float)(120000000.0/(fabs(SpeedCmd)*(float)EncoderResolution)) + 0.5);
				}
//				else
//				{
//					StopPulseGenerating();
//				}
				//if (fabs(SpeedCmd) >= PullingMaxSpeed)
				//if ((fabs(ObjectPosition - PullingTotalDistance) >= PullingAccelDistance) || (SpeedCmd <= -PullingMaxSpeed)) // Switch to braking mode

//				if ( (((PullingBotomPulseCmdPosition - MotorEncPulse + OriginPulse)) >= FlyingPosPulseCmd) || ( fabs(SpeedCmd) >= PullingMaxSpeed) ) // unit: pulses)
//				{
//					//RunningTime = 0;
//					StartBraking = true;
//					//TransitionSpeed = SpeedCmd;
//					PreAccRef = AccRef;
//				}
			}
			else // Braking Stage
			{
				//RunningTime += SampleTime;
				//AccRef = LinearGeneration(RunningTime,8,PreAccRef, PreAccRef, -9.6+PullingDecel); // ramping the reference signal
				//LinearGeneration(&AccRef,60,-9.6+PullingDecel); // ramping the reference signal
				AccRef = -9.6+PullingDecel;
				if (IsOpenLoopControl)
				{
					//SpeedCmd = LinearGeneration(RunningTime,(AccRef+9.6)*10/DrumRadius,TransitionSpeed,TransitionSpeed,0);
					SpeedCmd += SampleTime*0.001*(AccRef+9.6)*10/DrumRadius;

					if (SpeedCmd <= -PullingMaxSpeed)
						SpeedCmd = -PullingMaxSpeed;
					if (SpeedCmd >= 0)
						SpeedCmd = 0;
				}
				else
				{
					AccSet = PIDCalculate(AccRef,AccZ,-5,5, true); // PID term
					AccSet = AccSet + AccRef + 9.6;
					//SpeedCmd = LinearGeneration(RunningTime,AccSet*10/DrumRadius,TransitionSpeed,TransitionSpeed,0);// Feedforward term
					SpeedCmd += SampleTime*0.001*AccSet*10/DrumRadius;
					if (SpeedCmd >= 0)
						SpeedCmd = 0;
				}

				if (SpeedCmd != 0)
				{
					// Calculate Timer3CountPeriod to generate pulse
					Timer3CountPeriod = CalculateTimer3Period (MotorDriver, SpeedCmd);
					StartPulseCmdCounting = true;
					//Timer3CountPeriod = (int)((float)(120000000.0/(fabs(SpeedCmd)*(float)EncoderResolution)) + 0.5);
				}
//				else
//				{
//					StopPulseGenerating();
//				}
				if (IsReachTargetPosition) // Reach the top position
				{
					//RunningTime = 0;
					SpeedCmd = 0; // reset/ stop

					//ResetPIDController();

					AccRef = - 9.8;

					StartBraking = false;
					StartAccleratePulling = false;
					StopPulseGenerating();
					CompleteRunning = true; // Set this flag to return true in the next time

					StartPulseCmdCounting = false;
					PositionPulseCmd = 0;
					TargetPosition = 0;
					IsReachTargetPosition = false;
					return true;
				}
			}
		}
		return false;
	}
}

bool PullAndDrop ()
{
	if (CompleteRunning)
	{
		return true;
	}
	else
	{
		// BEGIN PULLING UP
		if (StartPulling && !StartDropping) // Pulling Task
		{
			// First Pulling up including going to the initial position
			if (!StartAccleratePulling)
			{
				// Going to the initial position first
				// Direction = true => going down
				// Direction = false => Going up
				if (CheckGoingToRefPosition(Direction, PullingBotomPulseCmdPosition)) // if at the bottom position, then wait for some seconds
				{
					//BottomPosition = ObjectPosition;
					if (WaitingMiliSecond(5000)) // Wait for 5 seconds = 5000ms
					{
						StartAccleratePulling = true; // turn on flag to start acclerating pulling
						Direction = false; // false = move up to count the position pulse cmd
						StartBraking = false;

						PreAccRef = AccRef;

						PRIsToggled = true; // true = pulling up.
						DisableSTOP(); // Disable the stop
						StartPulseGenerating();

						TargetPosition = FlyingPosPulseCmd; // First, go to max-speed point
					}
					else return false;
				}
				else return false;
			}
			else // Start accelerated pulling
			{
				if (!StartBraking) // Accelerating Stage
				{
					if (IsReachTargetPosition)
					{
						StartBraking = true;
						PreAccRef = AccRef;

						StartPulseCmdCounting = false;

						PositionPulseCmd = 0; // Reset the pulse count variable
						TargetPosition = (int)(EncoderResolution*PullingDecelDistance/(2*3.14*DrumRadius)); // Stop at the top point of the flying (zero-speed point)

						IsReachTargetPosition = false; // Reset the flag
					}

					AccRef = -9.6-PullingAccel;

					//AccRef = LinearGeneration(RunningTime,-8,PreAccRef,-9.6-PullingAccel, PreAccRef); // ramping the reference signal

					//LinearGeneration(&AccRef,-16,-9.6-PullingAccel); // ramping the reference signal

					//LinearGeneration(&SpeedCmd,(AccRef+9.6)*10/DrumRadius,-PullingMaxSpeed);

					SpeedCmd += SampleTime*0.001*(AccRef + 9.6)*10/DrumRadius;
					if (SpeedCmd <= -PullingMaxSpeed)
						SpeedCmd = -PullingMaxSpeed;
					StartPulseCmdCounting = true;

					if (SpeedCmd != 0)
					{
						// Calculate Timer3CountPeriod to generate pulse
						Timer3CountPeriod = CalculateTimer3Period (MotorDriver,SpeedCmd);
						//Timer3CountPeriod = (int)((float)(120000000.0/(fabs(SpeedCmd)*(float)EncoderResolution)) + 0.5);
					}
//					else
//					{
//						//StopPulseGenerating();
//						PulseGenerationFlag = false; // Stop pulse generation
//					}
				}
				else // Braking Stage
				{
					//AccRef =  = -9.8+PullingDecel;

					//if (IsReachTargetPosition) // Reach the top position

					if (SpeedCmd == 0 || IsReachTargetPosition)
					{
						StopPulseGenerating();

						//PreAccRef = AccRef;

						//StartDropping = true;
						//StartBraking = false;
						//Direction = true; // variable to show the direction, false = move up, true = move down
						//StartPulling = false;

						StartPulseCmdCounting = false;
						PRIsToggled = false; // false = Dropping Down. change to true/false to change the direction: pulling or dropping

						//InitializeRunning(DroppingMode);

						PositionPulseCmd = 0;
						TargetPosition = BottomFreeDropPulseCmd; // First, go to max-speed point
						IsReachTargetPosition = false;


						//RunningTime = 0;
						SpeedCmd = 0; // reset/ stop

						//StartPulling = false; // flag to finish Pulling Stage

						StartWaiting = true; // Switch to waiting stage

//						StartPulseCmdCounting = false;
//						PositionPulseCmd = 0;
//						TargetPosition = 0;
//						IsReachTargetPosition = false;
//						TopPulseCmd = MotorEncPulse - OriginPulse;

					}
					AccRef = -9.6+PullingDecel;

					//LinearGeneration(&AccRef,FlyAccSlope,-9.6+PullingDecel);

					if (IsOpenLoopControl) // Use open-loop control
					{
						//SpeedCmd = LinearSpeedGeneration(RunningTime,PullingEpsilonDec,TransitionSpeed,-PullingMaxSpeed,0); // Feedforward term
						//LinearGeneration(&SpeedCmd,(AccRef + 9.6)*10/DrumRadius,0);// Feedforward term
						SpeedCmd += SampleTime*0.001*(AccRef + 9.6)*10/DrumRadius;
						if (SpeedCmd <= -810)
							SpeedCmd = -810;
						if (SpeedCmd >= 0)
							SpeedCmd = 0;
					}
					else // Use closed loop Control
					{
						AccSet = PIDCalculate(AccRef,AccZ,-10,10, true); // PID term
						AccSet = AccSet + AccRef + 9.6;
						//SpeedCmd = LinearSpeedGeneration(RunningTime,AccSet/DrumRadius,TransitionSpeed,TransitionSpeed,DroppingMaxSpeed); // Feedforward term
						//SpeedCmd = LinearGeneration(RunningTime,AccSet*10/DrumRadius,TransitionSpeed,-PullingMaxSpeed,0);// Feedforward term
						//LinearGeneration(&SpeedCmd,AccSet*10/DrumRadius,TransitionSpeed,0);// Feedforward term
						SpeedCmd += SampleTime*0.001*AccSet*10/DrumRadius;
						if (SpeedCmd >= 0)
							SpeedCmd = 0;
					}

					if (SpeedCmd != 0)
					{
						// Calculate Timer3CountPeriod to generate pulse
						Timer3CountPeriod = CalculateTimer3Period (MotorDriver,SpeedCmd);
						StartPulseCmdCounting = true;
						// Timer3CountPeriod = (int)((float)(120000000.0/(fabs(SpeedCmd)*(float)EncoderResolution)) + 0.5);
					}
//					else
//					{
//						PulseGenerationFlag = false;
//					}



//					if ((ObjectPosition <= 0.2) || (SpeedCmd >= 0)) // condition to switch to dropping stage
//					{
//						SpeedCmd = 0; //
//						TopPulseCmd = MotorEncPulse - OriginPulse;
//
//						if (IsOpenLoopControl)
//						{
//							StartWaiting = true; // Switch to waiting stage
//						}
//						else // closed-loop control
//						{
//							PreAccRef = AccRef;
//							StartDropping = true;
//							StartBraking = false;
//							Direction = true; // false = move up, true = move down
//							StartPulling = false;
//
//							// Reverse pulse generation direction to switch to dropping
//							HAL_GPIO_WritePin(PC8_PR_GPIO_Port, PC8_PR_Pin, GPIO_PIN_SET); // Set CW direction
//							HAL_GPIO_WritePin(PE9_TIM1_CH1_PFIN_GPIO_Port, PE9_TIM1_CH1_PFIN_Pin,GPIO_PIN_SET);
//							PRIsToggled = false; // false = Dropping Down. change to true/false to change the direction: pulling or dropping
//							PulseGenerationFlag = true;
//						}
//						//InitializeRunning(DroppingMode);
//					}
				}
			}
		}
		// END Pulling Task

//		// BEGIN WAITING TASK
//		// Wait for some time before dropping

// BEGIN Delay before dropping

//
		if (StartWaiting)
		{
			if (WaitingMiliSecond(StoppingTime))
			{
				StartWaiting = false;

				PreAccRef = AccRef;

				StartDropping = true;
				StartBraking = false;
				Direction = true; // variable to show the direction, false = move up, true = move down
				StartPulling = false;

				StartPulseCmdCounting = false;
				PRIsToggled = false; // false = Dropping Down. change to true/false to change the direction: pulling or dropping

				//InitializeRunning(DroppingMode);

				TargetPosition = BottomFreeDropPulseCmd; // First, go to max-speed point
				StartPulseGenerating();
			}
		}

// END WAITING

		// BEGIN DROPPING TASK
		if (StartDropping && !StartPulling)
		{
				// ACCLERATING DROPPING STAGE
				if (!StartBraking) // Accelerating Stage
				{
					if (IsReachTargetPosition) // Reach dropping distance
					{
						StartBraking = true;

						StartPulseCmdCounting = false;
						PositionPulseCmd = 0;
						//TargetPosition = 0;

						TargetPosition = DroppingDecelPulseCmd;
						IsReachTargetPosition = false;
					}

//					if ((fabs(MotorEncPulse - OriginPulse - TopPulseCmd)) >= BottomFreeDropPulseCmd) // constraint the dropping distance
//					{
//						//RunningTime = 0;
//						StartBraking = true;
//						//TransitionSpeed = SpeedCmd;
//						// Reset PID Controller
//						IntergraError = 0;
//						PreError = 0;
//
//						//PreAccRef = AccRef;
//						//ResetPIDController();
//					}


					AccRef = -9.6+DroppingAccel;
					// Calculate speed cmd
					//RunningTime += SampleTime;

					//AccRef = LinearGeneration(RunningTime,16,PreAccRef, PreAccRef,-9.6+DroppingAccel);

//					if (PreAccRef > (-9.6+DroppingAccel))
//					{
//						AccRef = LinearGeneration(RunningTime,-16,PreAccRef,-9.6+DroppingAccel, PreAccRef);
//					}
//					else
//					{
//						AccRef = LinearGeneration(RunningTime,16,PreAccRef, PreAccRef,-9.6+DroppingAccel);
//					}

					if (IsOpenLoopControl) // Use open-loop control
					{

						//LinearGeneration(&SpeedCmd,(AccRef+9.6)*10/DrumRadius,DroppingMaxSpeed);// Feedforward term

						SpeedCmd += SampleTime*0.001*(AccRef+9.6)*10/DrumRadius;
						if (SpeedCmd >= DroppingMaxSpeed) // Saturation
							SpeedCmd = DroppingMaxSpeed;
					}
					else // Use closed loop Control
					{
						AccSet = PIDCalculate(AccRef,AccZ,-8,8, true); // PID term
						AccSet = AccSet + DroppingAccel;
						//SpeedCmd = LinearSpeedGeneration(RunningTime,AccSet/DrumRadius,0,0,DroppingMaxSpeed); // Feedforward term
						//SpeedCmd = LinearGeneration(RunningTime,AccSet*10/DrumRadius,0,0,DroppingMaxSpeed); // Feedforward term
						//LinearGeneration(&SpeedCmd,AccSet*10/DrumRadius,DroppingMaxSpeed); // Feedforward term

						SpeedCmd += SampleTime*0.001*AccSet*10/DrumRadius;
						if (SpeedCmd >= DroppingMaxSpeed) // Saturation
							SpeedCmd = DroppingMaxSpeed;
						StartPulseCmdCounting = true;
					}

					if (SpeedCmd != 0)
					{
						// Calculate Timer3CountPeriod to generate pulse
						Timer3CountPeriod = CalculateTimer3Period (MotorDriver,SpeedCmd);
						StartPulseCmdCounting = true;
						// Timer3CountPeriod = (int)((float)(120000000.0/(fabs(SpeedCmd)*(float)EncoderResolution)) + 0.5);
					}
				}
				// END ACCELERATED DROPPING

				// BEGIN BRAKING STAGE
				else
				{
					//RunningTime += SampleTime;

					//AccRef = LinearGeneration(RunningTime,-8,PreAccRef,-9.6-DroppingDecel, PreAccRef);

					//LinearGeneration(&AccRef,-BrakeAccSlope,-9.6-DroppingDecel);

					AccRef = -9.6-DroppingDecel;

	//				FeedFWSpeedCmd = LinearSpeedGeneration(RunningTime,-DroppingEpsilonDec,DroppingMaxSpeed,0,DroppingMaxSpeed);
	//				PIDSpeedCmd = PIDCalculate(AccRef,AccZ,-850,850, true); // PID term

					//SpeedCmd = LinearSpeedGeneration(RunningTime,-DroppingEpsilonDec,TransitionSpeed,0,TransitionSpeed);
					//SpeedCmd = LinearGeneration(RunningTime,-DroppingEpsilonDec*10,TransitionSpeed,0,TransitionSpeed);
					//SpeedCmd = LinearSpeedGeneration(RunningTime,-DroppingEpsilonDec,DroppingMaxSpeed,0,DroppingMaxSpeed);
					//SpeedCmd = LinearGeneration(RunningTime,(AccRef+9.6)*10/DrumRadius,TransitionSpeed,0,2*TransitionSpeed);
					//LinearGeneration(&SpeedCmd,(AccRef+9.6)*10/DrumRadius);
					SpeedCmd += SampleTime*0.001*(AccRef+9.6)*10/DrumRadius;
					if (SpeedCmd <= 0)
						SpeedCmd = 0;

					if (SpeedCmd != 0)
					{
						// Calculate Timer3CountPeriod to generate pulse
						Timer3CountPeriod = CalculateTimer3Period (MotorDriver,SpeedCmd);
						StartPulseCmdCounting = true;
						//Timer3CountPeriod = (int)((float)(120000000.0/(fabs(SpeedCmd)*(float)EncoderResolution)) + 0.5);
					}

					if (SpeedCmd <= 0 || IsReachTargetPosition) // Stop braking
					{
						//RunningTime = 0;
						SpeedCmd = 0; // reset/ stop

						ResetPIDController();
						AccRef = - 9.6;

						StartDropping = false; //
						StartBraking = false;
						StopPulseGenerating();
						CompleteRunning = true;
						return true;
					}
				}
				// END BRAKING STAGE
		}
		// END DROPPING TASK.
		return false;
	}
}
bool Dropping() // Dropping Program
// Mode = false -> Manual Running
// Mode = true -> Automatic Running
// return true if finishing, else return false while running
// StoppingDelayTime (ms): the time duration of stopping before pulling
{
	if (CompleteRunning)
	{
		return true;
	}
	else
	{
		if (StartDropping && !StartPulling) // Dropping Stage
		{
			if (!StartBraking) // Accelerating Stage
			{
				// Calculate speed cmd
				//RunningTime += SampleTime;

				// Ramping the reference signal
				// LinearGeneration(&AccRef,8,-9.6+DroppingAccel);
				AccRef = -9.6+DroppingAccel;
				if (IsOpenLoopControl) // Use open-loop control
				{
					//SpeedCmd = LinearSpeedGeneration(RunningTime,DroppingEpsilonAcc,0,0,DroppingMaxSpeed); // Feedforward term
					LinearGeneration(&SpeedCmd,DroppingEpsilonAcc*10,DroppingMaxSpeed); // Feedforward term
				}
				else // Use closed loop Control
				{

//				FeedFWSpeedCmd = LinearSpeedGeneration(RunningTime,DroppingEpsilonAcc,0,0,DroppingMaxSpeed); // Feedforward term
//				PIDSpeedCmd = PIDCalculate(AccRef,AccZ,-850,850, true); // PID term

					//SpeedCmd = PIDSpeedCmd; // PID controller only
					//SpeedCmd = FeedFWSpeedCmd;
					//SpeedCmd = FeedFWSpeedCmd + PIDSpeedCmd; // Feedforward controller

					AccSet = PIDCalculate(AccRef,AccZ,-5,5, true); // PID term
					AccSet = AccSet + AccRef + 9.6;
					//SpeedCmd = LinearSpeedGeneration(RunningTime,AccSet/DrumRadius,0,0,DroppingMaxSpeed); // Feedforward term
					LinearGeneration(&SpeedCmd,AccSet*10/DrumRadius,DroppingMaxSpeed); // Feedforward term
				}

				if (SpeedCmd != 0)
				{
					// Calculate Timer3CountPeriod to generate pulse
					Timer3CountPeriod = CalculateTimer3Period (MotorDriver,SpeedCmd);
					//Timer3CountPeriod = (int)((float)(120000000.0/(fabs(SpeedCmd)*(float)EncoderResolution)) + 0.5);
				}
				else
				{
					StopPulseGenerating();
				}
				//if (SpeedCmd >= DroppingMaxSpeed || fabs(ObjectPosition) >= DroppingAccelDistance )
				if (ObjectPosition >= DroppingAccelDistance)
				{
					//RunningTime = 0;
					StartBraking = true;
					//ResetPIDController();
					IntergraError = 0;
					PreError = 0;
				}
			}
			else // Braking Stage
			{
				//RunningTime += SampleTime;

				AccRef = -9.8-DroppingDecel;

				//LinearGeneration(&AccRef,-8,-9.6-DroppingDecel);
//				FeedFWSpeedCmd = LinearSpeedGeneration(RunningTime,-DroppingEpsilonDec,DroppingMaxSpeed,0,DroppingMaxSpeed);
//				PIDSpeedCmd = PIDCalculate(AccRef,AccZ,-850,850, true); // PID term

				//SpeedCmd = LinearSpeedGeneration(RunningTime,-DroppingEpsilonDec,TransitionSpeed,0,TransitionSpeed);
				//SpeedCmd = LinearGeneration(RunningTime,-DroppingEpsilonDec*10,TransitionSpeed,0,TransitionSpeed);
				SpeedCmd += SampleTime*0.001*(AccRef+9.6)*10/DrumRadius;
				if (SpeedCmd <= 0)
					SpeedCmd = 0;
				if (SpeedCmd >= 810)
					SpeedCmd = 810;

				if (SpeedCmd != 0)
				{
					// Calculate Timer3CountPeriod to generate pulse
					Timer3CountPeriod = CalculateTimer3Period (MotorDriver,SpeedCmd);
					// Timer3CountPeriod = (int)((float)(120000000.0/(fabs(SpeedCmd)*(float)EncoderResolution)) + 0.5);
				}
				else
				{
					StopPulseGenerating();
				}
				if (SpeedCmd <= 0) // Stop braking
				{
					StopPulseGenerating();
					//RunningTime = 0;
					SpeedCmd = 0; // reset/ stop
					ResetPIDController();
					//AccRef = - 9.8;

					StartDropping = false; //
					StartBraking = false;
				}
			}
		}
		if (!StartDropping && !StartPulling) // Waiting for some seconds before pulling up
		{
	//		POSReach = HAL_GPIO_ReadPin(CN1_47_INSPD_INPOS_GPIO_Port,CN1_47_INSPD_INPOS_Pin);
	//		if (POSReach) // Check if position is reached or not
	//		{

				if (WaitingMiliSecond(StoppingTime)) // Wait some second
				{
					// Change to pulling stage
					StartPulling = true;
					// Calculate Timer3CountPeriod to generate pulse
					//Timer3CountPeriod = CalculateTimer3Period(MotorDriver, PullingSpeed);
					// Timer3CountPeriod = (int)((float)(120000000.0/((float)PullingSpeed*(float)EncoderResolution)) + 0.5);
					// Start pulling to the home position
	//				StepPulseCmd = (int)MotorEncPulse/8; // calculate # of pulse cmd to return to the top postion
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
			if (MotorDriver) // FDA7000, big model
			{
				if (CheckGoingToRefPosition(false, 0))
				{
		//			POSReach = HAL_GPIO_ReadPin(CN1_47_INSPD_INPOS_GPIO_Port,CN1_47_INSPD_INPOS_Pin);
		//			if (POSReach) // Reaching to the top/home postion
		//			{
						StartPulling = false;
						CompleteRunning = true; // to return true next time

						StopPulseGenerating();
						return true;
		//			}
				}
			}
			else // ASDA A3
			{
				if (CheckGoingToRefPosition(false, 0))
				{
						StartPulling = false;
						CompleteRunning = true; // to return true next time

						StopPulseGenerating();
						return true;
				}
			}
		}
		return false;
	}
}

void StopExperiment ()
{
	TargetPosition = 0;
	PositionPulseCmd = 0;
	StartPulseCmdCounting = false;
	IsReachTargetPosition = false;

	// Reset all the flag and state
	StartRunning = false;
	StartDropping = false;
	StartPulling = false;
	StartBraking = false;
	StartAccleratePulling = false;
	StopPulseGenerating(); // Stop pulse generation
	//ResetPIDController();
	//RunningTime = 0;
	Timer3CountPeriod = 0;
	SpeedCmd = 0;
}
void CalculateRunningSpec () // Calculate running parameters
{
	GoingAcceleration = 0.1*PullingSpeed/RampingGoingSpdTime; // to rad/s2

	// Dropping Stage Calculations
	DroppingMaxSpeed = (uint16_t)(10*sqrt(2*DroppingAccel*DroppingAccelDistance)/(DrumRadius)); // in rpm; 10~60/2pi
	DroppingEpsilonAcc = DroppingAccel/DrumRadius;
	DroppingEpsilonDec = DroppingDecel/DrumRadius;

	DroppingDecelDistance = (pow(DroppingMaxSpeed*2*3.14*DrumRadius/60,2))/(2*DroppingDecel); // unit: m
	DroppingDecelPulseCmd = ((int)(EncoderResolution*DroppingDecelDistance/(2*3.14*DrumRadius))); // unit: pulses

	BottomFreeDropPulseCmd = ((int)(EncoderResolution*DroppingAccelDistance/(2*3.14*DrumRadius)));

	// Pulling Stage Calculations
	PullingMaxSpeed = (uint16_t)(9.5*sqrt(2*PullingAccel*PullingAccelDistance)/(DrumRadius)); // in rpm
	PullingEpsilonAcc = PullingAccel/DrumRadius;
	PullingEpsilonDec = PullingDecel/DrumRadius;

	PullingDecelTime = (PullingMaxSpeed * (2*3.14/60) * DrumRadius) / PullingDecel; //Motor Deceleration time

	//PullingDecelDistance = 0.5*PullingMaxSpeed*(2*3.14/60)*DrumRadius*PullingDecelTime;

	//PullingDecelDistance = (pow(PullingMaxSpeed*2*3.14*DrumRadius/60,2))/(2*9.8); // Object flying distance
	PullingDecelDistance = (pow(PullingMaxSpeed*2*3.14*DrumRadius/60,2))/(2*PullingDecel); // Object flying distance

	PullingTotalDistance = ((float)PullingAccelDistance + (float)PullingDecelDistance)*DistCoeff;
	PullingBotomPulseCmdPosition = (int)((float)EncoderResolution*(float)PullingTotalDistance/((float)(2*3.14*DrumRadius))); // unit: pulses

	FlyingPosPulseCmd = ((int)(EncoderResolution*PullingAccelDistance/(2*3.14*DrumRadius)));


	WaitingTime = (uint16_t)(((PullingMaxSpeed*(2*3.14/60)*DrumRadius)/9.8 - PullingDecelTime)*1000*2*0.7); // *1000 to convert to ms; *2 for both flying up and dropping down
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

	// Dropping Stage Params
	DroppingAccelDistance = Params[4];
	DroppingAccel  = Params[5];
	DroppingDecel = Params[6];


	// Pulling Stage Params
	PullingAccelDistance = Params[7];
	PullingAccel = Params[8];
	PullingDecel = Params[9];

	// PID Controller params
	FlyAccSlope = Params[10];
	DistCoeff = Params[11];
	//Kd = Params[12];
	BrakeAccSlope = Params[12];

	CalculateRunningSpec ();
}

void ProcessReceivedCommand () // Proceed the command from the UI
{
	//ExtractMotionCode(); // Extract data to MotionCode
	switch ((int)MotionCode[0])
	{
		case 44: //Emergency Stop Change to 44 to avoid data confusion
			if ((int)MotionCode[1] == 0) // 44/0
			{
				Estop(); // Estop button on the UI
				PulseGenerationFlag = false; // Stop generating pulses
				StartDropping = false;
				StartPulling = false;
				EMO = true;
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
				}
				else // ASDA A3
				{
					StopPulseGenerating();
					StopExperiment();
					IsHoming = false;
					IsStepPulseCmd = false;
					JoggingMoveUp = false;
				}
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
					JoggingMoveUp = true;
					if (PositionControlMode) // If the control Mode is Position Mode
					{
						if (!MotorDriver) // ASDA-A3
						{
//							if (OverTopLimit) // Reach the top limit
//							{
//								break;
//							}
						}
						// Calculate Timer3CountPeriod to generate pulse
						Timer3CountPeriod = CalculateTimer3Period(MotorDriver,JogSpeed);
						//Timer3CountPeriod = (int)((float)(120000000.0/((float)JogSpeed*(float)EncoderResolution)) + 0.5);
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
						// Calculate Timer3CountPeriod to generate pulse
						Timer3CountPeriod = CalculateTimer3Period(MotorDriver,JogSpeed);
						//Timer3CountPeriod = (int)((float)(120000000.0/((float)JogSpeed*(float)EncoderResolution)) + 0.5);
						PRIsToggled = false; //
						Direction = true; // true = move down
						StartPulseGenerating(); // Reset PF, PR + Enable Timer + PulseGeneratingFlag = true
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
					InitializeRunning (ExperimentMode);
					EMO = false;
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
				//Timer3CountPeriod = (int)((float)(120000000.0/((float)JogSpeed*(float)EncoderResolution)) + 0.5);
				char JogSpeedBuff[10];
				TxPCLen = sprintf(JogSpeedBuff,"j%.de",JogSpeed);
				HAL_UART_Transmit(&huart6,(uint8_t *)JogSpeedBuff,TxPCLen,200); // Send to uart6 to check the params are set or not
				// = (60*10e6)/(JogSpeed*EncoderRelsolution*Timer3Period)
				// Where JogSpeed in rpm; EcoderRelsolution in pulses, Timer3Period in us
				// Timer3 period in us = 2 us
			}
//			else // Speed control mode
//			{
//					if ((int)MotionCode[1] == 0) // Write int Value
//					{
//						WriteIntData((uint16_t)MotionCode[2], (uint16_t)MotionCode[3], true);
//					}
//					else // Write float Value
//					{
//						DroppingAccelDistance = roundf(MotionCode[3] * 10)/10;
//						WriteFloatData((uint16_t)MotionCode[2], DroppingAccelDistance, true);
//					}
//			}
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
			//SaveParams(DrumRadius, DroppingAccelDistance, PullingSpeed, StoppingTime, DroppingAccel, SampleTime);
			SaveSystemParams();
			HAL_Delay(500);
			SaveSystemParams(); // Do twice times
			// Send back to the UI to notify
			char MessageBuffer[10];
			TxPCLen = sprintf(MessageBuffer,"r7/1e");
			HAL_UART_Transmit(&huart6,(uint8_t *)MessageBuffer,TxPCLen,100); // Send to uart6 to check the params are set or not
			break;

		case 8: // Request reading digital driver output
			if((int)MotionCode[1] == 1) {OutputDataRequest = true;} // 8/1 = request
			else OutputDataRequest = false; // 8/0 = stop request
			break;

		case 45: // Load saved parameters
			LoadSavedParam(MemoryAddress,Params);

			// General Params
			DrumRadius = Params[0];
			PullingSpeed = Params[1];
			StoppingTime = Params[2];
			SampleTime = Params[3];

			// Dropping Stage Params
			DroppingAccelDistance = Params[4];
			DroppingAccel  = Params[5];
			DroppingDecel = Params[6];


			// Pulling Stage Params
			PullingAccelDistance = Params[7];
			PullingAccel = Params[8];
			PullingDecel = Params[9];

			// PID Controller params
			Kp = Params[10];
			Ki = Params[11];
			BrakeAccSlope = Params[12];
			// Send to the GUI
			char ParamBuffer[60];
			TxPCLen = sprintf(ParamBuffer,"p%.2f/%d/%d/%d/%.1f/%.2f/%.2f/%.1f/%.2f/%.2f/%.3f/%.4f/%.4fe"
	                   ,DrumRadius, PullingSpeed, StoppingTime, SampleTime,
										 DroppingAccelDistance, DroppingAccel, DroppingDecel,
                     PullingAccelDistance, PullingAccel, PullingDecel,
										 Kp, Ki, BrakeAccSlope); // Combine to a string
			HAL_UART_Transmit(&huart6,(uint8_t *)ParamBuffer,TxPCLen,200); // Send to uart6 to check the params are set or not
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
				CalculateRunningSpec();
				char DrumRadiusBuffer[10];
				TxPCLen = sprintf(DrumRadiusBuffer,"r11/%.2fe",DrumRadius);
				HAL_UART_Transmit(&huart6,(uint8_t *)DrumRadiusBuffer,TxPCLen,200); // Send to uart6 to check the params are set or not
				break;
			}

		case 12: // Set DroppingAccelDistance
			if (StartRunning) // Setting is not available while running
			{
				InitializeRunning (ExperimentMode);
				break;
			}
			else
			{
				DroppingAccelDistance = MotionCode[1];

				CalculateRunningSpec();

				char DroppingAccelDistanceBuffer[10];
				TxPCLen = sprintf(DroppingAccelDistanceBuffer,"r12/%.1fe",DroppingAccelDistance);
				HAL_UART_Transmit(&huart6,(uint8_t *)DroppingAccelDistanceBuffer,TxPCLen,200); // Send to uart6 to check the params are set or not
				break;
			}

		case 13: // Set PullingSpeed
			// PullingSpeed is the maximum speed when homing or going to the initial Posion
			if (StartRunning) // Setting is not available while running
			{
				InitializeRunning (ExperimentMode);
				break;
			}
			else
			{
				PullingSpeed = MotionCode[1];
				GoingAcceleration = 0.1*PullingSpeed/RampingGoingSpdTime; // to rad/s2
				char PullingSpeedBuffer[10];
				TxPCLen = sprintf(PullingSpeedBuffer,"r13/%de",PullingSpeed);
				HAL_UART_Transmit(&huart6,(uint8_t *)PullingSpeedBuffer,TxPCLen,200); // Send to uart6 to check the params are set or not
				break;
			}

		case 14: // also start running -> unused now
			InitializeRunning (ExperimentMode);
			break;
		case 15: // Set DroppingAccel
			if (StartRunning)
			{
				InitializeRunning (ExperimentMode);
				break;
			}
			else
			{
				DroppingAccel = MotionCode[1];

				CalculateRunningSpec();

				char DroppingAccelBuffer[10];
				TxPCLen = sprintf(DroppingAccelBuffer,"r15/%.3fe",DroppingAccel);
				HAL_UART_Transmit(&huart6,(uint8_t *)DroppingAccelBuffer,TxPCLen,200); // Send to uart6 to check the params are set or not
				break;
			}
		case 16: // Set SampleTime
			if (StartRunning) // When the experiment is running, disable this fcn
			{
				InitializeRunning (ExperimentMode);
				break;
			}
			else
			{
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
			}

		case 17: // Reset MCU
			HAL_NVIC_SystemReset();
			break;
		case 18: // Servo Enable on/off
			if (MotionCode[1] == 1) // Servo Enable ON
				HAL_GPIO_WritePin(SerVoReset_PC4_18_GPIO_Port, SerVoReset_PC4_18_Pin, GPIO_PIN_SET); // Servo enable on
			else
				HAL_GPIO_WritePin(SerVoReset_PC4_18_GPIO_Port, SerVoReset_PC4_18_Pin, GPIO_PIN_RESET); // Servo enable OFF
			break;
		case 20: // Set Step Pulse Cmd
			if (StartRunning)
			{
				InitializeRunning (ExperimentMode);
				break;
			}
			else
			{
				StepPulseCmd = MotionCode[2];
				if (MotionCode[1] == 1) // CW, +
				{
					Direction = true;
					PRIsToggled = false;
					IsStepPulseCmd = true;
					// Calculate Timer3CountPeriod to generate pulse
					Timer3CountPeriod = CalculateTimer3Period(MotorDriver,JogSpeed);
					//Timer3CountPeriod = (int)((float)(120000000.0/((JogSpeed)*(float)EncoderResolution)) + 0.5);
					//Start Running
					StartPulseGenerating();
					DisableSTOP();
				}
				else // CCW, -
				{
					Direction = false;
					PRIsToggled = true;
					IsStepPulseCmd = true;
					// Calculate Timer3CountPeriod to generate pulse
					Timer3CountPeriod = CalculateTimer3Period(MotorDriver,JogSpeed);
					//Timer3CountPeriod = (int)((float)(120000000.0/((JogSpeed)*(float)EncoderResolution)) + 0.5);
					//Start Running
					StartPulseGenerating();
					DisableSTOP();
				}
				break;
			}
		case 27: // Set Running Mode
			if(StartRunning) // Setting is not available while running
			{
				InitializeRunning(ExperimentMode);
				break;
			}
			else
			{
				if (MotionCode[1] == 1) // RunningMode = true => Automatic Running
				{
					RunningMode = true; // Automatic
				}
				else
				{
					RunningMode = false; // Manual
				}
				char SammpleTimeBuffer[10];
				TxPCLen = sprintf(SammpleTimeBuffer,"g27/%de",RunningMode);
				HAL_UART_Transmit(&huart6,(uint8_t *)SammpleTimeBuffer,TxPCLen,200); // Send to uart6 to check the params are set or not
				break;
			}

		case 28: // Stop jog move up/down in Position Jog control;
			if (StartRunning) // Setting is not available while running
			{
				InitializeRunning(ExperimentMode);
				break;
			}
			else
			{
				StopPulseGenerating();
				JoggingMoveUp = false;
				//HAL_TIM_IC_Stop()
				break;
			}

//		case 30: // Set Driver parameters
//			       // 30/data type/adress/value
//						 // datatype: 1-> float; 0-> int
//			if (StartRunning)// Setting is not available while running
//			{
//				InitializeRunning(ExperimentMode);
//				break;
//			}
//			else
//			{
//				if (MotionCode[1] == 1) // Write float value
//				{
//					WriteFloatData(MotionCode[2], MotionCode[3], true);
//				}
//				else // Write Int Value
//				{
//					WriteIntData(MotionCode[2], MotionCode[3], true);
//				}
//				break;
//			}

		case 31: // Set Experiment Mode
			if (StartRunning) // Setting is not available while running
			{
				InitializeRunning(ExperimentMode);
				break;
			}
			else
			{
				ExperimentMode = MotionCode[1]; // 1=Dropping Mode;2 = Pulling; 3= Pulling->Dropping
				char SetModeBuff[8];
				TxPCLen = sprintf(SetModeBuff,"m%de",ExperimentMode);
				HAL_UART_Transmit(&huart6,(uint8_t *)SetModeBuff,TxPCLen,100); // Send to uart6 to check the params are set or not
				break;
			}

		case 32: // Set Pulling Accelerating Distance; Pulling Mode
			if (StartRunning) // Setting is not available while running
			{
				InitializeRunning(ExperimentMode);
				break;
			}
			else
			{
				PullingAccelDistance = MotionCode[1];

				CalculateRunningSpec();

				char PullingDistanceBuffer[10];
				TxPCLen = sprintf(PullingDistanceBuffer,"r32/%.1fe",PullingAccelDistance);
				HAL_UART_Transmit(&huart6,(uint8_t *)PullingDistanceBuffer,TxPCLen,100); // Send to uart6 to check the params are set or not
				break;
			}

		case 33: // Set Pulling AccRef in Pulling Mode
			if (StartRunning)// Setting is not available while running
			{
				InitializeRunning(ExperimentMode);
				break;
			}
			else
			{
				PullingAccel = MotionCode[1];

				CalculateRunningSpec();

				char AccRefPullingBuffer[10];
				TxPCLen = sprintf(AccRefPullingBuffer,"r33/%.2fe",PullingAccel);
				HAL_UART_Transmit(&huart6,(uint8_t *)AccRefPullingBuffer,TxPCLen,100); // Send to uart6 to check the params are set or not
				break;
			}

		case 34: // Set DroppingDecel, m/s2
			if (StartRunning)// Setting is not available while running
			{
				InitializeRunning(ExperimentMode);
			}
			else
			{
				DroppingDecel = MotionCode[1];

				CalculateRunningSpec();

				char DroppingDecelBuffer[10];
				TxPCLen = sprintf(DroppingDecelBuffer,"r34/%.2fe",DroppingDecel);
				HAL_UART_Transmit(&huart6,(uint8_t *)DroppingDecelBuffer,TxPCLen,100); // Send to uart6 to check the params are set or not
			}
			break;

		case 35: // Set Pulling Deceleration in m/s2
			if (StartRunning)// Setting is not available while running
			{
				InitializeRunning(ExperimentMode);
			}
			else
			{
				PullingDecel = MotionCode[1];

				CalculateRunningSpec();

				char PullingDecelBuffer[10];
				TxPCLen = sprintf(PullingDecelBuffer,"r35/%.2fe",PullingDecel);
				HAL_UART_Transmit(&huart6,(uint8_t *)PullingDecelBuffer,TxPCLen,100); // Send to uart6 to check the params are set or not
			}
			break;

		case 36: // Resource
			break;

		case 37: // Set Stopping Time
			if (StartRunning)// Setting is not available while running
			{
				InitializeRunning(ExperimentMode);
				break;
			}
			else
			{
				StoppingTime = MotionCode[1];
//				if (StoppingTime <= 2000) // ms
//				{
//					StoppingTime = 2000; // min = 2000 ms
//				}
//				if (StoppingTime >= 10000)
//				{
//					StoppingTime = 10000; // max = 10000 ms
//				}
				char StoppingTimeBuffer[10];
				TxPCLen = sprintf(StoppingTimeBuffer,"r37/%de",StoppingTime);
				HAL_UART_Transmit(&huart6,(uint8_t *)StoppingTimeBuffer,TxPCLen,200); // Send to uart6 to check the params are set or not
				break;
			}
		case 38: // Homing task
			if (StartRunning)// Setting is not available while running
			{
				InitializeRunning(ExperimentMode);
				break;
			}
			else
			{
				IsHoming = true;
				//Timer3CountPeriod = CalculateTimer3Period(MotorDriver,PullingSpeed);
				//Timer3CountPeriod = (int)((float)(120000000.0/((PullingSpeed)*(float)EncoderResolution)) + 0.5); // Set going down speed
				Direction = false; // false = move up, true = move down
				PRIsToggled = true; // false = Dropping Down. change to true/false to change the direction: pulling or dropping
				JoggingMoveUp = true;
				DisableSTOP(); // Disable the stop
				StartPulseGenerating();
			}
			break;

		case 39: // Set Driver type, FDA7000 or ASDA A3
			if (StartRunning)// Setting is not available while running
			{
				InitializeRunning(ExperimentMode);
				break;
			}
			else
			{
				if (MotionCode[1] == 1) // FDA7000
				{
					MotorDriver = true;
					NoOfBytes = 29; // For FDA7000, read 5 register => receive 25 bytes
					EncoderResolution = HigenEncoderResolution;
					InitParams ();
				}
				else // ASDA A3
				{
					MotorDriver = false;
					NoOfBytes = 17;
					EncoderResolution = AsdaEncoderResolution;
					InitParams ();
					// For ASDA Drier, read 1 register => receive 9 bytes
					// read 2 registers => receive 13 bytes
				}
				char DriverTypeBuffer[10];
				TxPCLen = sprintf(DriverTypeBuffer,"g39/%de",MotorDriver);
				HAL_UART_Transmit(&huart6,(uint8_t *)DriverTypeBuffer,TxPCLen,200); // Send to uart6 to check the params are set or not
			}
			break;

		case 41: // Set Fly Acc slope
			if (StartRunning)// Setting is not available while running
			{
				InitializeRunning(ExperimentMode);
			}
			else
			{
				FlyAccSlope = MotionCode[1];

				char FlyAccSlopeBuffer[10];
				TxPCLen = sprintf(FlyAccSlopeBuffer,"r41/%.3fe",FlyAccSlope);
				HAL_UART_Transmit(&huart6,(uint8_t *)FlyAccSlopeBuffer,TxPCLen,100); // Send to uart6 to check the params are set or not
			}

			break;

		case 42: // Set Ki
			if (StartRunning)// Setting is not available while running
			{
				InitializeRunning(ExperimentMode);
			}
			else
			{
				DistCoeff = MotionCode[1];
				CalculateRunningSpec();
				char DistCoeffBuffer[10];
				TxPCLen = sprintf(DistCoeffBuffer,"r42/%.2fe",DistCoeff);
				HAL_UART_Transmit(&huart6,(uint8_t *)DistCoeffBuffer,TxPCLen,100); // Send to uart6 to check the params are set or not
			}
			break;

		case 43: //Set Kd
			if (StartRunning)// Setting is not available while running
			{
				InitializeRunning(ExperimentMode);
			}
			else
			{
				BrakeAccSlope = MotionCode[1];

				char BrakeAccSlopeBuffer[10];
				TxPCLen = sprintf(BrakeAccSlopeBuffer,"r43/%.4fe",BrakeAccSlope);
				HAL_UART_Transmit(&huart6,(uint8_t *)BrakeAccSlopeBuffer,TxPCLen,100); // Send to uart6 to check the params are set or not
			}
			break;

		case 46: // Set origin (home) position
			OriginPulse = MotorEncPulse;
			PositionPulseCmd = 0;
			break;

		case 47: // Set open/Closed loop control Mode
			if (StartRunning)// Setting is not available while running
			{
				InitializeRunning(ExperimentMode);
			}
			else
			{
				if(MotionCode[1] == 0) // Set to Closed loop control
				{
					IsOpenLoopControl = false;
				}
				else // Set to Open-Loop control
				{
					IsOpenLoopControl = true;
				}

				char ControlLoopBuffer[10];
				TxPCLen = sprintf(ControlLoopBuffer,"g47/%de",IsOpenLoopControl);
				HAL_UART_Transmit(&huart6,(uint8_t *)ControlLoopBuffer,TxPCLen,100); // Send to uart6 to check the params are set or not
			}
			break;

		case 48: // turn on/off the software upper limit
			if (StartRunning)// Setting is not available while running
			{
				InitializeRunning(ExperimentMode);
			}
			else
			{
				if(MotionCode[1] == 1) // turn on software limit
				{
					SoftWareLimit = true;
				}
				else // Set to Open-Loop control
				{
					SoftWareLimit = false;
				}

				char SoftWareLimitBuffer[10];
				TxPCLen = sprintf(SoftWareLimitBuffer,"g48/%de",SoftWareLimit);
				HAL_UART_Transmit(&huart6,(uint8_t *)SoftWareLimitBuffer,TxPCLen,100); // Send to uart6 to check the params are set or not
			}
			break;
		default:
			if (StartRunning)// Keep running
			{
				InitializeRunning(ExperimentMode);
				break;
			}
			else
			{
				break;
			}
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
				if (RxPCData != NULL) // remove the null character
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

		// BEGIN UART3
		if (huart->Instance==USART3) // UART3, receive Acc data
		{

			if(RxUart3Data!=EndChar) // read up to the ending char
			{
				if (RxUart3Data != NULL) // remove the null character
				{
					RxUart3Buff[_rxUart3Index]=RxUart3Data;// Copy the data to buffer
				  _rxUart3Index++;
				}
			}
			else //if(RxPCData==EndChar)
			{
				_rxUart3Index=0;
				RxUart3_Cpl_Flag=true; // reading completed
			}
			HAL_UART_Receive_IT(&huart3,&RxUart3Data,1);
		}
//		// END UART3

//		// BEGIN UART4
//		if (huart->Instance==UART4) // UART4, ESP32 to STM
//		{
//			if(RxESPData!=EndChar) // read up to the ending char
//			{
//				if (RxESPData != NULL) // remove the null character
//				{
//					RxESPBuff[_rxESPIndex]=RxESPData;// Copy the data to buffer
//				  _rxESPIndex++;
//				}
//			}
//			else //if(RxPCData==EndChar)
//			{
//				_rxESPIndex=0;
//				RxESP_Cpl_Flag=true; // reading completed
//			}
//			HAL_UART_Receive_IT(&huart4,&RxESPData,1);
//		}
//		// END UART4
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
					if(MotorDriver) // HIGEN Driver
					{
						if ( abs(8*PositionPulseCmd) > abs(TargetPosition)) // 8 is th gear ratio
						{
							IsReachTargetPosition = true;
							return;
						}
					}
					else //ASDA Driver
					{
						if ( abs(PositionPulseCmd) > abs(TargetPosition))
						{
							IsReachTargetPosition = true;
							return;
						}
					}

					if (PRIsToggled)
					{
						HAL_GPIO_TogglePin(PE9_TIM1_CH1_PFIN_GPIO_Port, PE9_TIM1_CH1_PFIN_Pin); // Generate pulses on PF by tonggling this input
						PRIsToggled = false;

						if (StartPulseCmdCounting) // Only counting the pulse cmd while running
						{
							if (Direction) // Direction = true: dropping down
							{
								PositionPulseCmd++; // Increase the pulse cmd
							}
							else // false: pulling up
							{
								PositionPulseCmd--; // Decrease the pulse cmd
							}
						}

//						if (IsStepPulseCmd == true)
//						{
//							PulseCmd++;
//							if (PulseCmd >= abs(StepPulseCmd)) // Pulse cmd is reached
//							{
//								StopPulseGenerating();
//								IsStepPulseCmd = false;
//								PulseCmd = 0;
//								return;
//							}
//						}
						return; // exit the function
					}
					else
					{
						HAL_GPIO_TogglePin(PC8_PR_GPIO_Port, PC8_PR_Pin); // Generate pulses on PF by tonggling this input
						PRIsToggled = true;

						if (StartPulseCmdCounting) // Only counting the pulse cmd while running
						{
							if (Direction) // Direction = true: dropping down
							{
								PositionPulseCmd++; // Increase the pulse cmd
							}
							else // false: pulling up
							{
								PositionPulseCmd--; // Decrease the pulse cmd
							}
						}
//						if (IsStepPulseCmd == true)
//						{
//							PulseCmd++;
//							if (PulseCmd >= abs(StepPulseCmd)) // Pulse cmd is reached
//							{
//								StopPulseGenerating();
//								IsStepPulseCmd = false;
//								PulseCmd = 0;
//								return;
//							}
//						}
						return;
					}
				}
		}
	}

	if (htim->Instance == TIM2) // Timer 2 interrupt, for the main control function, 1ms
		{
				// To calculate PID controller
				Timer2SampleTimeControlCount++;
				if (Timer2SampleTimeControlCount >= SampleTime) // turn on the flag when the sample time reaches, fix the data sample time to 50ms
				{
					Timer2ControlInterrupt = true;
					Timer2SampleTimeControlCount = 0;
				}

				// To transmit the data each 50ms
				Timer2Count++;
				if (Timer2Count >= 50) // turn on the flag when the sample time reaches, fix the data sample time to 40ms
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
  MX_UART5_Init();
  MX_USART6_UART_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(PE15_RELAY1_GPIO_Port, PE15_RELAY1_Pin, GPIO_PIN_SET);
	HAL_Delay(5000);

	InitParams (); // Read the saved params from the flash memory

// Init PID controller
// Assume that FeedForwardPulse is the feedback signal to check the PID calculations
// PID(&TPID, &AccFb, &PIPulseCmd, &DroppingAccel, Kp, StoppingTime, 0, _PID_P_ON_E, _PID_CD_DIRECT);

// PID(&TPID, &AccFb, &PIPulseCmd, &DroppingAccel, Kp, StoppingTime, 0, _PID_P_ON_E, _PID_CD_DIRECT); // Kd = 0, use PI controller
//  PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);
//  PID_SetSampleTime(&TPID, Timer2Period); // the sample time is 50ms = Timer2 time interval
//  PID_SetOutputLimits(&TPID, -2000, 2000); // min PID: -2000rpm, max: 2000rpm

	HAL_GPIO_WritePin(PB15_485_MCU_PC_DIR_GPIO_Port, PB15_485_MCU_PC_DIR_Pin, GPIO_PIN_SET); // Set Uart3 to receive Mode
	HAL_UART_Receive_IT(&huart3,&RxUart3Data,1); // Enable Interrupt

	HAL_TIM_Base_Start_IT(&htim2); // Enable Timer 2 interrupt
// Not turn on timer3 at the start
//	HAL_TIM_Base_Start_IT(&htim3); // Enable Timer 3 interrupt
	HAL_UART_Receive_IT(&huart6,&RxPCData,1);
	//HAL_UART_Receive_IT(&huart4,&RxESPData,1);
	DriverInit();
	ReadMultiRegister(StE03,5);
	//ReadDriverData(StE07); // Read speed

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
			{
				switch (ExperimentMode)
				{
					case 1: // Dropping Mode
						if (Dropping()) // Dropping() return true when it finishing
						{
//							POSReach = HAL_GPIO_ReadPin(CN1_47_INSPD_INPOS_GPIO_Port,CN1_47_INSPD_INPOS_Pin);	// Check if the position is reached or not
							if (!POSReach) // Check if position is reached or not
							{
								if (WaitingMiliSecond(3000)) // Wait for 2 Seconds
								{
									StopExperiment();
									if (RunningMode) // Running Mode = false = manual, true=Automatic
									{
										memset (TxPCBuff, '\0', sizeof (TxPCBuff)); // reset
										TxPCLen = sprintf(TxPCBuff,"$"); // $ means finish running one episode
										HAL_UART_Transmit(&huart6,(uint8_t *)TxPCBuff,TxPCLen,200); // use uart6 to send
									}
								}
							}
						}
						break;
					case 2: // Pulling Mode

						if (PullingExperiment()) // PullingExperiment() return true when it finishing
						{
//							POSReach = HAL_GPIO_ReadPin(CN1_47_INSPD_INPOS_GPIO_Port,CN1_47_INSPD_INPOS_Pin);	// Check if the position is reached or not
							if (!POSReach) // Check if position is reached or not
							{
								if (WaitingMiliSecond(3000)) // Wait for 2 Seconds
								{
									StopExperiment();
									if (RunningMode) // Running Mode = false = manual, true=Automatic
									{
										memset (TxPCBuff, '\0', sizeof (TxPCBuff)); // reset
										TxPCLen = sprintf(TxPCBuff,"$"); // $ means finish running one episode
										HAL_UART_Transmit(&huart6,(uint8_t *)TxPCBuff,TxPCLen,200); // use uart6 to send
									}
								}
							}
						}
						break;

					case 3: // Pulling -> Dropping Mode
						if (PullAndDrop()) // PullingExperiment() return true when it finishing
						{
//							POSReach = HAL_GPIO_ReadPin(CN1_47_INSPD_INPOS_GPIO_Port,CN1_47_INSPD_INPOS_Pin);	// Check if the position is reached or not
							if (!POSReach) // Check if position is reached or not
							{
								if (WaitingMiliSecond(3000)) // Wait for 3 Seconds
								{
									StopExperiment();
									if (RunningMode) // Running Mode = false = manual, true=Automatic
									{
										memset (TxPCBuff, '\0', sizeof (TxPCBuff)); // reset
										TxPCLen = sprintf(TxPCBuff,"$"); // $ means finish running one episode
										HAL_UART_Transmit(&huart6,(uint8_t *)TxPCBuff,TxPCLen,200); // use uart6 to send
									}
								}
							}
						}
						break;
					default:
						break;
				}
			}
		}
			// END Running Experiment
		// END Timer2ControlInterrupt

		// BEGIN Timer2 interrupt for sending the data
		if (Timer2SampleTimeInterrupt)
		{
			Timer2SampleTimeInterrupt = false;

			// Read CN1-22-RDY, Check if Servo Ready or not, or Servo ON of OFF.
			// If servo is not ready or not ON, then turn off all the functions
			if ( HAL_GPIO_ReadPin(CN1_22_RDY_GPIO_Port,CN1_22_RDY_Pin) || HAL_GPIO_ReadPin(CN1_48_BRAKE_GPIO_Port,CN1_48_BRAKE_Pin))
			{
				// If the servo is not ON. Then stop all the running function
				StopPulseGenerating();
				StopExperiment();
				ResetPIDController();
				IsHoming = false; // Disable Homming
			}

			// BEGIN Software Limit ASDA Driver
			if (!MotorDriver) // Applied for ASDA-A3 Diver since the encoder pulse only can be cleared when cycle the driver
			{
				if (StartRunning && StartAccleratePulling) // Always turn on the software limit while running
				{
					if (MotorEncPulse-OriginPulse <= 200) // Software Limit Switch based on actual motor position, 500/2048 pulses
					{
						StopPulseGenerating();
						StopExperiment();
						ResetPIDController();
					}
				}
				if (SoftWareLimit) // Software limit is on
				{
					if (MotorEncPulse-OriginPulse <= 200) // Software Limit Switch based on actual motor position, 500/2048 pulses
					{
						StopPulseGenerating();
						StopExperiment();
						ResetPIDController();
					}
				}
			}
			else // HIGEN FDA Driver, Software Limit Switch
			{
				if (StartRunning && StartAccleratePulling)// Always turn on the software limit while running
				{
					if (MotorEncPulse-OriginPulse <= 1000) // Software Limit Switch based on actual motor position, 500/2048 pulses
					{
						StopPulseGenerating();
						StopExperiment();
						ResetPIDController();
					}
				}
				if (SoftWareLimit) // Software limit is on
				{
					if (MotorEncPulse-OriginPulse <= 1000) // Software Limit Switch based on actual motor position, 500/2048 pulses
					{
						StopPulseGenerating();
						StopExperiment();
						ResetPIDController();
					}
				}
			}
			// END Software Limit ASDA Driver
			//BEGIN Homing
			if (IsHoming)
			{
				if(CheckGoingToRefPosition(false, 100)) // false = go up, 0 = home position
				{
					IsHoming = false; // finish homing
				}
			}
			//END Homing

			// BEGIN Send data to the UI
			if (UIDataRequest)
			{
				if (PositionControlMode) // Position Mode, read both Position and Speed, Send both Position and Speed
				{
					memset (TxPCBuff, '\0', sizeof (TxPCBuff)); // reset
					if (MotorDriver) // FDA7000 Driver
					{

						//TxPCLen = sprintf(TxPCBuff,"s%.1f/%.1f/%d/%de",MotorSpeed,SpeedCmd,MotorEncPulse,PositionPulseCmd*EgearRatio); // s means speed
						//TxPCLen = sprintf(TxPCBuff,"s%.1f/%.1f/%d/%de",MotorSpeed,SpeedCmd,MotorEncPulse,PulseError);
						//TxPCLen = sprintf(TxPCBuff,"s%.1f/%.1f/%.1f/%.1f/%.1fe",MotorSpeed,SpeedCmd,ObjectPosition,AccZ,AccRef);
						TxPCLen = sprintf(TxPCBuff,"s%.1f/%.1f/%.1f/%.1fe",MotorSpeed,SpeedCmd,ObjectPosition,AccRef);
						//TxPCLen = sprintf(TxPCBuff,"s%.1f/%.1f/%de",MotorSpeed,SpeedCmd,PositionPulseCmd*EgearRatio); // 8 is the Egear ratio
						//TxPCLen = sprintf(TxPCBuff,"s2/%de",PulseCmd);
						HAL_UART_Transmit(&huart6,(uint8_t *)TxPCBuff,TxPCLen,200); // use uart6 to send
						ReadMultiRegister(StE03,6); // Read from StE03 -> StE08
					}
					else // ASDA-A3 Driver
					{
						//TxPCLen = sprintf(TxPCBuff,"s%.1f/%.1f/%d/%d/%.1f/%.1fe",MotorSpeed,SpeedCmd,MotorEncPulse,PulseError,AccZ,AccRef); // s means speed
						//TxPCLen = sprintf(TxPCBuff,"s%.1f/%.1f/%.1f/%.1f/%.1fe",MotorSpeed,SpeedCmd,ObjectPosition,AccZ,AccRef);
						TxPCLen = sprintf(TxPCBuff,"s%.1f/%.1f/%.1f/%.1fe",MotorSpeed,SpeedCmd,ObjectPosition,AccRef);
						//TxPCLen = sprintf(TxPCBuff,"s%.1f/%.1f/%de",MotorSpeed,SpeedCmd,PositionPulseCmd*EgearRatio); // 8 is the Egear ratio
						//TxPCLen = sprintf(TxPCBuff,"s2/%de",PulseCmd);
						HAL_UART_Transmit(&huart6,(uint8_t *)TxPCBuff,TxPCLen,200); // use uart6 to send
						// Read 4 words start from 0x012 to 0x015
						// Encoder pulse: 0x012 + 0x013 (2 words)
						// Motor Speed: 0x014 + 0x015 (2 words)
						// Pulse Error: (2 word)
						// Total is 6 words, 1word = 4 bytes
						ReadMultiRegister(ASDA_MotorSpeed,6);
					}
				}
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
		// END 50ms period interrupt

		// BEGIN Uart3 receive complete
		if (RxUart3_Cpl_Flag)
		{
			RxUart3_Cpl_Flag = false;


//			AccZDataRegion[0] = AccZDataRegion[3];
//			AccZDataRegion[1] = AccZDataRegion[2];
//			AccZDataRegion[2] = AccZDataRegion[1];
//			AccZDataRegion[3] = AccZDataRegion[0];

//			AccZDataRegion[0] = RxUart3Buff[3];
//			AccZDataRegion[1] = RxUart3Buff[2];
//			AccZDataRegion[2] = RxUart3Buff[1];
//			AccZDataRegion[3] = RxUart3Buff[0];
//
//			memcpy(&AccZ, AccZDataRegion, 4);



//			memset (RxUart3Buff, '\0', sizeof (RxUart3Buff)); // reset buffer
//			if (atof((char *)RxUart3Buff) != 0)
//			{
//				AccZ = atof((char *)RxUart3Buff);
//			}
//			else
//			{
//				AccZ = AccZ + 10;
//			}

			memset (AccZDataRegion, '\0', sizeof (AccZDataRegion)); // reset buffer

			for (uint8_t i = 0; i<= sizeof(RxUart3Buff); i++) // Remove the null byte
			{
				if (RxUart3Buff[i] != 0x00)
				{
					AccZDataRegion[i] = RxUart3Buff[i];
				}
			}
			if (fabs(atof((char *)RxUart3Buff)) <= 100)
			{
				AccZ = atof((char *)RxUart3Buff);
			}


//			memset (RxUart3Buff, '\0', sizeof (RxUart3Buff)); // reset buffer

		}
		// END uart 3
//	 BEGIN UART4 (ESP32) Intterupt complete
//		if(RxESP_Cpl_Flag)
//		{
//			RxESP_Cpl_Flag = false;
//			AccZ = atof((char *)RxESPBuff);
//			memset (RxESPBuff, '\0', sizeof (RxESPBuff)); // reset buffer
//			//HAL_UART_Receive_IT(&huart4,&RxESPData,1);
//		}
// 	 End UART4 (ESP32) process

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


//								MotorSpeedBuff = (float)((RxDriverBuff[5+i] << 24) | (RxDriverBuff[6+i] << 16) | (RxDriverBuff[3+i] << 8) | RxDriverBuff[4+i])/((float)-10.0); // Minus to Reverse
//
//								if (fabs(MotorSpeedBuff) <= 2000)
//								{
//									MotorSpeed = MotorSpeedBuff;
//								}


								MotorEncPulse = (RxDriverBuff[19+i] << 24) | (RxDriverBuff[20+i] << 16) | (RxDriverBuff[21+i] << 8) | RxDriverBuff[22+i];

								//PulseError = (RxDriverBuff[23+i] << 24) | (RxDriverBuff[24+i] << 16) | (RxDriverBuff[25+i] << 8) | RxDriverBuff[26+i];
							}
							else // ASDA-A3
							{
								MotorSpeedBuff = (float)((RxDriverBuff[5+i] << 24) | (RxDriverBuff[6+i] << 16) | (RxDriverBuff[3+i] << 8) | RxDriverBuff[4+i])/((float)-10.0); // Minus to Reverse

								if (fabs(MotorSpeedBuff) <= 2000)
								{
									MotorSpeed = MotorSpeedBuff;
								}

								MotorEncPulse = -((RxDriverBuff[9+i] << 24) | (RxDriverBuff[10+i] << 16) | (RxDriverBuff[7+i] << 8) | RxDriverBuff[8+i]); // Minus to Reverse

								//PulseError = ((RxDriverBuff[13+i] << 24) | (RxDriverBuff[14+i] << 16) | (RxDriverBuff[11+i] << 8) | RxDriverBuff[12+i]);

								//ObjectPosition = 2*3.14*DrumRadius*(MotorEncPulse-OriginPulse)/AsdaEncoderResolution; // Calculate Object Position in m
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
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* UART5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UART5_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(UART5_IRQn);
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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

