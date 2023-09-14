#include "stdint.h"
#include "stdbool.h"
#include "main.h"

// Constant parameter
#define DriverID 1
#define TachometerID 2
#define P402_SpeedCommand 401 // Register address of the parameter P402
#define StE03 12 // Register for motor speed in rpm
#define StE07 16 // Encoder/Feedback pulse register
#define StE08 17 // Error pulse

#define ASDA_EncoderRegister 20 // 0x014h = 20, ASDA Driver's Status Mornitoring Register 2 - Encoder Pulses
#define ASDA_MotorSpeed 18 // 0x012h = 18, ASDA Driver's Status Mornitoring Register 1 - Motor Speed
#define CounterA 1 // Counter A high word register

#define Timer2Period 1 // ms, timer2 period
#define Timer3Period 15 // us, timer3 period
#define DataSampleTime 50 // ms
#define HigenEncoderResolution 8192
#define AsdaEncoderResolution 1024

#define RampingGoingSpdTime 3 // seconds


#define DroppingMode 1
#define PullingMode 2
#define PullAndDropMode 3

// Address to save the parameters Sector5
//#define MemoryAddress 0x0800C100 // Sector5

#define MemoryAddress 0x08040000 //

void Stop(); // Stop motor function
void AlarmReset(); // reset alarm function
void Estop(); // Estop function
void JogMoveUp();
void JogMoveDown();
void DisableSTOP();
void SetPositionMode();
void SetSpeedMode();
void PulseClear();
bool DriverInit();
uint16_t ReadLogicF7000Out(void);
