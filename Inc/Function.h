#include "stdint.h"
#include "stdbool.h"
#include "main.h"

// Constant parameter
#define DriverID 1
#define P402_SpeedCommand 401 // Register address of the parameter P402
#define StE03 12 // Register for motor speed in rpm
#define StE07 16 // Encoder/Feedback pulse register
#define Timer2Period 1 // ms, timer2 period
#define DataSampleTime 50 // ms

// Address to save the parameters Sector5
#define MemoryAddress 0x0800C100

void Stop(); // Stop motor function
void AlarmReset(); // reset alarm function
void Estop(); // Estop function
void JogMoveUp();
void JogMoveDown();
void SetPositionMode();
void SetSpeedMode();
bool DriverInit(bool SpeedControl,bool DirCW);