/*
 Name:		XeTuDong.ino
 Created:	1/7/2018 9:24:40 AM
 Author:	Than
*/

#include <EEPROM.h>
#include "MultiThread.h"
#include "homephone.h"
#include "Omni.h"
#include "FloorSensorsMaster.h"
#include <Servo.h>

#define RED_TEAM		0
#define BLUE_TEAM		1

#define M1_DIR		PQ_1
#define M2_DIR		PP_3
#define M3_DIR		PA_7

#define M1_PWM		PM_6
#define M2_PWM		PM_2
#define M3_PWM		PB_3

#define CHA_ENCODER_SWITCH	PN_3
#define CHB_ENCODER_SWITCH	PP_2

#define RELAY_SHOOT	PM_5
#define RELAY_TZ1	PM_4
#define RELAY_TZ2	PA_6
#define RELAY_TZ3	PD_7

#define SHOOT_TZ1	0
#define SHOOT_TZ2	1
#define SHOOT_TZ3	2

#define LOCATION_START	0
#define THROWING_ZONE1	1
#define THROWING_ZONE3	2
#define THROWING_ZONE2	3

#define TIME_SHOOT		900
#define TIME_SHOOT_TZ1	400
#define TIME_LOCK		300
#define TIME_READY		800

#define LCD_SDIN	PG_0
#define LCD_SLK		PF_2
#define LCD_A		PF_3
#define LCD_RESET	PF_1
#define LCD_CS		PL_4

#define BUTTON_1	PH_2
#define BUTTON_2	PH_3

#define CBTC	PE_3

#define PIN_GRIP	PE_4

#define GRIP_OPEN	98
#define GRIP_CLOSE	177

#define UART_MPU		Serial6
#define UART_BLUETOOTH	Serial3
#define UART_SENSOR		Serial7
#define UART_CAMERA		Serial5
#define DEBUG			Serial

#define KP_ANGLE			3
#define MAX_ERROR_ANGLE		14

#define TURNCAMERA		1
#define	TURNMPU			0

#define KP_ANGLE_CAMERA		0.5
#define MAX_ANGLE_CAMERA	14

#define CAMERA_ERROR	5
#define CAMERA_OFFSET	5

#define CAMERA_OFFSET_L	 
#define CAMERA_OFFSET_R	 5

#define ACCEL_START		200

#define ADDRESS_EEPROM	21

Omni OMNI3(M1_DIR, M1_PWM, M2_DIR, M2_PWM, M3_DIR, M3_PWM);
homephone LCD(LCD_SDIN, LCD_SLK, LCD_A, LCD_RESET, LCD_CS);

MultiThread LCDScheduler;

MultiThread GetSensorScheduler;

MultiThread SetTurnScheduler;

MultiThread UploadOmniScheduler;

MultiThread ShootScheduler;

MultiThread LockScheduler;

MultiThread DelayMs;

MultiThread SerScheduler;

MultiThread CBTCScheduler;

Servo Grip;

byte Team = RED_TEAM;

float Angle_Mpu = 0;
float Angle_Robot = 0;
float Turn = 0;

float Moving_Angle = 0;
float Present_Speed = 0;
float Speed = 0;

byte TurnMode = TURNMPU;

float SpeedSet = 60;

int Accel = ACCEL_START;

bool IsReady = false;
bool IsWaiting = true;
int CounterShoot = 0;

bool IsShoot = false;
bool IsLock = false;

uint16_t Time_Shoot = TIME_SHOOT;
uint16_t Time_Lock = TIME_LOCK;
int ShootMode = SHOOT_TZ3;

bool IsMovingTz = false;
bool IsRunShoot = false;
int Current_Position = LOCATION_START;

int Run_Order = 0;
int subRun_Order = 0;

//Floor Sensor/////////////////
bool IsFront_Right = false;
bool IsFront_Center = false;
bool IsFront_Left = false;

bool IsSide_Back = false;
bool IsSide_Midle = false;
bool IsSide_Front = false;

float BackOffset = 0;
float RightOffset = 0;

uint8_t BackAnalog[7];
uint8_t RightAnalog[7];

uint8_t DesiredLineColor = RED;

bool IsBackLine = false;
bool IsRightLine = false;

//Camera////////////////////////////////////////////////////////////////
String InputString =		"";
int RedTreePosition =		0;
int YellowTreePosition =	0;
int BlueTreePosition =		0;

byte CountRightLine =	0;
byte CountBackLine =	0;

byte CountFR =	0;
byte CountFC =	0;
byte CountFL =	0;

byte CountSB =	0;
byte CountSM =	0;
byte CountSF =	0;

byte CountCBTC = 0;
bool IsCBTC = false;

int AngleServo = GRIP_OPEN;
int LastAngleServo = 0;

void setup() {
	Init();
}

void loop() {
	////////////////
	LCDDisplay();

	EventsButton();

	EventsCBTC();

	GetValueCamera();

	GetSensor();
	
	ReadMPU();
	
	MovingToTZ_ONE();
	RunShoot_ONE();
	MovingToTZ_TWO();
	MovingToTZ_THREE();
	
	Lock();
	Shoot();
	
	SetTurn();
	UploadOmni();
	GripServo();

}

void Init() {
	//UART_BLUETOOTH.begin(115200);
	UART_MPU.begin(115200);
	UART_CAMERA.begin(9600);

	UART_SENSOR.begin(115200);
	FloorSensors.init(&UART_SENSOR);

	DEBUG.begin(115200);
	DEBUG.println("---EVA AUTO ROBOT---");

	LCDInit();

	Grip.attach(PIN_GRIP);
	//Grip.write(GRIP_OPEN);

	pinMode(PUSH1, INPUT_PULLUP);
	pinMode(PUSH2, INPUT_PULLUP);
	pinMode(BUTTON_1, INPUT_PULLUP);
	pinMode(BUTTON_2, INPUT_PULLUP);

	pinMode(CBTC, INPUT);

	RelayInit();
	OMNI3.Init();
	OMNI3.Stop();
}

//LCD DISPLAY///////////////////////////////////////////////////////////////////////////////////////////////////////
void LCDInit() {
	LCD.begin();
	LCD.setContrast(0x10);
	LCD.clear();
	
	LCD.ascString(34, 1, "___EVA___", 1);
	LCD.ascString(1, 12, "Angle_R:", 1);
	LCD.ascString(1, 22, "Speed:", 1);
	LCD.ascString(1, 32, "Moving_Angle:", 1);
	LCD.ascString(1, 42, "Accel:", 1);
	
	LCD.ascString(60, 52, "Yellow:", 1);
	LCD.display();
}
void LCDDisplay()
{
	RUN_EVERY(LCDScheduler, 100);
	
	if (Team == RED_TEAM)
	{
		LCD.ascString(1, 1, "RED-", 1);
	}
	else if (Team == BLUE_TEAM)
	{
		LCD.ascString(1, 1, "BLUE-", 1);
	}

	if (CounterShoot == 0)
	{
		LCD.ascString(105, 1, "TZ1", 1);
	}
	else
	{
		LCD.ascString(105, 1, "TZ2", 1);
	}

	LCD.setCursor(56, 12);
	LCD.print(Angle_Robot);
	LCD.setCursor(90, 12);
	LCD.print(Angle_Mpu);

	LCD.setCursor(56, 22);
	LCD.print((int)SpeedSet);
	LCD.setCursor(90, 22);
	LCD.print(Present_Speed);

	LCD.setCursor(80, 32);
	LCD.print((int)Moving_Angle);

	LCD.setCursor(40, 42);
	LCD.print(Accel);

	if (Team == RED_TEAM)
	{
		LCD.ascString(1, 52, "Red__:", 1);
	}
	else if (Team == BLUE_TEAM)
	{
		LCD.ascString(1, 52, "Blue_:", 1);
	}

	LCD.setCursor(38, 52);
	//LCD.print(RedTreePosition);
	LCD.print((Team == RED_TEAM) ? RedTreePosition : BlueTreePosition);
	LCD.setCursor(104, 52);
	LCD.print(YellowTreePosition);

	LCD.setTextColor(black, white);
	LCD.display();
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void EventsButton() {

	if (digitalRead(PUSH1) == LOW)
	{
		while (digitalRead(PUSH1) == LOW)
			delay(10);
		IsReady = true;
		AngleServo = GRIP_CLOSE;
	}

	if (digitalRead(PUSH2) == LOW)
	{
		while (digitalRead(PUSH2) == LOW)
			delay(10);

		if (Current_Position == THROWING_ZONE1 || Current_Position == THROWING_ZONE2)
		{
			IsReady = true;
			AngleServo = GRIP_CLOSE;
		}

		IsMovingTz = true;
	}


	if (digitalRead(BUTTON_1) == LOW)
	{
		while (digitalRead(BUTTON_1) == LOW)
			delay(10);
		// Chon vung san bang
		if (Current_Position == LOCATION_START)
		{
			if (Team == RED_TEAM)
			{
				Team = BLUE_TEAM;
			}
			else
			{
				Team = RED_TEAM;
			}
		}	
	}

	if (digitalRead(BUTTON_2) == LOW)
	{
		while (digitalRead(BUTTON_2) == LOW)
			delay(10);
		// Chon vung ban TZ1 or TZ2
		if (Current_Position == LOCATION_START)
		{
			if (CounterShoot == 1)
			{
				CounterShoot = 0;
			}
			else
			{
				CounterShoot = 1;
			}
		}
	}
}
// Xac Nhan Da Nhan Con///////////////////////////////////////////////////
void EventsCBTC() {
	if (IsWaiting && !IsReady)
	{
		if (IsCBTC)
		{
			if (CBTCScheduler.isSchedule(TIME_READY))
			{
				if (CounterShoot == 0)
				{
					IsReady = true;
					AngleServo = GRIP_CLOSE;
				}
				else
				{
					IsReady = true;
					IsMovingTz = true;
					AngleServo = GRIP_CLOSE;
				}
			}
		}
		else
		{
			CBTCScheduler.Counter = millis();
		}
	}
}

//Get Camera ///////////////////////////////////////////////////////////////////////////////////////////////////
void GetValueCamera() {
	while (UART_CAMERA.available())
	{
		char c = (char)UART_CAMERA.read();
		if (c == '\n')
		{
			int tabOrder = InputString.indexOf('\t');
			int tabOrder1 = InputString.lastIndexOf('\t');

			YellowTreePosition = InputString.substring(0, tabOrder).toInt();
			RedTreePosition = InputString.substring(tabOrder + 1, tabOrder1).toInt();
			BlueTreePosition = InputString.substring(tabOrder1 + 1).toInt();

			InputString = "";

			//DEBUG.print(YellowTreePosition);
			//DEBUG.print("\t");
			//DEBUG.print(RedTreePosition);
			//DEBUG.println("");
		}
		else
		{
			InputString += c;
			//DEBUG.println(InputString);
		}
	}

}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Get Sensor///////////////////////////////////////////////////////////////////////////////////////////////////////
void GetSensor() {
	FloorSensors.waitForSlave();
	RUN_EVERY(GetSensorScheduler, 5);
	//FloorSensors.run();
	FloorSensors.setColor(DesiredLineColor);
	//FloorSensors.setColor(WHITE);
	FloorSensors.request();

	//FR
	if (FloorSensors.getProximityState(FR))
	{
		CountFR++;
		if (CountFR >= 3) {
			IsFront_Right = true;
		}
	}
	else
	{
		IsFront_Right = 0;
		CountFR = false;
	}
	//FC
	if (FloorSensors.getProximityState(FC))
	{
		CountFC++;
		if (CountFC > 3) {
			IsFront_Center = true;
		}
	}
	else
	{
		IsFront_Center = 0;
		CountFC = false;
	}
	//FL
	if (FloorSensors.getProximityState(FL))
	{
		CountFL++;
		if (CountFL >= 2) {
			IsFront_Left = true;
		}
	}
	else
	{
		IsFront_Left = 0;
		CountFL = false;
	}

	//SB
	if (FloorSensors.getProximityState(SB))
	{
		CountSB++;
		if (CountSB > 3) {
			IsSide_Back = true;
		}
	}
	else
	{
		IsSide_Back = 0;
		CountSB = false;
	}
	//SM
	if (FloorSensors.getProximityState(SM))
	{
		CountSM++;
		if (CountSM > 3) {
			IsSide_Midle = true;
		}
	}
	else
	{
		IsSide_Midle = 0;
		CountSM = false;
	}
	//SF
	if (FloorSensors.getProximityState(SF))
	{
		CountSF++;
		if (CountSF >= 3) {
			IsSide_Front = true;
		}
	}
	else
	{
		IsSide_Front = 0;
		CountSF = false;
	}

	//DEBUG.print(IsFront_Right);
	//DEBUG.print(IsFront_Center);
	//DEBUG.print(IsFront_Left);
	//DEBUG.print("\t");
	//DEBUG.print(IsSide_Back);
	//DEBUG.print(IsSide_Midle);
	//DEBUG.print(IsSide_Front);
	//DEBUG.println("*");

	RightOffset = FloorSensors.getSideOffset();
	BackOffset = FloorSensors.getBackOffset();
	//DEBUG.print(BackOffset); DEBUG.print("\t");
	//DEBUG.print(BackOffset); DEBUG.println(" ");

	for (uint8_t _i = 0; _i < 7; _i++)
	{
		BackAnalog[_i] = FloorSensors.getBackAnalogState(_i);
		RightAnalog[_i] = FloorSensors.getSideAnalogState(_i);
	}
	/*DEBUG.print("BACK: ");
	for(uint8_t i = 0 ; i < 7; i++)
		DEBUG.print(BackAnalog[i]);

	DEBUG.print("\nRIGHT: ");
	for (uint8_t i = 0; i < 7; i++)
		DEBUG.print(RightAnalog[i]);

	DEBUG.println("**************");*/
	FindBack();
	FindRight();

	GetCBTC();
	//DEBUG.println(IsCBTC);
}
void GetCBTC() {
	if (!digitalRead(CBTC))
	{
		CountCBTC++;
		if (CountCBTC >= 10) {
			IsCBTC = true;
		}
	}
	else
	{
		IsCBTC = 0;
		CountCBTC = false;
	}
}
//Find Back Line///////////////////////////////////////////////////////
bool FindBackLine(uint8_t _color) {
	DesiredLineColor = _color;
	return IsBackLine;
}
void FindBack() {
	uint8_t _K = 0;

	for (uint8_t _i = 0; _i < 7; _i++)
	{
		if (BackAnalog[_i] == 1)
		{
			_K++;
		}
	}

	if (_K >= 5)
	{
		CountBackLine++;
	}
	else
	{
		CountBackLine = 0;
	}

	if (CountBackLine > 3)
	{
		IsBackLine = true;
	}
	else
	{
		IsBackLine = false;
	}
}
//Find Right Line///////////////////////////////////////////////////////
bool FindRightLine(uint8_t _color) {
	DesiredLineColor = _color;
	return IsRightLine;
}
void FindRight() {
	uint8_t _K = 0;

	for (uint8_t _i = 0; _i < 7; _i++)
	{
		if (RightAnalog[_i] == 1)
		{
			_K++;
		}
	}
	if (_K >= 5)
	{
		CountRightLine++;
	}
	else
	{
		CountRightLine = 0;
	}

	if (CountRightLine > 2)
	{
		IsRightLine = true;
	}
	else
	{
		IsRightLine = false;
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Servo Grip///////////////////////////////////////////////////////////////////////////////////////////////////////
void GripServo() {

	if (AngleServo != LastAngleServo)
	{
		Grip.write(AngleServo);
		LastAngleServo = AngleServo;
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//MPU //////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ReadMPU() {
	if (UART_MPU.available() <= 1) return;

	char data[2];
	UART_MPU.readBytes(data, 2);

	if (data[0] < 100)
	{
		Angle_Mpu = ((float)data[1] * 100 + (float)data[0]) / 100;
	}
	else
	{
		Angle_Mpu = -((float)data[1] * 100 + (256 - (float)data[0])) / 100;
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SetTurn() {
	RUN_EVERY(SetTurnScheduler, 2);
	float error = 0;

	if (TurnMode == TURNMPU)
	{
		error = Angle_Mpu - Angle_Robot;
		error = constrain(error, -MAX_ERROR_ANGLE, MAX_ERROR_ANGLE);
		Turn = error * KP_ANGLE;
	}
	else if (TurnMode == TURNCAMERA)
	{
		Angle_Robot = Angle_Mpu;

		if (ShootMode == SHOOT_TZ3)
		{			
			if (YellowTreePosition != 999) error = -YellowTreePosition; else error = 0;		
		}
		else
		{
			if (Team == RED_TEAM)
			{
				if (RedTreePosition != 999) error = -RedTreePosition; else error = 0;
			}
			else if(Team == BLUE_TEAM)
			{
				if (BlueTreePosition != 999) error = -BlueTreePosition; else error = 0;
			}
		}

		Turn = error * KP_ANGLE_CAMERA;

		if (ShootMode == SHOOT_TZ3)
		{
			if (YellowTreePosition <= -CAMERA_OFFSET || YellowTreePosition >= CAMERA_OFFSET_R )
			{
				if (Turn >= 0)
				{
					if (Turn < 7) Turn = 7;
				}
				else
				{
					if (Turn > -7) Turn = -7;
				}
			}
		}
		else
		{
			if (Team == RED_TEAM)
			{
				if (RedTreePosition <= -(CAMERA_OFFSET - 2) || RedTreePosition >= (CAMERA_OFFSET - 2))
				{
					if (Turn >= 0)
					{
						if (Turn < 7) Turn = 7;
					}
					else
					{
						if (Turn > -7) Turn = -7;
					}
				}
			}
			else if (Team == BLUE_TEAM)
			{
				if (BlueTreePosition <= -(CAMERA_OFFSET - 2) || BlueTreePosition >= (CAMERA_OFFSET - 2))
				{
					if (Turn >= 0)
					{
						if (Turn < 7) Turn = 7;
					}
					else
					{
						if (Turn > -7) Turn = -7;
					}
				}
			}
		}

		Turn = constrain(Turn, -MAX_ANGLE_CAMERA, MAX_ANGLE_CAMERA);
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Moving and shoot///////////////////////////////////////////////////////////////////////////////////////////////
void MovingToTZ_ONE() {
	if (Current_Position != LOCATION_START) return;
	if (!IsMovingTz) return;

	switch (Run_Order)
	{
	case 0:Moving_Angle = 25; Speed = 110; Angle_Robot = 0; Accel = 300;

		DelayMs.Counter = millis();
		IsWaiting = false;
		Run_Order++;
		break;

	case 1:Moving_Angle = 25; Speed = 110; Angle_Robot = 0; Accel = 300;

		if(DelayMs.isSchedule(1600))
		{
			Run_Order++;
		}		
		break;

	case 2:Moving_Angle = 25; Speed = 110; Angle_Robot = 0; Accel = 300;
		
		if (IsFront_Left && IsFront_Right)
		{
			Run_Order++;
		}
		break;

	case 3:Moving_Angle = 25; Speed = 70; Angle_Robot = 0; Accel = 60;

		if (IsFront_Center)
		{
			Run_Order++;
		}
		break;

	case 4:Moving_Angle = 0; Speed = 80; Angle_Robot = 0; Accel = 300;

		if (!IsFront_Right && IsFront_Left)
		{
			Run_Order++;
		}
		else if (FindRightLine(WHITE))
		{
			Run_Order = 6;
		}
		break;



	case 5:Moving_Angle = 0; Speed = 30; Angle_Robot = 0; Accel = 50;

		if (FindRightLine(WHITE))
		{
			Run_Order++;
		}
		break;

	case 6:Moving_Angle = 100; Speed = 50; Angle_Robot = 0; Accel = 200;

		//if (FindBackLine((Team == RED_TEAM) ? RED : BLUE))
		//{
		//	Run_Order++;
		//}
		//else
		if (IsSide_Back)
		{
			Run_Order++;
		}
		break;

	case 7:Moving_Angle = 180; Speed = 30; Angle_Robot = 0; Accel = 200;

		if (IsSide_Midle)
		{
			Run_Order = 9;
		}
		break;

	case 8:Moving_Angle = -90; Speed = 20; Angle_Robot = 0; Accel = 200;

		if (FindBackLine((Team == RED_TEAM) ? RED : BLUE))
		{
			Run_Order++;
		}
		break;

	case 9:Moving_Angle = 180; Speed = 0; Angle_Robot = 0; Accel = 1000;

		IsMovingTz = false;
		IsWaiting = true;
		IsReady = false;
		Current_Position = THROWING_ZONE1;
		Run_Order = 0;		
		break;

	default:
		break;
	}
}

void RunShoot_ONE() {
	if (Current_Position != THROWING_ZONE1) return;
	if (!IsReady) return;
	if (IsMovingTz) return;

	switch (Run_Order)
	{
	case 0: 
		
		DelayMs.Counter = millis();
		Run_Order++;
		IsWaiting = false;
		break;

	case 1:
		
		ShootMode = SHOOT_TZ1;
		if (DelayMs.isSchedule(200))
		{
			Run_Order++;
		}
		break;
	
	case 2:Moving_Angle = 0; Speed = 60; Angle_Robot += 0.005; Accel = 300;
		
		if (Angle_Robot > 82)
		{
			Angle_Robot = 82;
		}
		if (Angle_Mpu > 80)
		{
			Run_Order++;
		}
		break;

	case 3: Moving_Angle = -65; Speed = 70; Angle_Robot = 80.5; Accel = 300;
		
		DelayMs.Counter = millis();
		Run_Order++;
		break;

	case 4: Moving_Angle = -65; Speed = 70; Angle_Robot = 80.5; Accel = 300;

		if (DelayMs.isSchedule(830))
		{
			Run_Order=6;
		}
		break;

	case 5:Speed = 0; Accel = 1000;
		
		TurnMode = TURNCAMERA;
		if (Team == RED_TEAM)
		{
			if (RedTreePosition >= -CAMERA_OFFSET && RedTreePosition <= CAMERA_OFFSET)
			{
				Run_Order++;
			}
		}
		else if (Team == BLUE_TEAM)
		{
			if (BlueTreePosition >= -CAMERA_OFFSET && BlueTreePosition <= CAMERA_OFFSET)
			{
				Run_Order++;
			}
		}
		break;

	case 6:Moving_Angle = -65; Speed = 0; Angle_Robot = 80.5; Accel = 300;
		
		DelayMs.Counter = millis();		
		Run_Order++;
		break;

	case 7:
		 
		if (DelayMs.isSchedule(400))
		{
			Run_Order++;
		}
		break;

	case 8:
		
		IsShoot = true;
		TurnMode = TURNMPU;
		Run_Order++;
		CounterShoot++;
		break;

	case 9:
		
		AngleServo = GRIP_OPEN;
		if (!IsShoot) ReturnWaitOne();
		if (IsWaiting)
		{ 
			Run_Order = 0;
		}
		break;

	default:
		break;
	}
}

void ReturnWaitOne() {
	switch (subRun_Order)
	{

	case 0: Moving_Angle = 180; Speed = 60; Angle_Robot -= 0.005; Accel = 200;

		if (Angle_Mpu < 7)
		{
			subRun_Order++;
		}
		break;

	case 1: Moving_Angle = -180; Speed = 80; Angle_Robot = 0; Accel = 300;

		if (IsSide_Front || IsSide_Back)
		{
			subRun_Order++;
		}
		break;

	case 2: Moving_Angle = -170; Speed = 40; Angle_Robot = 0; Accel = 60;

		if (IsSide_Midle)
		{
			subRun_Order++;
		}
		break;

	case 3: Moving_Angle = -90; Speed = 30; Angle_Robot = 0; Accel = 500;
	
		if (FindBackLine((Team == RED_TEAM) ? RED : BLUE))
		{
			subRun_Order++;
		}
		else if(!IsSide_Back)
		{
			subRun_Order++;
		}
		break;

	case 4: Moving_Angle = 0; Speed = 0; Angle_Robot = 0; Accel = 1000;

		subRun_Order = 0;
		IsWaiting = true;
		IsReady = false;
		ShootMode = SHOOT_TZ3;
		break;

	default:
		break;
	}
}

void MovingToTZ_TWO() {
	if (Current_Position != THROWING_ZONE1) return;
	if (!IsMovingTz) return;

	switch (Run_Order)
	{

	case 0:
		DelayMs.Counter = millis();
		Run_Order++;
		IsWaiting = false;
		break;

	case 1:
		if (DelayMs.isSchedule(200))
		{
			Run_Order++;
		}
		break;

	case 2:
		Moving_Angle = -40; Speed = 90; Angle_Robot = 0; Accel = 200;
		DelayMs.Counter = millis();
		Run_Order++;
		break;

	case 3:
		if (DelayMs.isSchedule(1500))
		{
			Run_Order++;
		}
		break;

	case 4:
		Moving_Angle = 0; Speed = 80; Angle_Robot = 0; Accel = 200;
		DelayMs.Counter = millis();
		Run_Order++;
		break;

	case 5:
		if (DelayMs.isSchedule(600))
		{
			Run_Order++;
		}
		break;
	

	case 6:Moving_Angle = 0; Speed = 60; Angle_Robot += 0.007; Accel = 200;
		
		ShootMode = SHOOT_TZ2;
		if (Angle_Robot > 82)
		{
			Angle_Robot = 82;
		}
		if (Angle_Mpu > 80)
		{
			Run_Order++;
		}
		break;

	case 7:Moving_Angle = 15; Speed = 70; Angle_Robot = 81.5; Accel = 200;

		if (IsFront_Left)
		{
			Run_Order++;
		}
		break;

	case 8:Moving_Angle = 50; Speed = 60; Angle_Robot = 82; Accel = 200;

		if (IsFront_Center)
		{
			Run_Order=10;
		}
		break;

	case 9:Moving_Angle = 90; Speed = 0; Accel = 1000;		
		
		TurnMode = TURNCAMERA;
		if (Team == RED_TEAM)
		{
			if (RedTreePosition >= -CAMERA_OFFSET && RedTreePosition <= CAMERA_OFFSET)
			{
				Run_Order++;
			}
		}
		else if (Team == BLUE_TEAM)
		{
			if (BlueTreePosition >= -CAMERA_OFFSET && BlueTreePosition <= CAMERA_OFFSET)
			{
				Run_Order++;
			}
		}
		break;

	case 10:Moving_Angle = 90; Speed = 0; Accel = 1000;

		DelayMs.Counter = millis();		
		Run_Order++;
		break;

	case 11:
		AngleServo = GRIP_OPEN;
		if (DelayMs.isSchedule(600))
		{
			Run_Order++;
		}
		break;

	case 12:
		
		IsShoot = true;
		CounterShoot++;
		TurnMode = TURNMPU;
		Run_Order++;
		break;

	case 13:

		if (!IsShoot) ReturnWaitTwo();
		if (IsWaiting)
		{
			IsMovingTz = false;
			Current_Position = THROWING_ZONE2;
			Run_Order = 0;
		}
		break;

	default:
		break;
	}
}

void ReturnWaitTwo() {
	switch (subRun_Order)
	{
	case 0: Moving_Angle = -90; Speed = 40; Accel = 200;

		DelayMs.Counter = millis();
		subRun_Order++;
		break;

	case 1: Moving_Angle = -90; Speed = 40; Accel = 200;

		if (DelayMs.isSchedule(500))
		{
			subRun_Order++;
		}
		break;

	case 2: Moving_Angle = 0; Speed = 0; Angle_Robot = 0; Accel = 200;

		if (Angle_Mpu < 5)
		{
			subRun_Order++;
		}
		break;

	case 3: Moving_Angle = -170; Speed = 45; Angle_Robot = 0; Accel = 200;

		if (IsSide_Midle)
		{
			subRun_Order++;
		}
		break;

	case 4: Moving_Angle = -90; Speed = 30; Angle_Robot = 0; Accel = 300;

		if (FindBackLine((Team == RED_TEAM) ? RED : BLUE))
		{
			subRun_Order++;
		}
		else if (!IsSide_Back)
		{
			subRun_Order++;
		}
		break;

	case 5: Moving_Angle = 0; Speed = 0; Angle_Robot = 0; Accel = 1000;

		subRun_Order = 0;
		IsWaiting = true;
		IsReady = false;
		//ShootMode = SHOOT_TZ3;
		break;

	default:
		break;
	}
}

void MovingToTZ_THREE() {
	if (Current_Position != THROWING_ZONE2) return;
	if (!IsMovingTz) return;

	switch (Run_Order)
	{

	case 0:
		DelayMs.Counter = millis();
		Run_Order++;
		IsWaiting = false;
		break;

	case 1:
		if (DelayMs.isSchedule(200))
		{
			Run_Order++;
		}
		break;

	case 2: Moving_Angle = 90 - Angle_Robot; Speed = 120; Angle_Robot += 0.003; Accel = 200;

		ShootMode = SHOOT_TZ3;		
		if (Moving_Angle > 90)
		{
			Moving_Angle = 90;
		}
		if (Angle_Mpu > 88)
		{
			Run_Order++;
		}
		break;
	case 3:
		Moving_Angle = 0; Speed = 110; Angle_Robot = 90; Accel = 200;
		DelayMs.Counter = millis();
		Run_Order++;
		break;

	case 4:
		Moving_Angle = 0;
		if (!IsFront_Center) Moving_Angle = 10;
		if (DelayMs.isSchedule(1000))
		{
			Run_Order++;
		}
		break;

	case 5: Moving_Angle = 0; Angle_Robot = 90; Accel = 300; 

		if (!IsFront_Center) Moving_Angle = 10;

		if (YellowTreePosition != 999)
		{
			Speed = constrain(abs(YellowTreePosition), 70, 120);
			if (YellowTreePosition > -30)
			{
				Run_Order++;
			}
		}
		break;

	case 6: Moving_Angle = 0; Speed = 40; Angle_Robot = 92; Accel = 65;

		if (FindRightLine(WHITE))
		{
			Run_Order++;
		}
		else if (BackOffset < 0.5 && BackOffset > -1)
		{
			Run_Order = 8;
		}
		break;

	case 7: Moving_Angle = 0; Speed = 30; Angle_Robot = 93; Accel = 50;
		if (BackOffset < 0.5 && BackOffset > -1)
		{
			Run_Order = 9;
		}
		break;

	case 8: Moving_Angle = 0; Speed = 0; Accel = 1000;

		TurnMode = TURNCAMERA;
		if (YellowTreePosition >= -CAMERA_OFFSET && YellowTreePosition <= CAMERA_OFFSET_R)
		{
			Run_Order++;
		}
		break;

	case 9:Moving_Angle = 0; Speed = 0; Angle_Robot = 93; Accel = 1000;
		DelayMs.Counter = millis();		
		Run_Order++;
		break;

	case 10: 
		AngleServo = GRIP_OPEN;
		if (DelayMs.isSchedule(700))
		{
			Run_Order++;
		}
		break;

	case 11: Moving_Angle = 0; Speed = 0; Accel = 500;
		
		IsShoot = true;
		CounterShoot++;
		TurnMode = TURNMPU;
		Run_Order++;
		break; 

	case 12:
		
		if (!IsShoot) ReturnTZ_THREE();
		if (IsWaiting)
		{
			IsMovingTz = false;
			//Current_Position = THROWING_ZONE3;
			Run_Order = 0;
		}
		break;

	default:
		break;
	}
}

void ReturnTZ_THREE() {

	switch (subRun_Order)
	{
	case 0: Moving_Angle = -90; Speed = 40; Accel = 200;
		DelayMs.Counter = millis();
		subRun_Order++;
		break;

	case 1: Moving_Angle = -90; Speed = 40; Accel = 200;

		if (DelayMs.isSchedule(400))
		{
			subRun_Order++;
		}	
		break;

	case 2: Moving_Angle = 0; Speed = 0; Angle_Robot = 0; Accel = 200;

		if (Angle_Mpu < 4) 
		{
			subRun_Order++;
		}
		break;

	case 3: Moving_Angle = -90; Speed = 110; Angle_Robot = 0; Accel = 200;
		DelayMs.Counter = millis();
		subRun_Order++;
		break;

	case 4:Moving_Angle = -90; Speed = 110; Angle_Robot = 0; Accel = 200;
		if (DelayMs.isSchedule(1900))
		{
			subRun_Order++;
		}
		break;

	case 5:Moving_Angle = -90; Speed = 90; Angle_Robot = 0; Accel = 200;

		if (FindBackLine(WHITE))
		{
			subRun_Order++;
		}
		break;

	case 6:
		if (!FindBackLine(WHITE))
		{
			subRun_Order++;
		}
		break;

	case 7: Moving_Angle = -90; Speed = 30; Angle_Robot = 0; Accel = 40;

		if (!IsSide_Midle) Moving_Angle = -110;

		if (FindBackLine((Team == RED_TEAM) ? RED : BLUE))
		{
			subRun_Order++;
		} 
		else if (!IsSide_Back)
		{
			subRun_Order++;
		}

		break;

	case 8: Moving_Angle = -90; Speed = 0; Accel = 1000;

		Current_Position = THROWING_ZONE2;
		subRun_Order = 0;
		IsWaiting = true;
		IsReady = false; 
		break;

	//
	case 9: Moving_Angle = 90; Speed = 30; Accel = 1000;

		if (FindBackLine((Team == RED_TEAM) ? RED : BLUE))
		{
			subRun_Order++;
		}
		break;

	case 10: Moving_Angle = 180; Speed = 30; Accel = 1000;

		if (IsSide_Midle)
		{
			subRun_Order = 8;
		}
		break;
		
	default:
		break;
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//RELAY/////////////////////////////////////////////////////////////////////////////////////////////////////////////
void RelayInit() {
	pinMode(RELAY_SHOOT, OUTPUT);
	pinMode(RELAY_TZ1, OUTPUT);
	pinMode(RELAY_TZ2, OUTPUT);
	pinMode(RELAY_TZ3, OUTPUT);

	digitalWrite(RELAY_SHOOT, 1);
	digitalWrite(RELAY_TZ1, 1);
	digitalWrite(RELAY_TZ2, 1);
	digitalWrite(RELAY_TZ3, 1);
}
void Lock() {
	int Pin_Relay[3] = { RELAY_TZ1 , RELAY_TZ2 , RELAY_TZ3 };
	if (IsShoot) return;
	if (!IsLock)
	{
		digitalWrite(RELAY_TZ3, 0);
		RUN_EVERY(LockScheduler, Time_Lock);
		IsLock = true;
	}
	else
	{
		LockScheduler.Counter = millis();
		for (int _i = 0; _i < 3; _i++)
		{
			if (_i == ShootMode)
			{
				digitalWrite(Pin_Relay[_i], 0);
			}
			else
			{
				digitalWrite(Pin_Relay[_i], 1);
			}
		}
	}
}
void Shoot() {	
	if (!IsShoot)
	{
		ShootScheduler.Counter = millis();
	}
	else
	{
		if (IsLock)
		{
			digitalWrite(RELAY_SHOOT, 0);
			IsLock = false;
		}
	}

	if (ShootMode == SHOOT_TZ1) Time_Shoot = TIME_SHOOT_TZ1;
	else Time_Shoot = TIME_SHOOT;

	RUN_EVERY(ShootScheduler, Time_Shoot);
	IsShoot = false;
	digitalWrite(RELAY_SHOOT, 1);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Calculator speed//////////////////////////////////////////////////////////////////////////////////////////////////
void UploadOmni() {
	RUN_EVERY(UploadOmniScheduler, 10);
	Acceleration();
	OMNI3.Move(Moving_Angle, Present_Speed, Turn);
}
void Acceleration() {
	if (Speed > Present_Speed)
	{
		Present_Speed += Accel * 0.01;
		if (Present_Speed > Speed) Present_Speed = Speed;
	}
	else if (Speed < Present_Speed)
	{
		Present_Speed -= Accel * 0.01;
		if (Present_Speed < Speed) Present_Speed = Speed;
	}
	else
	{
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
