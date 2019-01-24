// FloorSensorsMaster.h

#ifndef _FLOORSENSORSMASTER_h
#define _FLOORSENSORSMASTER_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

//#define LOGGER	Serial

#ifdef LOGGER
#define LOG(x)		LOGGER.print(x)
#define LOGHEX(x)	LOGGER.print(x, HEX)
#define LOGLN(x)	LOGGER.println(x)
#define LOGF(x)		LOGGER.print(F(x))
#define LOGLNF(x)	LOGGER.println(F(x))
#else
#define LOG(x)		
#define LOGHEX(x)	
#define LOGLN(x)	
#define LOGF(x)		
#define LOGLNF(x)
#endif // LOGGER

#define TIME_OUT 100

#define API_BAUD	115200
#define NOT_FOUND	255.0

#define FL	0
#define FC	1
#define FR	2
#define SF	3
#define SM	4
#define SB	5

#define REQUEST_BYTE	100

#define RED	0
#define BLUE 1
#define WHITE 2

#define START_CMD		254
#define COLOR_CMD_R		32
#define COLOR_CMD_B		64
#define COLOR_CMD_W		128

class FloorSensorsMaster
{
public:
	HardwareSerial * port;

	//sensors value
	boolean api_is_busy = false;
	uint32_t api_request_tick = 0;
	uint8_t api_prox_values = 0;
	uint8_t api_side_analog_values = 0;
	uint8_t api_back_analog_values = 0;

	uint8_t color = RED;

	/*void api_waitByte()
	{
	const uint32_t WAIT_BYTE_TIMEOUT = 1000;
	uint32_t timer_wait = micros();

	while (micros() - timer_wait < WAIT_BYTE_TIMEOUT)
	{
	if (port->available())
	{
	//LOG("TIME: ");
	//LOGLN(micros() - timer_wait);
	break;
	}
	}
	}*/

	void request(void);
	void waitForSlave(void);
public:
	void init(HardwareSerial * _port);
	void run();
	uint8_t getProximityState(uint8_t name);
	uint8_t getSideAnalogState(uint8_t index);
	uint8_t getBackAnalogState(uint8_t index);
	float getSideOffset(void);
	float getBackOffset(void);
	//float getAngle();
	//float getOffset();
	void setColor(uint8_t _color);
};

extern FloorSensorsMaster FloorSensors;

#endif
