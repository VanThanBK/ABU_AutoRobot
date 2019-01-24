// 
// 
// 

#include "FloorSensorsMaster.h"

void FloorSensorsMaster::request(void)
{
	if (api_is_busy == true)
	{
		LOGLN("SLAVE IS BUSY!");
		return;
	}
	api_is_busy = true;
	api_request_tick = millis();
	port->write(START_CMD);
	switch (color)
	{
	case RED:
		port->write(COLOR_CMD_R);
		break;
	case BLUE:
		port->write(COLOR_CMD_B);
		break;
	case WHITE:
		port->write(COLOR_CMD_W);
		break;
	}
	//Request
	/*port->println("RQ");
	switch (color)
	{
	case RED:
	port->println("SR");
	break;
	case BLUE:
	port->println("SB");
	break;
	case WHITE:
	port->println("SW");
	break;
	}*/
	//done
}

void FloorSensorsMaster::waitForSlave(void)
{
	if (!port->available())
	{
		if (millis() - api_request_tick > TIME_OUT && api_is_busy)
		{
			LOGLN("SLAVE TIME OUT");
			api_is_busy = false;
		}
		return;
	}
	//new data comes
	api_is_busy = false;
	char data[3];
	/*uint8_t count = 0;
	//read data from slave
	//LOG("[");
	while (port->available())
	{
	if (count >= 3)
	{
	LOGLN("MS BUFFER OVERFLOWS");
	//flush buffer
	while (port->available())
	port->read();
	}
	data[count] = port->read();
	//LOGHEX(data[count]);
	//LOG(',');
	count++;
	api_waitByte();
	}
	//LOGLN("]");
	*/
	port->readBytes(data, sizeof(data));
	api_prox_values = data[0];
	api_side_analog_values = data[1];
	api_back_analog_values = data[2];
}

void FloorSensorsMaster::init(HardwareSerial * _port)
{
	port = _port;
}

void FloorSensorsMaster::run()
{
	//request from slave each 4ms
	static uint32_t timer = 0;
	if (millis() - timer > 5)
	{
		timer = millis();
		request();
	}
	waitForSlave();
}

uint8_t FloorSensorsMaster::getProximityState(uint8_t name)
{
	return bitRead(api_prox_values, name);
}

uint8_t FloorSensorsMaster::getSideAnalogState(uint8_t index)
{
	return bitRead(api_side_analog_values, index);;
}

uint8_t FloorSensorsMaster::getBackAnalogState(uint8_t index)
{
	return bitRead(api_back_analog_values, index);;
}

float FloorSensorsMaster::getSideOffset()
{
	float offset_LTR = 0;
	float offset_RTL = 0;
	//full line
	if (api_side_analog_values == 127)
		return 0;
	if (api_side_analog_values == 0x00)
		return NOT_FOUND;
	for (uint8_t i = 0; i < 7; i++)
	{
		if (getSideAnalogState(i) == 1)
		{
			offset_LTR = float(i - 3);
			break;
		}
	}
	for (int8_t i = 6; i >= 0; i--)
	{
		if (getSideAnalogState(i) == 1)
		{
			offset_RTL = float(i - 3);
			break;
		}
	}
	return (offset_LTR + offset_RTL) / -2.0f;
}

float FloorSensorsMaster::getBackOffset()
{
	float offset_LTR = 0;
	float offset_RTL = 0;
	if (api_back_analog_values == 0xFF)
		return 0;
	if (api_back_analog_values == 0x00)
		return NOT_FOUND;
	for (uint8_t i = 0; i < 7; i++)
	{
		if (getBackAnalogState(i) == 1)
		{
			offset_LTR = float(i - 3);
			break;
		}
	}
	for (int8_t i = 6; i >= 0; i--)
	{
		if (getBackAnalogState(i) == 1)
		{
			offset_RTL = float(i - 3);
			break;
		}
	}
	return (offset_LTR + offset_RTL) / -2.0f;
}

void FloorSensorsMaster::setColor(uint8_t _color)
{
	color = _color;
}


FloorSensorsMaster FloorSensors;
