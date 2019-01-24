// Omni.h

#include "MultiThread.h"
#ifndef _OMNI_h
#define _OMNI_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif


class Omni
{
	int8_t dir1, dir2, dir3;
	int8_t pwm1, pwm2, pwm3;
	int16_t speed1, speed2, speed3;
	void setspeed(int16_t _speed1, int16_t _speed2, int16_t _speed3);
	void vector2speed(float _angle, float _speed, float _turn);
 protected:
	 
 public:
	 Omni(int8_t _dir1, int8_t _pwn1, int8_t _dir2, int8_t _pwn2, int8_t _dir3, int8_t _pwn3);
	 Omni();
	 void Init();
	 void Move(float _angle, float _speed, float _turn);
	 void Stop();
	 void Brake();
};

#endif

