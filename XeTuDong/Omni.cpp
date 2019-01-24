//  Van Than
//	
//	

#include "Omni.h"

Omni::Omni(int8_t _dir1, int8_t _pwn1, int8_t _dir2, int8_t _pwn2, int8_t _dir3, int8_t _pwn3)
{	
	dir1 = _dir1;
	dir2 = _dir2;
	dir3 = _dir3;
	pwm1 = _pwn1;
	pwm2 = _pwn2;
	pwm3 = _pwn3;
}
Omni::Omni()
{
	dir1 = PQ_1;
	dir2 = PP_3;
	dir3 = PA_7;
	pwm1 = PM_6;
	pwm2 = PM_2;
	pwm3 = PM_1;
}
void Omni::Move(float angle, float speed, float turn)
{
	vector2speed(angle, speed, turn);
	if (speed1<3 && speed1>-3)speed1 = 2;
	if (speed2<3 && speed2>-3)speed2 = 2;
	if (speed3<3 && speed3>-3)speed3 = 2;

	setspeed(speed1, speed2, speed3);
}
void Omni::Init()
{
	pinMode(dir1, OUTPUT);
	pinMode(dir2, OUTPUT);
	pinMode(dir3, OUTPUT);
	pinMode(pwm1, OUTPUT);
	pinMode(pwm2, OUTPUT);
	pinMode(pwm3, OUTPUT);
	speed1 = 0;
	speed2 = 0;
	speed3 = 0;
}
void Omni::Stop()
{
	setspeed(0, 0, 0);
}
void Omni::Brake()
{
	setspeed(2, 2, 2);
}

//
void Omni::setspeed(int16_t _speed1, int16_t _speed2, int16_t _speed3)
{
	if (_speed1 > 0) digitalWrite(dir1, 1);
		else digitalWrite(dir1, 0);
	if (_speed2 > 0) digitalWrite(dir2, 1);
		else digitalWrite(dir2, 0);
	if (_speed3 > 0) digitalWrite(dir3, 1);
		else digitalWrite(dir3, 0);

	_speed1 = abs(constrain(_speed1, -255, 255));
	_speed2 = abs(constrain(_speed2, -255, 255));
	_speed3 = abs(constrain(_speed3, -255, 255));
	
	analogWrite(pwm1, 255 - _speed1);
	analogWrite(pwm2, 255 - _speed2);
	analogWrite(pwm3, 255 - _speed3);
}
void Omni::vector2speed(float _angle, float _speed, float _turn)
{
	double beta = _angle*PI/180;
	float Fx = _speed*cos(beta);
	float Fy = _speed*sin(beta);

	speed1 = (int16_t)(_turn + 2 * Fx / 3);
	speed2 = (int16_t)(_turn - Fx / 3 - Fy / sqrt(3));
	speed3 = (int16_t)(_turn - Fx / 3 + Fy / sqrt(3));
}