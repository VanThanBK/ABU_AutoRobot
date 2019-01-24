// 
// 
// 
#include "LineADC.h"

void LineADC::Init()
{
	int demmid[34];
	for (int i_ = 0; i_ < 34; i_++) {
		demmid[i_] = ADCEEPROM.read(ADDRESS_EEPROM_ADC + i_);
	}
	for (int i_ = 0; i_ < 17; i_++)
	{
		pinMode(pins[i_], OUTPUT);
		valsMid[i_] = (demmid[i_*2] << 8) + demmid[i_*2+1];
	}
}
void LineADC::UpdateADC()
{
	ReadADC();
	if (IsUpdate == false)
	{
		for (int i_ = 0; i_ < 17; i_++)
		{
			valsMax[i_] = vals[i_];
			valsMin[i_] = vals[i_];
		}
		IsUpdate = true;
	}
	UpdateMidvals();
}
void LineADC::DoneUpdateADC()
{
	int demmid[34];
	for (int i_ = 0; i_ < 17; i_++)
	{
		demmid[i_*2+1] = valsMid[i_];
		demmid[i_ * 2] = valsMid[i_] >> 8;
	}
	IsUpdate = false;
	for (int i_ = 0; i_ < 34; i_++) {
		ADCEEPROM.write(ADDRESS_EEPROM_ADC + i_, demmid[i_]);
	}
}

void LineADC::LineRead(int *x, int *y ,int lenga)
{
	ReadADC();
	for (int i_ = 0; i_ < 9; i_++)
	{
		if (vals[i_] > valsMid[i_]-10)
		{
			x[i_] = 0;
		}
		else
		{
			x[i_] = 1;
		}
	}

	for (int i_ = 9; i_ < 13; i_++)
	{
		if (vals[i_] > valsMid[i_]-10)
		{
			y[i_ - 9] = 0;
		}
		else
		{
			y[i_ - 9] = 1;
		}
	}
	if (vals[4] > valsMid[4]-10) 
	{
		y[4] = 0;
	}
	else
	{
		y[4] = 1;
	}
	for (int i_ = 13; i_ < 17; i_++)
	{
		if (vals[i_] > valsMid[i_]-10)
		{
			y[i_ - 8] = 0;
		}
		else
		{
			y[i_ - 8] = 1;
		}
	}
}
//
void LineADC::ReadADC()
{
	for (int i_ = 0; i_ < 17; i_++)
	{
		vals[i_] = analogRead(pins[i_]);
	}
}
void LineADC::UpdateMidvals()
{
	for (int i_ = 0; i_ < 17; i_++)
	{
		if (vals[i_] > valsMax[i_])
		{
			valsMax[i_] = vals[i_];
		}
		else if (vals[i_] < valsMin[i_])
		{
			valsMin[i_] = vals[i_];
		}
	}
	for (int i_ = 0; i_ < 17; i_++)
	{
		valsMid[i_] = (valsMax[i_] + valsMin[i_]) / 2;
	}
}
