// LineADC.h

#include <EEPROM.h>
#ifndef _LINEADC_h
#define _LINEADC_h
#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#define ADDRESS_EEPROM_ADC 11

class LineADC
{
	int pins[17] = {PK_3,PK_2,PK_1,PK_0,PB_5,PB_4,PD_5,PD_4,PD_2,PD_7,PE_3,PE_2,PE_1,PE_0,PD_3,PE_5,PE_4};
	int valsMid[17];
	int valsMax[17];
	int valsMin[17];
	int vals[17];

	EEPROMClass ADCEEPROM;

	boolean IsUpdate = false;
	
	void ReadADC();
	void UpdateMidvals();
 protected:   
	 
 public:
	 //LineADC();
	void Init();
	void UpdateADC();
	void DoneUpdateADC();
	void LineRead(int *x, int *y, int lenga);
};
#endif

