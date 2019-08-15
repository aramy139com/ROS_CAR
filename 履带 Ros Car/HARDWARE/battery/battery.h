#ifndef _BATTERY_H_
#define _BATTERY_H_ 
#include "config.h"
#define Battery_Ch 4
#define ADC1_DR_ADDRESS         		((u32)0x4001244C)

class Battery {
public:
	Battery();
	void initialize();
	float get_volt();
private:
//	float threshold;
//	float volt_min;
//	float volt_max;
	float currpower;
	__IO u16 ADC_ConvertedValue[1];
};

#endif // _BATTERY_H_
