#ifdef __cplusplus
extern "C" {
#endif

#include "millisecondtimer.h"

volatile uint32_t _counter;

void initialise(void) 
{
	_counter = 0;
	SysTick_Config(SystemCoreClock / 1000);			//1ms定时器
}
//ms级别的延时
void delay(uint32_t millis) { 
	uint32_t target;	
	target = _counter + millis;
	while(_counter <= target);
} 
void delay_us(uint32_t uillis)
{ 
	uint32_t target;
  //SysTick_Config(SystemCoreClock / 100000);
	target = (uint64_t)_counter*1000 + uillis;
	while(millis_us() <= target);
}
void SysTick_Handler(void) {
	_counter++;
	if(_counter>4200000) _counter=0;
}
uint32_t millis_us(void){
	return (_counter+1)*1000-SysTick->VAL/(SystemCoreClock/1000000);
}

uint32_t millis(void) 
{
	return _counter;
}

void reset(void) 
{
	_counter = 0;
}

#ifdef __cplusplus
}
#endif

