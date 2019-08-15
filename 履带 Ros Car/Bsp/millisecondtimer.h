#ifdef __cplusplus
extern "C" {
#endif
#ifndef _MILLISECONDTIMER_H_
#define _MILLISECONDTIMER_H_
#include "stm32f10x.h"

void initialise(void);
void delay(uint32_t millis_);
void delay_us(uint32_t uillis);
uint32_t millis(void);
uint32_t millis_us(void);
void reset(void);

#endif // _MILLISECONDTIMER_H_ 
#ifdef __cplusplus
}
#endif



