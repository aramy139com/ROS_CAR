#ifndef _ENCODER_H_
#define _ENCODER_H_
#include "config.h"
#include "Gpio.h"
class Encoder {
public:
    //初始化部分
    void initialize(uint8_t _side);
    void setEncoder();						//获取一次码盘信息
		int16_t getEnValue() {		//获取偏移量 在时间周期内
        return en_value;
    }
    int32_t getTotleValue() {	//获得累计的偏移量
        return total_value;
    }
    void setTotleValue(uint32_t value) {
        total_value=value;
    }
    void addTotleValue(uint16_t value) {
        total_value+=value;
    }
private:
    uint8_t side;
    int16_t en_value;				//当前周期变化值
    int32_t total_value;		//累计变化量
   
    TIM_TypeDef* TIMx;
};
#endif // _ENCODER_H_