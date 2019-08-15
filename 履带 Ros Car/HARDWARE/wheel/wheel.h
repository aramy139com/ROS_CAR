#ifndef _WHEEL_H_
#define _WHEEL_H_
#include "Gpio.h"
#include "config.h"
//车轮相关的类
//轮子拥有的属性有 pwm值，
class Wheel{
	public:
		//初始化部分  这个是 2018版的 使用驱动为 A4950  
		void initialize(uint8_t _side,uint16_t _arr=999,uint16_t _psc=3);
		void setPWM(int16_t _pwm);
		int16_t getPWM(){
			return pwm;
		}
		double getSpeed(){						//获取当前速度
			return speed;
		}
		void setSpeed(double _speed){	//设置当前速度
			speed=_speed;
		}
	private:
		uint8_t side;
		uint16_t arr;
		uint16_t psc;
		//pwm值
		int16_t pwm;
		//轮子的速度  单位 米/秒  通过编码器 和轮子直径 计算出来的。
	  double speed;
};
#endif // _WHEEL_H_