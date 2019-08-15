#ifndef _WHEEL_H_
#define _WHEEL_H_
#include "Gpio.h"
#include "config.h"
//������ص���
//����ӵ�е������� pwmֵ��
class Wheel{
	public:
		//��ʼ������  ����� 2018��� ʹ������Ϊ A4950  
		void initialize(uint8_t _side,uint16_t _arr=999,uint16_t _psc=3);
		void setPWM(int16_t _pwm);
		int16_t getPWM(){
			return pwm;
		}
		double getSpeed(){						//��ȡ��ǰ�ٶ�
			return speed;
		}
		void setSpeed(double _speed){	//���õ�ǰ�ٶ�
			speed=_speed;
		}
	private:
		uint8_t side;
		uint16_t arr;
		uint16_t psc;
		//pwmֵ
		int16_t pwm;
		//���ӵ��ٶ�  ��λ ��/��  ͨ�������� ������ֱ�� ��������ġ�
	  double speed;
};
#endif // _WHEEL_H_