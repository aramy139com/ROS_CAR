#ifndef _ENCODER_H_
#define _ENCODER_H_
#include "config.h"
#include "Gpio.h"
class Encoder {
public:
    //��ʼ������
    void initialize(uint8_t _side);
    void setEncoder();						//��ȡһ��������Ϣ
		int16_t getEnValue() {		//��ȡƫ���� ��ʱ��������
        return en_value;
    }
    int32_t getTotleValue() {	//����ۼƵ�ƫ����
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
    int16_t en_value;				//��ǰ���ڱ仯ֵ
    int32_t total_value;		//�ۼƱ仯��
   
    TIM_TypeDef* TIMx;
};
#endif // _ENCODER_H_