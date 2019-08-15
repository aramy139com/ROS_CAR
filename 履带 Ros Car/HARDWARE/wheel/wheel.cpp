#include "wheel.h"

//��ʼ��pwm  
//���ʹ����
//�ҵ�� TIM1_CH1:PA8
//���� TIM1_CH4:PA11
//��� ��ʼ��
//����Ŀ��ƹܽ�
//���� PB0 PB1  �ҵ�� PA6 PA7
void Wheel::initialize(uint8_t _side,uint16_t _arr,uint16_t _psc) {
    side = _side;
    arr=_arr;
    psc=_psc;
    //����STM32���Ĳο��ֲ�2010�еڵ�119ҳ��֪��
    //��û����ӳ��ʱ��TIM3���ĸ�ͨ��CH1��CH2��CH3��CH4�ֱ��ӦPA6��PA7,PB0,PB1	�˴�ʹ�����ĸ�ͨ��
    //��������ӳ��ʱ��TIM3���ĸ�ͨ��CH1��CH2��CH3��CH4�ֱ��ӦPB4��PB5,PB0,PB1
    //����ȫ��ӳ��ʱ��TIM3���ĸ�ͨ��CH1��CH2��CH3��CH4�ֱ��ӦPC6��PC7,PC8,PC9
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    if(side==LEFT) {
        Gpio(PB,0,GM_AFPP);
        Gpio(PB,1,GM_AFPP);
    } else {
        Gpio(PA,6,GM_AFPP);
        Gpio(PA,7,GM_AFPP);
    }

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = arr;       //����ʱ����0������999����Ϊ1000�Σ�Ϊһ����ʱ����
    TIM_TimeBaseStructure.TIM_Prescaler = psc;	    //����Ԥ��Ƶ����Ԥ��Ƶ����Ϊ72MHz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	//����ʱ�ӷ�Ƶϵ��������Ƶ
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //���ϼ���ģʽ
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    /* PWM2 Mode configuration: Channel3 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //����ΪPWMģʽ1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;	   //��������ֵ�������������������ֵʱ����ƽ��������
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ

    if(side==LEFT) {
        TIM_OC3Init(TIM3, &TIM_OCInitStructure);	 //ʹ��ͨ��3
        TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

        TIM_OC4Init(TIM3, &TIM_OCInitStructure);	  //ʹ��ͨ��4
        TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
    } else {
        TIM_OC1Init(TIM3, &TIM_OCInitStructure);	  //ʹ��ͨ��1
        TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

        TIM_OC2Init(TIM3, &TIM_OCInitStructure);	  //ʹ��ͨ��2
        TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    }

    TIM_ARRPreloadConfig(TIM3, ENABLE);			 // ʹ��TIM3���ؼĴ���ARR
    TIM_Cmd(TIM3, ENABLE);
}

//����PWM  ���ֵ��0 ~ arr
void Wheel::setPWM(int16_t _pwm) {
    uint16_t currpwm,blind;
    pwm=_pwm;
		//�����ֵ�ä��
		 if(side==LEFT) {
			 blind=LEFTBLIND;
		 }else{
			  blind=RIGHTBLIND;
		 }
		 
    if(_pwm<0) {
        currpwm=-_pwm+blind;
        //���Ƽ�ֵ
        if(currpwm > arr) {
            currpwm =arr;
            pwm=-currpwm;
        }
        //����ٶ�
        if(side==LEFT) {
            TIM_SetCompare3(TIM3,currpwm);
            TIM_SetCompare4(TIM3,0);
        } else {
            TIM_SetCompare1(TIM3,currpwm);
            TIM_SetCompare2(TIM3,0);
        }
    }	else {
        currpwm=_pwm+blind;
        //���Ƽ�ֵ
        if(currpwm > arr) {
            currpwm =arr;
            pwm=currpwm;
        }
        //����ٶ�
        if(side==LEFT) {
            TIM_SetCompare4(TIM3,currpwm);
            TIM_SetCompare3(TIM3,0);
        } else {
            TIM_SetCompare2(TIM3,currpwm);
            TIM_SetCompare1(TIM3,0);
        }
    }
}