#include "wheel.h"

//初始化pwm  
//电机使用了
//右电机 TIM1_CH1:PA8
//左电机 TIM1_CH4:PA11
//电机 初始化
//电机的控制管脚
//左电机 PB0 PB1  右电机 PA6 PA7
void Wheel::initialize(uint8_t _side,uint16_t _arr,uint16_t _psc) {
    side = _side;
    arr=_arr;
    psc=_psc;
    //根据STM32中文参考手册2010中第第119页可知：
    //当没有重映射时，TIM3的四个通道CH1，CH2，CH3，CH4分别对应PA6，PA7,PB0,PB1	此次使用这四个通道
    //当部分重映射时，TIM3的四个通道CH1，CH2，CH3，CH4分别对应PB4，PB5,PB0,PB1
    //当完全重映射时，TIM3的四个通道CH1，CH2，CH3，CH4分别对应PC6，PC7,PC8,PC9
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
    TIM_TimeBaseStructure.TIM_Period = arr;       //当定时器从0计数到999，即为1000次，为一个定时周期
    TIM_TimeBaseStructure.TIM_Prescaler = psc;	    //设置预分频：不预分频，即为72MHz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	//设置时钟分频系数：不分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    /* PWM2 Mode configuration: Channel3 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //配置为PWM模式1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;	   //设置跳变值，当计数器计数到这个值时，电平发生跳变
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //当定时器计数值小于CCR1_Val时为高电平

    if(side==LEFT) {
        TIM_OC3Init(TIM3, &TIM_OCInitStructure);	 //使能通道3
        TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

        TIM_OC4Init(TIM3, &TIM_OCInitStructure);	  //使能通道4
        TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
    } else {
        TIM_OC1Init(TIM3, &TIM_OCInitStructure);	  //使能通道1
        TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

        TIM_OC2Init(TIM3, &TIM_OCInitStructure);	  //使能通道2
        TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    }

    TIM_ARRPreloadConfig(TIM3, ENABLE);			 // 使能TIM3重载寄存器ARR
    TIM_Cmd(TIM3, ENABLE);
}

//设置PWM  入口值从0 ~ arr
void Wheel::setPWM(int16_t _pwm) {
    uint16_t currpwm,blind;
    pwm=_pwm;
		//左右轮的盲区
		 if(side==LEFT) {
			 blind=LEFTBLIND;
		 }else{
			  blind=RIGHTBLIND;
		 }
		 
    if(_pwm<0) {
        currpwm=-_pwm+blind;
        //控制极值
        if(currpwm > arr) {
            currpwm =arr;
            pwm=-currpwm;
        }
        //输出速度
        if(side==LEFT) {
            TIM_SetCompare3(TIM3,currpwm);
            TIM_SetCompare4(TIM3,0);
        } else {
            TIM_SetCompare1(TIM3,currpwm);
            TIM_SetCompare2(TIM3,0);
        }
    }	else {
        currpwm=_pwm+blind;
        //控制极值
        if(currpwm > arr) {
            currpwm =arr;
            pwm=currpwm;
        }
        //输出速度
        if(side==LEFT) {
            TIM_SetCompare4(TIM3,currpwm);
            TIM_SetCompare3(TIM3,0);
        } else {
            TIM_SetCompare2(TIM3,currpwm);
            TIM_SetCompare1(TIM3,0);
        }
    }
}