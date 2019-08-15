//控制编码器的类
#include "encoder.h"
#include "millisecondtimer.h"

void Encoder::initialize(uint8_t _side) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    side = _side;
    if(side==LEFT) {
        TIMx=TIM4;
    } else {
        TIMx=TIM2;
    }
    en_value=0;
    total_value=0;
    //初始化时钟
    if(side==LEFT) {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//使能定时器2的时钟
        Gpio(PB,6,GM_IN_FLOATING);
        Gpio(PB,7,GM_IN_FLOATING);
    } else {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//使能定时器4的时钟
				Gpio(PA,0,GM_IN_FLOATING);
        Gpio(PA,1,GM_IN_FLOATING);
    }
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // 预分频器
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF; //设定计数器自动重装值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//选择时钟分频：不分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM向上计数
    TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);
    TIM_EncoderInterfaceConfig(TIMx, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码器模式3
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 10;
    TIM_ICInit(TIMx, &TIM_ICInitStructure);
    TIM_ClearFlag(TIMx, TIM_FLAG_Update);			//清除TIM的更新标志位
    TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE);
    //Reset counter
    TIM_SetCounter(TIMx,0);
    TIM_Cmd(TIMx, ENABLE);

}
//采集电机速度脉冲
void Encoder::setEncoder() {
    if(side==LEFT) {
        //en_value = TIM_GetCounter(TIMx);
				en_value = -1*TIM_GetCounter(TIMx);
    } else {
        en_value = TIM_GetCounter(TIMx);
    }
    TIMx->CNT = 0;   //清零

    total_value+=en_value;    //计算速度值
}
