//���Ʊ���������
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
    //��ʼ��ʱ��
    if(side==LEFT) {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//ʹ�ܶ�ʱ��2��ʱ��
        Gpio(PB,6,GM_IN_FLOATING);
        Gpio(PB,7,GM_IN_FLOATING);
    } else {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//ʹ�ܶ�ʱ��4��ʱ��
				Gpio(PA,0,GM_IN_FLOATING);
        Gpio(PA,1,GM_IN_FLOATING);
    }
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // Ԥ��Ƶ��
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF; //�趨�������Զ���װֵ
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//ѡ��ʱ�ӷ�Ƶ������Ƶ
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM���ϼ���
    TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);
    TIM_EncoderInterfaceConfig(TIMx, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//ʹ�ñ�����ģʽ3
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 10;
    TIM_ICInit(TIMx, &TIM_ICInitStructure);
    TIM_ClearFlag(TIMx, TIM_FLAG_Update);			//���TIM�ĸ��±�־λ
    TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE);
    //Reset counter
    TIM_SetCounter(TIMx,0);
    TIM_Cmd(TIMx, ENABLE);

}
//�ɼ�����ٶ�����
void Encoder::setEncoder() {
    if(side==LEFT) {
        //en_value = TIM_GetCounter(TIMx);
				en_value = -1*TIM_GetCounter(TIMx);
    } else {
        en_value = TIM_GetCounter(TIMx);
    }
    TIMx->CNT = 0;   //����

    total_value+=en_value;    //�����ٶ�ֵ
}
