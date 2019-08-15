//Gpio.h
//#pragma once   //编译一次
#ifndef	__AOBO_stm3210x_Gpio_H_
#define 	__AOBO_stm3210x_Gpio_H_

#include "stm32f10x.h"

/*--------------------Gpio--------------------------------------*/
// //eg:
//  Gpio  key1(RCC_APB2Periph_GPIOC,GPIOC,GPIO_Pin_1,GM_IN_FLOATING);
//  Gpio  pins(RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO,GPIOC,GPIO_Pin_1|GPIO_Pin_10);
//  Gpio  EnTk(PA,0);
//   特别注意：
//   1.当你使用PB3,4 PA13,14,15 时一定要加上这2句：
//   		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
//		GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);  // 同时关闭 JLink 和 STlink
//   2.PD0,1 是不能被用做I/O口使用的 . 他们是接外部进制的


typedef struct   tagGPIO_PIN
{
    uint32_t			periph;//eg:RCC_APB2Periph_GPIOF
    GPIO_TypeDef*   		port;	 //eg:GPIOF
    uint16_t 			pin;	 //eg:GPIO_Pin_10
    GPIOMode_TypeDef  	mode;	 //eg.GPIO_Mode_IN_FLOATING;
    GPIOSpeed_TypeDef  	speed; 	 //eg.GPIO_Speed_50MHz
} GPIO_PIN;

enum PORT_INDEX
{
    PA=0,PB,PC,PD,PE,PF,PG
};

typedef struct _periph_PORT
{
    uint32_t		p_periph;
    GPIO_TypeDef*		p_port;
} periph_PORT;


#define	GM_AIN		      GPIO_Mode_AIN
#define	GM_IN_FLOATING		GPIO_Mode_IN_FLOATING
#define	GM_IPD			GPIO_Mode_IPD
#define	GM_IPU 			GPIO_Mode_IPU
#define	GM_OUT_OD 			GPIO_Mode_Out_OD
#define	GM_OUT_PP 			GPIO_Mode_Out_PP
#define	GM_AFOD 			GPIO_Mode_AF_OD
#define	GM_AFPP 			GPIO_Mode_AF_PP


class Gpio
{
private:
    GPIO_PIN m_gpio;
public:
    ~Gpio()
    {

    }
    Gpio()
    {

    }

    Gpio(PORT_INDEX 	indexPort,
         uint16_t 		      indexPin, //管脚0~15  值GPIO_Pin_0~GPIO_Pin_15
         GPIOMode_TypeDef		p_mode=GM_OUT_PP,
         GPIOSpeed_TypeDef		p_speed=GPIO_Speed_50MHz );
    Gpio( uint32_t		p_periph,
          GPIO_TypeDef*		p_port,
          uint16_t			p_pins,    //???????
          GPIOMode_TypeDef		p_mode=GPIO_Mode_Out_PP,
          GPIOSpeed_TypeDef		p_speed=GPIO_Speed_50MHz );

    void initialize( GPIOMode_TypeDef   p_mode=GPIO_Mode_Out_PP );

    void initialize( uint32_t    		p_periph,
                     GPIO_TypeDef*    		p_port,
                     uint16_t    		p_pins,    //???????,???RAM?????
                     GPIOMode_TypeDef   	p_mode=GPIO_Mode_Out_PP,
                     GPIOSpeed_TypeDef  	p_speed=GPIO_Speed_50MHz );

    inline  bool get(void) {
        if( ishigh() ) {
            return true;
        }
        else {
            return false;
        }
    }

    inline void set(bool bs) {
        if(bs) {
            high();//GPIO_SetBits(m_gpio.port, m_gpio.pin);
        }
        else {
            low();//GPIO_ResetBits(m_gpio.port, m_gpio.pin);
        }
    }

    inline void invert(void) {
        if ( ishigh() ) {
            low();//GPIO_ResetBits(m_gpio.port, m_gpio.pin);
        }
        else {
            high();//GPIO_SetBits(m_gpio.port, m_gpio.pin);
        }
    }

    inline void high(void) {
        //GPIO_SetBits(m_gpio.port, m_gpio.pin);
        m_gpio.port->BSRR = m_gpio.pin;
    }

    inline void low(void) {
        //GPIO_ResetBits(m_gpio.port, m_gpio.pin);
        m_gpio.port->BRR = m_gpio.pin;
    }

    inline bool ishigh()
    {
        // if ((GPIOx->IDR & GPIO_Pin) != (uint32_t)Bit_RESET)	GPIO_ReadInputDataBit(m_gpio.port, m_gpio.pin)==Bit_SET
        if( m_gpio.port->IDR & m_gpio.pin) {
            return true;
        }
        else {
            return false;
        }
    }

    inline bool islow() {
        // if ((GPIOx->IDR & GPIO_Pin) != (uint32_t)Bit_RESET)	GPIO_ReadInputDataBit(m_gpio.port, m_gpio.pin)==Bit_SET
        if( m_gpio.port->IDR & m_gpio.pin) {
            return false;
        }
        else {
            return true;
        }
    }

    void toggle(uint32_t t=1000,bool bLoop=true) {
        while(bLoop) {
            high();
            for(int i=0; i<t; i++);
            low();
            for(int i=0; i<t; i++);
        }
    }
};

#endif
