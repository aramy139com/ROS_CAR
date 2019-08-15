#include "Gpio.h"


periph_PORT PERIPH_PORT[]=
{
    RCC_APB2Periph_GPIOA,GPIOA,
    RCC_APB2Periph_GPIOB,GPIOB,
    RCC_APB2Periph_GPIOC,GPIOC,
    RCC_APB2Periph_GPIOD,GPIOD,
    RCC_APB2Periph_GPIOE,GPIOE,
    RCC_APB2Periph_GPIOF,GPIOF,
    RCC_APB2Periph_GPIOG,GPIOG,
};

Gpio::Gpio(PORT_INDEX 	indexPort,
           uint16_t 		      indexPin, //¹Ü½Å0~15 GPIO_Pin_0~GPIO_Pin_15
           GPIOMode_TypeDef		p_mode,
           GPIOSpeed_TypeDef		p_speed) {

    initialize(PERIPH_PORT[indexPort].p_periph,
               PERIPH_PORT[indexPort].p_port,
               (uint16_t)1<<indexPin,
               p_mode,
               p_speed
              );
}

Gpio::Gpio( uint32_t		p_periph,
            GPIO_TypeDef*		p_port,
            uint16_t			p_pins,   
            GPIOMode_TypeDef		p_mode,
            GPIOSpeed_TypeDef		p_speed) {

    initialize(p_periph,
               p_port,
               p_pins,    
               p_mode,
               p_speed
              );
}

void Gpio::initialize( GPIOMode_TypeDef   p_mode) {
    if(m_gpio.mode==p_mode)return;
    initialize(m_gpio.periph,
               m_gpio.port,
               m_gpio.pin,    
               p_mode,
               m_gpio.speed
              );
    m_gpio.mode=p_mode;
}

void Gpio::initialize( uint32_t    		p_periph,
                       GPIO_TypeDef*    		p_port,
                       uint16_t    		p_pins,    
                       GPIOMode_TypeDef   	p_mode,
                       GPIOSpeed_TypeDef  	p_speed) {
    m_gpio.periph = p_periph;
    m_gpio.port = p_port;
    m_gpio.pin = p_pins;
    m_gpio.mode=p_mode;
    m_gpio.speed=p_speed;

    GPIO_InitTypeDef tmp_InitType;
    tmp_InitType.GPIO_Pin= m_gpio.pin ;
    tmp_InitType.GPIO_Mode=m_gpio.mode;
    tmp_InitType.GPIO_Speed=m_gpio.speed;

    RCC_APB2PeriphClockCmd( m_gpio.periph, ENABLE );

    GPIO_Init( m_gpio.port ,&tmp_InitType);
}
