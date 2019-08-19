#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <errno.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <math.h>
#include "stm32f10x.h"
#include "millisecondtimer.h"

#define PI      					3.1415926
#define WHEEL_DIAMETER		0.0586    //���ӵ�ֱ�� ��λ�� 
#define WHEEL_PERIMETER  	0.1840973295 //wheel's diameter in meters  ����תһ�� ǰ���ľ��� ��
#define COUNTS_PER_REV  	47450 //wheel encoder's no of ticks per rev(gear_ratio * pulse_per_rev) ���� ��תһȦ ��������ֵ
#define CAR_WIDTHS				0.142     //С���Ŀ��
#define MAXLINESPEED			0.35			//�������ٶ� ���� ��λ ��/��
#define FIXANGLESPEED			0.0			//���ٶ�����ֵ

#define MAGNETOMETER_ISIN 1						//�Ƿ�װ�˴�����  1 ��װ��  0 δ��װ

#define IMU_PUBLISH_RATE 50 //hz
#define BAT_PUBLISH_RATE 0.2 //hz
#define COMMAND_RATE 50 //hz
#define DEBUG_RATE 1


#define K_P    2182.0 // P constant
#define K_I    1320.0 // I constant
#define K_D    -260.0 // D constant

enum Side{
	LEFT = 0,
	RIGHT = 1
}; 

//������������������������  500 Ϊ ������
#define LEFTBLIND 0
#define RIGHTBLIND 0

#define 	USE_SERIAL1
#define 	USE_SERIAL2
#define 	USE_SERIAL3
/** --------���������������-------- **/
typedef enum {
	SERIAL1 = 0,
	SERIAL2 = 1,
	SERIAL3 = 2,
	SERIAL_END = 3
}Serial_TypeDef; 

#define SERIALn							3

#define ROS_SERIAL1									USART1
#define ROS_SERIAL1_IRQ							USART1_IRQn
#define ROS_SERIAL1_CLK             RCC_APB2Periph_USART1
#define ROS_SERIAL1_GPIO_CLK        RCC_APB2Periph_GPIOA
#define ROS_SERIAL1_GPIO_PORT       GPIOA
#define ROS_SERIAL1_TX_PIN          GPIO_Pin_9
#define ROS_SERIAL1_RX_PIN          GPIO_Pin_10
#define ROS_SERIAL1_NVIC						1

#define ROS_SERIAL2									USART2
#define ROS_SERIAL2_IRQ							USART2_IRQn
#define ROS_SERIAL2_CLK             RCC_APB1Periph_USART2
#define ROS_SERIAL2_GPIO_CLK        RCC_APB2Periph_GPIOA
#define ROS_SERIAL2_GPIO_PORT      	GPIOA
#define ROS_SERIAL2_TX_PIN          GPIO_Pin_2
#define ROS_SERIAL2_RX_PIN          GPIO_Pin_3
#define ROS_SERIAL2_NVIC						2

#define ROS_SERIAL3									USART3
#define ROS_SERIAL3_IRQ							USART3_IRQn
#define ROS_SERIAL3_CLK           	RCC_APB1Periph_USART3
#define ROS_SERIAL3_GPIO_CLK       	RCC_APB2Periph_GPIOB
#define ROS_SERIAL3_GPIO_PORT      	GPIOB
#define ROS_SERIAL3_TX_PIN         	GPIO_Pin_10
#define ROS_SERIAL3_RX_PIN          GPIO_Pin_11
#define ROS_SERIAL3_NVIC						3
///////////////////////////////////////////////////////
#endif // _CONFIG_H_