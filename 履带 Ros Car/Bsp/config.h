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
#define WHEEL_DIAMETER		0.0586    //轮子的直径 单位米 
#define WHEEL_PERIMETER  	0.1840973295 //wheel's diameter in meters  轮子转一周 前进的距离 米
#define COUNTS_PER_REV  	47450 //wheel encoder's no of ticks per rev(gear_ratio * pulse_per_rev) 轮子 旋转一圈 编码器的值
#define CAR_WIDTHS				0.142     //小车的宽度
#define MAXLINESPEED			0.35			//最大的线速度 上限 单位 米/秒
#define FIXANGLESPEED			0.0			//角速度修正值

#define MAGNETOMETER_ISIN 1						//是否安装了磁力计  1 安装了  0 未安装

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

//电机不够给力，添加死区控制  500 为 启动点
#define LEFTBLIND 0
#define RIGHTBLIND 0

#define 	USE_SERIAL1
#define 	USE_SERIAL2
#define 	USE_SERIAL3
/** --------串口配置相关内容-------- **/
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