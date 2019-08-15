#ifndef _ROSCAR_H_
#define _ROSLCAR_H_
#include "config.h"
#include "Gpio.h"
#include "mpu6050.h"
#include "encoder.h"
#include "battery.h"
#include "wheel.h"
#include "PID.h"
#include <riki_msgs/Imu.h>
#include <riki_msgs/Velocities.h>
 
//����ص���
class Roscar{
	private:
		//״̬��Ϣ
		Mpu6050 mpu;		//������
		riki_msgs::Imu raw_imu_msg;								//�����ǡ����ٶȡ�������
		//�ƹ�
		Gpio ledred;
		Gpio ledgreen;	
		//���
		Battery batpower;		
		//����
		Wheel leftwheel;
		Wheel rightwheel;		
		
		uint32_t prev_update_time;		//�ϴ������ʱ��
		double actualLineSpeed;				//ʵ�ʵ����ٶ�  ��/��
		double expectLineSpeed;				//Ԥ�ڵ����ٶ�  ��/��  ��� +-0.25��/��
		double actualAngleSpeed;			//ʵ�ʵĽ��ٶ�	����/��
		double expectAngleSpeed;			//Ԥ�ڵĽ��ٶ�	����/��
		double headingRadians;				//С����ָ��	
		double offsetmag;							//�����Ƶı任ֵ
	public:			
		Gpio ledwarn;
		Gpio beep;
		uint32_t commandTime;		//�����ʱ���
		void initialize();			//��ʼ����������
		void flushImuInfo();		//��� mpu6050 �����Ƶĵ�ǰֵ
		void selfCheck();				//С���Լ�  �е�����20190812
		//������
		Encoder leftencode;
		Encoder rightencode;
		void flushEncodeInfo();	//���� ��������Ϣ
	
		//�����ֵ�PID
		PID leftpid;
		PID rightpid;
		float getBatPower();	//��õ�ǰ����
		void dispDebugInfo(char *buf);			//��ʾС����ǰ״̬��Ϣ		
		void moveCar();				//����С�����˶�
	
		double getActualLineSpeed(){			//��õ�ǰʵ�ʵ����ٶ�
			return actualLineSpeed;
		}
		double getExpectLineSpeed(){			//��õ�ǰ���������ٶ�
			return expectLineSpeed;
		}
		double getActualAngleSpeed(){
			return actualAngleSpeed;
		}
		double getExpectAngleSpeed(){
			return expectAngleSpeed;
		}
		void setExpectLineSpeed(double linespeed){
			expectLineSpeed=linespeed;
		}
		void setExpectAngleSpeed(double anglespeed){
			expectAngleSpeed=anglespeed;
		}
		riki_msgs::Imu getRaw_imu_msg(){
			return raw_imu_msg;
		}
		PID getLeftpid(){return leftpid;}
		PID getRightpid(){return rightpid;}
		double getHeadingRadians(){
			return headingRadians;
		}
		void setCommandTime(uint32_t cmdintime){
			commandTime=cmdintime;
		}
		uint32_t getCommandTime(){
			return commandTime;
		}
		Gpio getGreeLed(){
			return ledgreen;
		}
		Gpio getLedRed(){
			return ledred;
		}			
};

#endif // _ROSLCAR_H_