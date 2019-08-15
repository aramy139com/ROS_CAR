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
 
//车相关的类
class Roscar{
	private:
		//状态信息
		Mpu6050 mpu;		//传感器
		riki_msgs::Imu raw_imu_msg;								//陀螺仪、加速度、磁力计
		//灯光
		Gpio ledred;
		Gpio ledgreen;	
		//电池
		Battery batpower;		
		//轮子
		Wheel leftwheel;
		Wheel rightwheel;		
		
		uint32_t prev_update_time;		//上次命令传入时间
		double actualLineSpeed;				//实际的线速度  米/秒
		double expectLineSpeed;				//预期的线速度  米/秒  最大 +-0.25米/秒
		double actualAngleSpeed;			//实际的角速度	弧度/秒
		double expectAngleSpeed;			//预期的角速度	弧度/秒
		double headingRadians;				//小车的指向	
		double offsetmag;							//磁力计的变换值
	public:			
		Gpio ledwarn;
		Gpio beep;
		uint32_t commandTime;		//命令传入时间点
		void initialize();			//初始化各个参数
		void flushImuInfo();		//获得 mpu6050 磁力计的当前值
		void selfCheck();				//小车自检  有点问题20190812
		//编码器
		Encoder leftencode;
		Encoder rightencode;
		void flushEncodeInfo();	//更新 编码器信息
	
		//左右轮的PID
		PID leftpid;
		PID rightpid;
		float getBatPower();	//获得当前电量
		void dispDebugInfo(char *buf);			//显示小车当前状态信息		
		void moveCar();				//控制小车的运动
	
		double getActualLineSpeed(){			//获得当前实际的线速度
			return actualLineSpeed;
		}
		double getExpectLineSpeed(){			//获得当前期望的线速度
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