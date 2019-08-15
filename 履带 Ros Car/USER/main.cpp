#include "stm32f10x.h"
#include <stdio.h>
#include "hardwareserial.h"
#include <ros.h>
#include <riki_msgs/Velocities.h>
#include <geometry_msgs/Twist.h>
#include <riki_msgs/PID.h>
#include <riki_msgs/Imu.h>
#include <riki_msgs/Battery.h>
#include <geometry_msgs/Vector3.h>
#include <ros/time.h>
#include "roscar.h"
void commandCallBack(const geometry_msgs::Twist& cmd_msg);		//命令回调函数

Roscar trackcar;																							//履带车
ros::NodeHandle  nh;
riki_msgs::Imu raw_imu_msg;							//传感器话题
riki_msgs::Velocities raw_vel_msg;			//线速度话题
riki_msgs::Battery raw_battery_msg;			//电量话题
ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallBack);		//命令处理话题
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);
ros::Publisher raw_battery_pub("battery", &raw_battery_msg);

void commandCallBack( const geometry_msgs::Twist& cmd_msg) {    //命令回调函数
    trackcar.setExpectLineSpeed(cmd_msg.linear.x);
    trackcar.setExpectAngleSpeed(cmd_msg.angular.z);
		trackcar.setCommandTime(millis());
		trackcar.getGreeLed().invert();
}

HardwareSerial serial;
int main() {
		char buf[400];
    uint32_t carCtlTime = 0,ctlMoveTime=0,publishImuTime=0,publishBatTime=0,debugTime=0;	
    SystemInit();
    initialise();
    trackcar.initialize();					//车体的初始化
		serial.begin(115200);
		delay(100);
		while(1){
			trackcar.flushImuInfo();				//刷新IMU信息 
//			trackcar.dispDebugInfo(buf);
//			serial.print("%s\n",buf);
			delay(10);
		}
		/*
		trackcar.ledwarn.low();					//等待ROS连接
		nh.initNode();
		nh.subscribe(cmd_sub);					//订阅命令话题
		nh.advertise(raw_battery_pub);	//订阅电量话题
		nh.advertise(raw_imu_pub);			//订阅6050传感器话题
		nh.advertise(raw_vel_pub);			//线速度 角速度话题
		while (!nh.connected()) {
        nh.spinOnce();
    }
    nh.loginfo("Stm32 Connected!");
//		trackcar.beep.high();
//		delay(100);
//		trackcar.beep.low();
		trackcar.ledwarn.high();				//ROS连接成功
		
    while(1) {
        //定时读取imu的信息
        if((millis()-publishImuTime)>=(1000/IMU_PUBLISH_RATE)) {
            publishImuTime=millis();
						trackcar.flushImuInfo();				//刷新IMU信息 
						trackcar.flushEncodeInfo();			//刷新码盘信息
					
						raw_imu_msg=trackcar.getRaw_imu_msg();
						raw_imu_pub.publish(&raw_imu_msg);
						raw_vel_msg.linear_x=trackcar.getActualLineSpeed();
						raw_vel_msg.linear_y=0;
						raw_vel_msg.angular_z=trackcar.getActualAngleSpeed();
						raw_vel_pub.publish(&raw_vel_msg);
        }
				 //车体的速度信息 和控制
        if((millis()-ctlMoveTime)>=(1000/COMMAND_RATE)) {
            ctlMoveTime=millis();						
						trackcar.moveCar();
						if(millis()-trackcar.getCommandTime()>=400){
							trackcar.setExpectAngleSpeed(0.0);
							trackcar.setExpectLineSpeed(0.0);
						}
						
        }
				//读取电池信息
				if((millis()-publishBatTime)>=(1000/BAT_PUBLISH_RATE)) {
						publishBatTime=millis();
            raw_battery_msg.battery=trackcar.getBatPower();					//读取电量 
						if(raw_battery_msg.battery <7.2) {			//电量低警告
							trackcar.ledwarn.low();
						}else	if(raw_battery_msg.battery <7.8){	//充电提示
							trackcar.ledwarn.invert();			
						}
						raw_battery_pub.publish(&raw_battery_msg);
        }
				//显示调试信息
				if((millis()-debugTime)>=(1000/DEBUG_RATE)) {
						debugTime=millis();
            //使用串口 打印当前系统状态信息
						trackcar.dispDebugInfo(buf);
						nh.loginfo(buf);
						trackcar.getLedRed().invert();
        }
				nh.spinOnce();
    }
		*/
}
