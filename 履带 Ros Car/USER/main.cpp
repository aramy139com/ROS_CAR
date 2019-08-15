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
void commandCallBack(const geometry_msgs::Twist& cmd_msg);		//����ص�����

Roscar trackcar;																							//�Ĵ���
ros::NodeHandle  nh;
riki_msgs::Imu raw_imu_msg;							//����������
riki_msgs::Velocities raw_vel_msg;			//���ٶȻ���
riki_msgs::Battery raw_battery_msg;			//��������
ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallBack);		//�������
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);
ros::Publisher raw_battery_pub("battery", &raw_battery_msg);

void commandCallBack( const geometry_msgs::Twist& cmd_msg) {    //����ص�����
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
    trackcar.initialize();					//����ĳ�ʼ��
		serial.begin(115200);
		delay(100);
		while(1){
			trackcar.flushImuInfo();				//ˢ��IMU��Ϣ 
//			trackcar.dispDebugInfo(buf);
//			serial.print("%s\n",buf);
			delay(10);
		}
		/*
		trackcar.ledwarn.low();					//�ȴ�ROS����
		nh.initNode();
		nh.subscribe(cmd_sub);					//���������
		nh.advertise(raw_battery_pub);	//���ĵ�������
		nh.advertise(raw_imu_pub);			//����6050����������
		nh.advertise(raw_vel_pub);			//���ٶ� ���ٶȻ���
		while (!nh.connected()) {
        nh.spinOnce();
    }
    nh.loginfo("Stm32 Connected!");
//		trackcar.beep.high();
//		delay(100);
//		trackcar.beep.low();
		trackcar.ledwarn.high();				//ROS���ӳɹ�
		
    while(1) {
        //��ʱ��ȡimu����Ϣ
        if((millis()-publishImuTime)>=(1000/IMU_PUBLISH_RATE)) {
            publishImuTime=millis();
						trackcar.flushImuInfo();				//ˢ��IMU��Ϣ 
						trackcar.flushEncodeInfo();			//ˢ��������Ϣ
					
						raw_imu_msg=trackcar.getRaw_imu_msg();
						raw_imu_pub.publish(&raw_imu_msg);
						raw_vel_msg.linear_x=trackcar.getActualLineSpeed();
						raw_vel_msg.linear_y=0;
						raw_vel_msg.angular_z=trackcar.getActualAngleSpeed();
						raw_vel_pub.publish(&raw_vel_msg);
        }
				 //������ٶ���Ϣ �Ϳ���
        if((millis()-ctlMoveTime)>=(1000/COMMAND_RATE)) {
            ctlMoveTime=millis();						
						trackcar.moveCar();
						if(millis()-trackcar.getCommandTime()>=400){
							trackcar.setExpectAngleSpeed(0.0);
							trackcar.setExpectLineSpeed(0.0);
						}
						
        }
				//��ȡ�����Ϣ
				if((millis()-publishBatTime)>=(1000/BAT_PUBLISH_RATE)) {
						publishBatTime=millis();
            raw_battery_msg.battery=trackcar.getBatPower();					//��ȡ���� 
						if(raw_battery_msg.battery <7.2) {			//�����;���
							trackcar.ledwarn.low();
						}else	if(raw_battery_msg.battery <7.8){	//�����ʾ
							trackcar.ledwarn.invert();			
						}
						raw_battery_pub.publish(&raw_battery_msg);
        }
				//��ʾ������Ϣ
				if((millis()-debugTime)>=(1000/DEBUG_RATE)) {
						debugTime=millis();
            //ʹ�ô��� ��ӡ��ǰϵͳ״̬��Ϣ
						trackcar.dispDebugInfo(buf);
						nh.loginfo(buf);
						trackcar.getLedRed().invert();
        }
				nh.spinOnce();
    }
		*/
}
