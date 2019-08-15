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
void commandCallBack(const geometry_msgs::Twist& cmd_msg);		//ÃüÁî»Øµ÷º¯Êı

Roscar trackcar;																							//ÂÄ´ø³
ros::NodeHandle  nh;
riki_msgs::Imu raw_imu_msg;							//´«¸ĞÆ÷»°Ìâ
riki_msgs::Velocities raw_vel_msg;			//ÏßËÙ¶È»°Ìâ
riki_msgs::Battery raw_battery_msg;			//µçÁ¿»°Ìâ
ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallBack);		//ÃüÁî´¦Àí»°Ìâ
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);
ros::Publisher raw_battery_pub("battery", &raw_battery_msg);

void commandCallBack( const geometry_msgs::Twist& cmd_msg) {    //ÃüÁî»Øµ÷º¯Êı
    trackcar.setExpectLineSpeed(cmd_msg.linear.x);
    trackcar.setExpectAngleSpeed(cmd_msg.angular.z);
		trackcar.setCommandTime(millis());
		trackcar.getGreeLed().invert();
}

int main() {
		char buf[400];
    uint32_t carCtlTime = 0,ctlMoveTime=0,publishImuTime=0,publishBatTime=0,debugTime=0;	
    SystemInit();
    initialise();
    trackcar.initialize();					//³µÌåµÄ³õÊ¼»¯
		
		trackcar.ledwarn.low();					//µÈ´ıROSÁ¬½Ó
		nh.initNode();
		nh.subscribe(cmd_sub);					//¶©ÔÄÃüÁî»°Ìâ
		nh.advertise(raw_battery_pub);	//¶©ÔÄµçÁ¿»°Ìâ
		nh.advertise(raw_imu_pub);			//¶©ÔÄ6050´«¸ĞÆ÷»°Ìâ
		nh.advertise(raw_vel_pub);			//ÏßËÙ¶È ½ÇËÙ¶È»°Ìâ
		while (!nh.connected()) {
        nh.spinOnce();
    }
    nh.loginfo("Stm32 Connected!");
		trackcar.beep.high();
		delay(100);
		trackcar.beep.low();
		trackcar.ledwarn.high();				//ROSÁ¬½Ó³É¹¦
		
    while(1) {
        //¶¨Ê±¶ÁÈ¡imuµÄĞÅÏ¢
        if((millis()-publishImuTime)>=(1000/IMU_PUBLISH_RATE)) {
            publishImuTime=millis();
						trackcar.flushImuInfo();				//Ë¢ĞÂIMUĞÅÏ¢ 
						trackcar.flushEncodeInfo();			//Ë¢ĞÂÂëÅÌĞÅÏ¢
					
						raw_imu_msg=trackcar.getRaw_imu_msg();
						raw_imu_pub.publish(&raw_imu_msg);
						raw_vel_msg.linear_x=trackcar.getActualLineSpeed();
						raw_vel_msg.linear_y=0;
						raw_vel_msg.angular_z=trackcar.getActualAngleSpeed();
						raw_vel_pub.publish(&raw_vel_msg);
        }
				 //³µÌåµÄËÙ¶ÈĞÅÏ¢ ºÍ¿ØÖÆ
        if((millis()-ctlMoveTime)>=(1000/COMMAND_RATE)) {
            ctlMoveTime=millis();						
						trackcar.moveCar();
						if(millis()-trackcar.getCommandTime()>=400){
							trackcar.setExpectAngleSpeed(0.0);
							trackcar.setExpectLineSpeed(0.0);
						}
						
        }
				//¶ÁÈ¡µç³ØĞÅÏ¢
				if((millis()-publishBatTime)>=(1000/BAT_PUBLISH_RATE)) {
						publishBatTime=millis();
            raw_battery_msg.battery=trackcar.getBatPower();					//¶ÁÈ¡µçÁ¿ 
						if(raw_battery_msg.battery <7.2) {			//µçÁ¿µÍ¾¯¸æ
							trackcar.ledwarn.low();
						}else	if(raw_battery_msg.battery <7.8){	//³äµçÌáÊ¾
							trackcar.ledwarn.invert();			
						}
						raw_battery_pub.publish(&raw_battery_msg);
        }
				//ÏÔÊ¾µ÷ÊÔĞÅÏ¢
				if((millis()-debugTime)>=(1000/DEBUG_RATE)) {
						debugTime=millis();
            //Ê¹ÓÃ´®¿Ú ´òÓ¡µ±Ç°ÏµÍ³×´Ì¬ĞÅÏ¢
						trackcar.dispDebugInfo(buf);
						nh.loginfo(buf);
						trackcar.getLedRed().invert();
        }
				nh.spinOnce();
    }
}
