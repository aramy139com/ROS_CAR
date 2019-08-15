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

Roscar trackcar;									//履带车  性能原因，开不了快车，最高到 0.25m/s


/*
ros::NodeHandle  nh;
riki_msgs::Imu raw_imu_msg;
riki_msgs::Velocities raw_vel_msg;
riki_msgs::Battery raw_battery_msg;

//命令回调函数
void command_callback( const geometry_msgs::Twist& cmd_msg) {
    trackcar.commandTime=millis();				//标记命令发起时间点
    trackcar.setExpectAngleSpeed(cmd_msg.angular.z);
    trackcar.setExpectLineSpeed(cmd_msg.linear.x);
    trackcar.cmdWarn();
}
//void pid_callback( const riki_msgs::PID& pid) {
//    trackcar.getLeftpid().updateConstants(pid.p, pid.i, pid.d);
//    trackcar.getRightpid().updateConstants(pid.p, pid.i, pid.d);
//}


//ros::Subscriber<riki_msgs::PID> pid_sub("pid", pid_callback);
ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", command_callback);
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);
ros::Publisher raw_battery_pub("battery", &raw_battery_msg);


int main(void) {
    uint32_t carCtlTime = 0,ctlMoveTime=0,publishImuTime=0,publishBatTime=0,debugTime=0;
    char buffer[300];
    SystemInit();
    initialise();
    trackcar.initialize();

    nh.initNode();
    nh.advertise(raw_vel_pub);
    nh.advertise(raw_imu_pub);
    nh.advertise(raw_battery_pub);
    nh.subscribe(cmd_sub);
//		nh.subscribe(pid_sub);

    while (!nh.connected()) {
        nh.spinOnce();
    }
    nh.loginfo("Stm32 Connected!");
    while(1) {
        if( (millis()-carCtlTime)>15) {			//控制小车
            carCtlTime=millis();
            trackcar.controlCar();
        }
        if ((millis() - trackcar.commandTime) >= 400) {
            trackcar.setExpectAngleSpeed(0.0);
            trackcar.setExpectLineSpeed(0.0);
        }
        ///////////////////////////////////////////
        if((millis()-ctlMoveTime)>=(1000/COMMAND_RATE)) {		//上送速度信息,控制命令
            ctlMoveTime=millis();
            //上传小车 线速度 和角速度
            raw_vel_msg.linear_x=trackcar.getActualLineSpeed();
            raw_vel_msg.linear_y=0;
            raw_vel_msg.angular_z=trackcar.getActualAngleSpeed();
            raw_vel_pub.publish(&raw_vel_msg);
        }
        if((millis()-publishImuTime)>=(1000/IMU_PUBLISH_RATE)) {		//上送姿态传感器信息
            publishImuTime=millis();
            raw_imu_msg=trackcar.getRaw_imu_msg();
            raw_imu_pub.publish(&raw_imu_msg);
        }
        if((millis()-publishBatTime)>=(1000/BAT_PUBLISH_RATE)) {		//上送电量信息
            publishBatTime=millis();
            raw_battery_msg.battery=trackcar.getBatPower();
            if(raw_battery_msg.battery<7.5) trackcar.batWarn();
            raw_battery_pub.publish(&raw_battery_msg);
        }
        if((millis()-debugTime)>=(1000/DEBUG_RATE)) {		//调试信息
            debugTime=millis();
            trackcar.dispDebugInfo(buffer);
            nh.loginfo(buffer);
        }
        nh.spinOnce();
    }
}


*/

/*
HardwareSerial serial;
uint32_t prev_update_time;
int main() {
    double offtime,leftspeed,rightspeed;								//流逝的时间  车轮的速度 米/秒
    uint32_t curr_time,offtimems;
    char ch;
    int pwm=0;
    Wheel wheell,wheelr;
    Encoder encodel,encoder;
    SystemInit();
    initialise();
    serial.begin(57600);
    wheell.initialize(LEFT);
    wheelr.initialize(RIGHT);
    encodel.initialize(LEFT);
    encoder.initialize(RIGHT);
    while(1) {
        delay(50);
        //serial.print("hello\n");
        while(serial.available()) {
            ch=serial.read();
            if(ch=='A') pwm+=10;
            if(ch=='B') pwm-=10;
            if(ch=='R') pwm=0;
            serial.print("%c\t%d\n",ch,pwm);
        }
        wheell.setPWM(pwm);
        wheelr.setPWM(pwm);
        //获得码盘信息
        encodel.setEncoder();
        encoder.setEncoder();
        curr_time=millis();
        offtimems=curr_time>prev_update_time?curr_time-prev_update_time:(4200000-prev_update_time)+curr_time;
        prev_update_time=curr_time;
        offtime=(double)offtimems/1000.00;			//秒
        //计算当前的小车的速度和角速度
        leftspeed=WHEEL_PERIMETER*(double)encodel.getEnValue()/COUNTS_PER_REV/offtime;			//米/秒
        rightspeed=WHEEL_PERIMETER*(double)encoder.getEnValue()/COUNTS_PER_REV/offtime;
        serial.print("%d:[%.4f,%.4f]\n",pwm,leftspeed,rightspeed);
    }
}
*/

//20190423整定PID
HardwareSerial serial;
int main() {   
		uint32_t carCtlTime = 0	;
		uint8_t serleng,i;
		float sp,si,sd,sspeed;
		SerTran info;
    SystemInit();
    initialise();
    serial.begin(57600);
    trackcar.initialize();
    while(1) {
       if( (millis()-carCtlTime)>20) {			//控制小车
            carCtlTime=millis();
            trackcar.controlCar();
        }
			 serleng=serial.available();
			 if(serleng>=16){		//成功地获取到了一组数据
				 //解析上位机数据
				 for(i=0;i<4;i++){
					 info.s[i]=serial.read();
				 }
				 sp=info.f;
				 for(i=0;i<4;i++){
					 info.s[i]=serial.read();
				 }
				 si=info.f;
				 for(i=0;i<4;i++){
					 info.s[i]=serial.read();
				 }
				 sd=info.f;
				 for(i=0;i<4;i++){
					 info.s[i]=serial.read();
				 }
				 sspeed=info.f;
				 trackcar.setExpectAngleSpeed(sspeed);			//设置期望速度
				 trackcar.leftpid.updateConstants(sp,si,sd);
				 trackcar.cmdWarn();
			 }
    }
}
