//filename roscar.cpp
#include "roscar.h"
#include "config.h"
#include "millisecondtimer.h"
#include "hardwareserial.h"
#include "StFlash.h"

void Roscar::initialize(){	
	float fbuf[7];
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);			//灯
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);	
	ledred= Gpio(PC,13);
	ledgreen= Gpio(PC,14);
	ledwarn=Gpio(PA,15);
	beep=Gpio(PB,3);
	ledred.high();							//关闭所有灯
	ledgreen.high();				
	ledwarn.high();
	beep.low();									//关闭蜂鸣器
	
	//初始化传感器
	mpu.setAcc_fsr(ACC16G);			//加速度传感器 量程
	mpu.setGyrofsr(GYRO2000);		//陀螺仪传感器 量程  角速度
	mpu.mpu6050_init();
	mpu.setOrientation(1,-1,-1);		//设置传感器旋转方向
	fbuf[0]=-31.2280655;
	fbuf[1]=-17.0690289;
	fbuf[2]=13.7129383;
	fbuf[3]=0;
	fbuf[4]=0;
	fbuf[5]=0;
	mpu.setOffset(fbuf);					//6050偏移量
	mpu.setMegorientationOffset(-26.6054993,-97.7064209,-95.2752228);
	
	if( MAGNETOMETER_ISIN ){			//磁力计的初始化
		mpu.hmc5883l_init();
	}
	leftwheel.initialize(LEFT,1999,359);		//轮子初始化
	rightwheel.initialize(RIGHT,1999,359);
		
	//电池初始化
	batpower.initialize();
	
	//selfCheck();				//小车自检
	//pid初始化
	leftpid.initialize(-1999, 1999, K_P, K_I, K_D);
	rightpid.initialize(-1999, 1999, K_P, K_I, K_D);	
	
	actualLineSpeed=0.0;				//实际的线速度  米/秒
	expectLineSpeed=0.0;				//预期的线速度  米/秒
	actualAngleSpeed=0.0;				//实际的角速度	弧度/秒
	expectAngleSpeed=0.0;				//预期的角速度	弧度/秒
	
	leftencode.initialize(LEFT);		//编码器
	rightencode.initialize(RIGHT);	
	prev_update_time=millis();
	
	if( MAGNETOMETER_ISIN ){
		mpu.getAllMagnetometer(fbuf,1);
		raw_imu_msg.magnetic_field.x=fbuf[0];
		raw_imu_msg.magnetic_field.z=fbuf[1];
		raw_imu_msg.magnetic_field.y=fbuf[2];		
		//计算小车的指向  假定小车与地面平行 只计算  X Y
		headingRadians=atan2(fbuf[2],fbuf[0]);
		if(headingRadians<0) headingRadians+=2*PI;
	}
}

//获得加速度、陀螺仪传感器的值
void Roscar::flushImuInfo(){
	float fbuf[3];
	double oldheading=headingRadians;
	uint8_t* buf;
	//处理加速度
	mpu.getAccAllVal(fbuf,G);	
	//mpu.getAccAllVal(fbuf,ORGI);
	raw_imu_msg.linear_acceleration.x=fbuf[0];
	raw_imu_msg.linear_acceleration.y=fbuf[1];
	raw_imu_msg.linear_acceleration.z=fbuf[2];
	//处理角速度
	mpu.getGyroAllVal(fbuf,RAD);
	//mpu.getGyroAllVal(fbuf,ORGI);
	raw_imu_msg.angular_velocity.x=fbuf[0];
	raw_imu_msg.angular_velocity.y=fbuf[1];
	raw_imu_msg.angular_velocity.z=fbuf[2];
	//处理磁力计
	if( MAGNETOMETER_ISIN ){
		mpu.getAllMagnetometer(fbuf,1);
		raw_imu_msg.magnetic_field.x=fbuf[0];
		raw_imu_msg.magnetic_field.z=fbuf[1];
		raw_imu_msg.magnetic_field.y=fbuf[2];		
		//计算小车的指向  假定小车与地面平行 只计算  X Y
		headingRadians=atan2(fbuf[2],fbuf[0]);
		if(headingRadians<0) headingRadians+=2*PI;
		offsetmag=oldheading-headingRadians;
		if(offsetmag<PI) offsetmag+=2*PI;
		if(offsetmag>PI) offsetmag-=2*PI;
		offsetmag=offsetmag/((double)1.0/IMU_PUBLISH_RATE);
	}
}
//更新编码器信息 
void Roscar::flushEncodeInfo(){
	uint32_t curr_time,offtimems;
	double offtime,leftspeed,rightspeed;								//流逝的时间  车轮的速度 米/秒
	//获得码盘信息
	leftencode.setEncoder();
	rightencode.setEncoder();	
	//serial.print("enc:[%d\t%d]    [%d\t%d]\n",leftencode.getEnValue(),rightencode.getEnValue(),leftencode.getTotleValue(),rightencode.getTotleValue());
	curr_time=millis();							//当前的时间
	offtimems=curr_time>prev_update_time?curr_time-prev_update_time:(4200000-prev_update_time)+curr_time;
	prev_update_time=curr_time;			//记录旧的时间点
	offtime=(double)offtimems/1000.00;			//秒
	
	//计算当前的小车的速度和角速度
	leftspeed=WHEEL_PERIMETER*(double)leftencode.getEnValue()/COUNTS_PER_REV/offtime;			//单位：米/秒
	rightspeed=WHEEL_PERIMETER*(double)rightencode.getEnValue()/COUNTS_PER_REV/offtime;
	leftwheel.setSpeed(leftspeed);						//将计算出的轮子速度，写回轮子
	rightwheel.setSpeed(rightspeed);
	actualLineSpeed=(leftspeed+rightspeed)/2;									//小车当前的 线速度
	
	//计算当前小车的角速度  角速度有三个方法计算了 
	//1 通过轮子速度差  放弃
	//2 通过mpu6050的角速度
	//3 通过磁力计计算角度的偏差
	actualAngleSpeed=actualAngleSpeed*0.5+raw_imu_msg.angular_velocity.z*0.20+offsetmag*0.3;
}

//获得电池电量信息
float Roscar::getBatPower(){
	return batpower.get_volt();
}

//显示小车当前状态信息
void Roscar::dispDebugInfo(char *buf){
	//sprintf(buf,"\nIMUZ:%.4fexp:[%.4f,%.4f],act:[%.4f,%.4f]\tbat=%.1f",raw_imu_msg.angular_velocity.z,expectLineSpeed,expectAngleSpeed,actualLineSpeed,actualAngleSpeed,getBatPower());
	//sprintf(buf,"\nIMUZ:%.3f, exp:[%.4f,%.4f],act:[%.4f,%.4f]\tbat=%.1f",raw_imu_msg.angular_velocity.z,expectLineSpeed,expectAngleSpeed,actualLineSpeed,actualAngleSpeed,getBatPower());
	//显示 imu相关信息   9轴信息
	
	//sprintf(buf,"IMU:[%.4f,%.4f,%.4f],[%.4f,%.4f,%.4f],[%.4f,%.4f,%.4f]\n",raw_imu_msg.linear_acceleration.x,raw_imu_msg.linear_acceleration.y,raw_imu_msg.linear_acceleration.z,raw_imu_msg.angular_velocity.x,raw_imu_msg.angular_velocity.y,raw_imu_msg.angular_velocity.z,raw_imu_msg.magnetic_field.x,raw_imu_msg.magnetic_field.y,raw_imu_msg.magnetic_field.z);
	//sprintf(buf,"%ld:[%.4f,%.4f,%.4f]\t%.2f\n",millis(),raw_imu_msg.magnetic_field.x,raw_imu_msg.magnetic_field.y,raw_imu_msg.magnetic_field.z,atan2(raw_imu_msg.magnetic_field.y,raw_imu_msg.magnetic_field.x)*(180/PI)+180);
	//sprintf(buf,"%ld:[%.4f,%.4f,%.4f]\t%.2f\n",millis(),raw_imu_msg.magnetic_field.x,raw_imu_msg.magnetic_field.y,raw_imu_msg.magnetic_field.z,headingRadians*(180/PI));
	//显示速度相关信息
	sprintf(buf,"EX:[%.2f,%.2f] SP_LR[%.3f,%.3f]\nACSP[%.2f,%.5f]  RAW[%.4f,%.4f]\n",expectLineSpeed,expectAngleSpeed,leftwheel.getSpeed(),rightwheel.getSpeed(),
			actualLineSpeed,actualAngleSpeed,	raw_imu_msg.angular_velocity.z,offsetmag);
}

//小车的运动信息
void Roscar::moveCar(){
	int16_t lpwm,rpwm;
	double l_expspeed,r_exspeed,angspeed;				//左右轮期望速度
	//小车通过 PID方式计算左右两个轮子的转速，旋转部分 通过 磁力计来消除
	//先不管旋转问题，将速度上限设置为 指定的速度上限。直接计算左右轮的 pwm值
	if( expectLineSpeed>MAXLINESPEED) expectLineSpeed=MAXLINESPEED;
	if( expectLineSpeed<-MAXLINESPEED) expectLineSpeed=-MAXLINESPEED;
	
	l_expspeed=r_exspeed=expectLineSpeed;			//期望的线速度
	//解决纠正带来的自激抖动
	angspeed=expectAngleSpeed-actualAngleSpeed;
	
	//角速度对应的线速度的计算
	if( angspeed<-1.2 || angspeed>1.2 ){			//判断为抖动的开始
		angspeed=expectAngleSpeed*CAR_WIDTHS;		//当抖动发生时，拒绝纠正抖动引起的偏差
	}	else{		
		angspeed=(expectAngleSpeed+expectAngleSpeed-actualAngleSpeed)*CAR_WIDTHS;			//角速度对应的线速度
	}
	l_expspeed-=angspeed;
	r_exspeed+=angspeed;

	//通过pid计算出 需要设置的PWM值
	lpwm=(int16_t)leftpid.computeLoc(l_expspeed,leftwheel.getSpeed());
	rpwm=(int16_t)rightpid.computeLoc(r_exspeed,rightwheel.getSpeed());
	leftwheel.setPWM(lpwm);									//动力输出
	rightwheel.setPWM(rpwm);
}



//小车自检程序
//逻辑：从flash中读取 Mpu6050  磁力计的偏移量
//然后读取 200ms mpu6050数据，求平均，看偏差是否过大。
//如果过大，则重新校准 imu和磁力计
//如果OK 则自检完成
void Roscar::selfCheck(){
	float fbuf[8],avgbuf[5],mag[5];			// X Z Y
	uint16_t i;	
	ledred.low();
	delay(100);
	ledred.high();	
	readFlashData(fbuf,mag);//读取flash数据
	//serial.print("read flash:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",fbuf[0],fbuf[1],fbuf[2],mag[0],mag[1],mag[2]);
	mpu.setOffset(fbuf);			//设置mpu的校正值
	mpu.setMegorientationOffset(mag[0],mag[1],mag[2]);	//写入磁力计的校正值
	
//	if(!isForceCheckSelf()){			//查看是否需要强制检查，如果不需要就直接退出
//		ledred.high();
//		return;
//	}
//	//检查一下6050是否还存在严重偏置
//	for(i=0;i<80;i++){
//		mpu.getGyroAllVal(fbuf,RAD);		//读取角速度
//		avgbuf[0]+=fbuf[0];
//		avgbuf[1]+=fbuf[1];
//		avgbuf[2]+=fbuf[2];
//		//serial.print("mpu:[%.2f,%.2f,%.2f]  [%.2f,%.2f,%.2f]\n",avgbuf[0],avgbuf[1],avgbuf[2],fbuf[0],fbuf[1],fbuf[2]);
//		delay(5);
//		ledred.invert();
//	}
	
//	serial.print("isCheck:%d,%.2f,%.2f,%.2f\n",isForceCheckSelf(),avgbuf[0]/80.0,avgbuf[1]/80.0,avgbuf[2]/80.0);
//	serial.print("battle=%f\n",batpower.get_volt());
//	if(isForceCheckSelf() || avgbuf[0]/80.0>0.15 || avgbuf[1]/80.0 >0.15 ||	avgbuf[2]/80.0>0.15 || avgbuf[0]/80.0<-0.15 || avgbuf[1]/80.0 <-0.15 ||	avgbuf[2]/80.0<-0.15 ){
//		//以0.1 为界，角速度超过0.1 则需要重新校正
//		//serial.print("rsret imu\n");
//		checkIMU(fbuf,fbuf+3);
//		writeFlashData(fbuf,fbuf+3);
//		mpu.setOffset(fbuf);			//设置mpu的校正值
//		mpu.setMegorientationOffset(mag[0],mag[1],mag[2]);	//写入磁力计的校正值
//	}
}

//通过电池口 的电量 判断 是否需要强制自检
bool Roscar::isForceCheckSelf(){
	return batpower.get_volt()>13?true:false;
}

//读取6050 和磁力计的偏置
void Roscar::readFlashData(float *imuoffset,float *magoffset){
	char charbuf[25];
	SerTran tmp;
	STFLASH flash;	
	uint8_t  i,j;
	//从flash里读取预先写入的 数据  6为float数据，共24个字节
	flash.readBuf(charbuf,24);
	for(i=0;i<3;i++){
		for(j=0;j<4;j++){
			tmp.s[j]=charbuf[i*4+j];
		}
		imuoffset[i]=tmp.f;
	}
	for(i=3;i<6;i++){
		for(j=0;j<4;j++){
			tmp.s[j]=charbuf[i*4+j];
		}
		magoffset[i-3]=tmp.f;
	}
}

//写入6050 和磁力计的偏置
void Roscar::writeFlashData(float *imuoffset,float *magoffset){
	char charbuf[25];
	SerTran tmp;
	STFLASH flash;	
	uint8_t  i,j;
	for(i=0;i<3;i++){
		tmp.f=imuoffset[i];
		for(j=0;j<4;j++){
			charbuf[i*4+j]=tmp.s[j];
		}
	}
	for(i=3;i<6;i++){
		tmp.f=magoffset[i-3];
		for(j=0;j<4;j++){
			charbuf[i*4+j]=tmp.s[j];
		}
	}
	//写入flash
	flash.writeBuf(charbuf,24);
}


//自检实现部分
void Roscar::checkIMU(float *imuoffset,float *magoffset){
	uint16_t i;
	float xmin=0,xmax=0,ymin=0,ymax=0,zmin=0,zmax=0,avgbuf[6],fbuf[3];
	ledgreen.low();				//绿灯亮起，准备校正，此时不要动小车
	for(i=0;i<6;i++) avgbuf[i]=0.0;
	mpu.setOffset(avgbuf);			//重置修正值，设置为0
	
	beep.high();
	for(i=0;i<1000;i++){
			mpu.getGyroAllVal(fbuf,ORGI);		//读取角速度  用原始值
			avgbuf[0]+=fbuf[0]/1000.0;
			avgbuf[1]+=fbuf[1]/1000.0;
			avgbuf[2]+=fbuf[2]/1000.0;
			delay(10);
			if(i%10) ledgreen.invert();
			if(i>5) beep.low();
			if(i>950) beep.high();
	}
	for(i=0;i<3;i++) imuoffset[i]=avgbuf[i];
	beep.low();
	//mpu6050校正完成
	if( MAGNETOMETER_ISIN ){
			//校正磁力计
			leftwheel.setPWM(800);		//小车原地旋转
			rightwheel.setPWM(-800);
			//假定磁力计水平，磁力计的角度 就只和X Y有关。这里校正 x,y  校正过程持续 20秒，需要小车不停的旋转
			for(i=0; i<2000; i++) {
					mpu.getAllMagnetometer(fbuf);		//读取原始数据
					if(xmin<fbuf[0]) xmin=fbuf[0];
					if(zmin<fbuf[1]) zmin=fbuf[1];
					if(ymin<fbuf[2]) ymin=fbuf[2];
					if(xmax>fbuf[0]) xmax=fbuf[0];
					if(zmax>fbuf[1]) zmax=fbuf[1];
					if(ymax>fbuf[2]) ymax=fbuf[2];
					delay(10);
					if(i%100) ledgreen.invert();
					if(i>1800) beep.high();
			}
			leftwheel.setPWM(0);		//小车停止运动
			rightwheel.setPWM(0);
			beep.low();
			ledgreen.high();
			ledred.low();
	}
	magoffset[0]=(xmax+xmin)/2.0;
	magoffset[1]=(zmax+zmin)/2.0;
	magoffset[2]=(ymax+ymin)/2.0;
}
