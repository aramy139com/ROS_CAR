//filename roscar.cpp
#include "roscar.h"
#include "config.h"
#include "millisecondtimer.h"
#include "hardwareserial.h"
#include "StFlash.h"
//extern HardwareSerial serial;
void Roscar::initialize(){	
	float fbuf[7];
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);			//��
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);	
	ledred= Gpio(PC,13);
	ledgreen= Gpio(PC,14);
	ledwarn=Gpio(PA,15);
	beep=Gpio(PB,3);
	ledred.high();							//�ر����е�
	ledgreen.high();				
	ledwarn.high();
	beep.low();									//�رշ�����
	
	//��ʼ��������
	mpu.setAcc_fsr(ACC16G);			//���ٶȴ����� ����
	mpu.setGyrofsr(GYRO2000);		//�����Ǵ����� ����  ���ٶ�
	mpu.mpu6050_init();
	mpu.setOrientation(1,-1,-1);		//���ô�������ת����
	fbuf[0]=-31.2280655;
	fbuf[1]=-17.0690289;
	fbuf[2]=13.7129383;
	fbuf[3]=0;
	fbuf[4]=0;
	fbuf[5]=0;
	mpu.setOffset(fbuf);					//6050ƫ����
	mpu.setMegorientationOffset(-26.6054993,-97.7064209,-95.2752228);
	
	if( MAGNETOMETER_ISIN ){			//�����Ƶĳ�ʼ��
		mpu.hmc5883l_init();
	}
	leftwheel.initialize(LEFT,1999,359);		//���ӳ�ʼ��
	rightwheel.initialize(RIGHT,1999,359);
		
	//��س�ʼ��
	batpower.initialize();
	

	//pid��ʼ��
	leftpid.initialize(-1999, 1999, K_P, K_I, K_D);
	rightpid.initialize(-1999, 1999, K_P, K_I, K_D);	
	
	actualLineSpeed=0.0;				//ʵ�ʵ����ٶ�  ��/��
	expectLineSpeed=0.0;				//Ԥ�ڵ����ٶ�  ��/��
	actualAngleSpeed=0.0;				//ʵ�ʵĽ��ٶ�	����/��
	expectAngleSpeed=0.0;				//Ԥ�ڵĽ��ٶ�	����/��
	
	leftencode.initialize(LEFT);		//������
	rightencode.initialize(RIGHT);	
	prev_update_time=millis();
	
	if( MAGNETOMETER_ISIN ){
		mpu.getAllMagnetometer(fbuf,1);
		raw_imu_msg.magnetic_field.x=fbuf[0];
		raw_imu_msg.magnetic_field.z=fbuf[1];
		raw_imu_msg.magnetic_field.y=fbuf[2];		
		//����С����ָ��  �ٶ�С�������ƽ�� ֻ����  X Y
		headingRadians=atan2(fbuf[2],fbuf[0]);
		if(headingRadians<0) headingRadians+=2*PI;
	}
}

//��ü��ٶȡ������Ǵ�������ֵ
void Roscar::flushImuInfo(){
	float fbuf[3];
	//������ٶ�
	mpu.getAccAllVal(fbuf,G);	
	//mpu.getAccAllVal(fbuf,ORGI);
	raw_imu_msg.linear_acceleration.x=fbuf[0];
	raw_imu_msg.linear_acceleration.y=fbuf[1];
	raw_imu_msg.linear_acceleration.z=fbuf[2];
	//������ٶ�
	mpu.getGyroAllVal(fbuf,RAD);
	//mpu.getGyroAllVal(fbuf,ORGI);
	raw_imu_msg.angular_velocity.x=fbuf[0];
	raw_imu_msg.angular_velocity.y=fbuf[1];
	raw_imu_msg.angular_velocity.z=fbuf[2];
	//���������
	if( MAGNETOMETER_ISIN ){
		mpu.getAllMagnetometer(fbuf,1);
		raw_imu_msg.magnetic_field.x=fbuf[0];						//�������ʹ����Ƶ���Ϣ
		raw_imu_msg.magnetic_field.z=fbuf[1];
		raw_imu_msg.magnetic_field.y=fbuf[2];		
		//����С����ָ��  �ٶ�С�������ƽ�� ֻ����  X Y
		headingRadians=headingRadians*0.4+0.6*atan2(fbuf[2],fbuf[0]);
		if(headingRadians<0) headingRadians+=2*PI;
//		offsetmag=oldheading-headingRadians;
//		if(offsetmag<PI) offsetmag+=2*PI;
//		if(offsetmag>PI) offsetmag-=2*PI;
//		offsetmag=offsetmag/((double)1.0/IMU_PUBLISH_RATE);
	}
}
//���±�������Ϣ 
void Roscar::flushEncodeInfo(){
	uint32_t curr_time,offtimems;
	double offtime,leftspeed,rightspeed;								//���ŵ�ʱ��  ���ֵ��ٶ� ��/��
	//���������Ϣ
	leftencode.setEncoder();
	rightencode.setEncoder();	
	//serial.print("enc:[%d\t%d]    [%d\t%d]\n",leftencode.getEnValue(),rightencode.getEnValue(),leftencode.getTotleValue(),rightencode.getTotleValue());
	curr_time=millis();							//��ǰ��ʱ��
	offtimems=curr_time>prev_update_time?curr_time-prev_update_time:(4200000-prev_update_time)+curr_time;
	prev_update_time=curr_time;			//��¼�ɵ�ʱ���
	offtime=(double)offtimems/1000.00;			//��
	
	//���㵱ǰ��С�����ٶȺͽ��ٶ�
	leftspeed=WHEEL_PERIMETER*(double)leftencode.getEnValue()/COUNTS_PER_REV/offtime;			//��λ����/��
	rightspeed=WHEEL_PERIMETER*(double)rightencode.getEnValue()/COUNTS_PER_REV/offtime;
	leftwheel.setSpeed(leftspeed);													//��������������ٶȣ�д������
	rightwheel.setSpeed(rightspeed);
	actualLineSpeed=(leftspeed+rightspeed)/2;									//С����ǰ�� ���ٶ�
	
	//���㵱ǰС���Ľ��ٶ�  ���ٶ����������������� 
	//1 ͨ�������ٶȲ� 
	//offsetmag=(rightspeed-leftspeed)/CAR_WIDTHS/2.0;
	//2 ͨ��mpu6050�Ľ��ٶ�
	//3 ͨ�������Ƽ���Ƕȵ�ƫ��
	//actualAngleSpeed=actualAngleSpeed*0.5+raw_imu_msg.angular_velocity.z*0.20+offsetmag*0.3;
	actualAngleSpeed=actualAngleSpeed*0.5+raw_imu_msg.angular_velocity.z*0.5+FIXANGLESPEED;
	//actualAngleSpeed=actualAngleSpeed*0.5+offsetmag*0.5;																			//�������Ǹ��ö�����Ʈ����������
}

//��õ�ص�����Ϣ
float Roscar::getBatPower(){
	return batpower.get_volt();
}

//��ʾС����ǰ״̬��Ϣ
void Roscar::dispDebugInfo(char *buf){
	//sprintf(buf,"\nIMUZ:%.4fexp:[%.4f,%.4f],act:[%.4f,%.4f]\tbat=%.1f",raw_imu_msg.angular_velocity.z,expectLineSpeed,expectAngleSpeed,actualLineSpeed,actualAngleSpeed,getBatPower());
	//sprintf(buf,"\nIMUZ:%.3f, exp:[%.4f,%.4f],act:[%.4f,%.4f]\tbat=%.1f",raw_imu_msg.angular_velocity.z,expectLineSpeed,expectAngleSpeed,actualLineSpeed,actualAngleSpeed,getBatPower());
	//��ʾ imu�����Ϣ   9����Ϣ
	
	//sprintf(buf,"IMU:[%.4f,%.4f,%.4f],[%.4f,%.4f,%.4f],[%.4f,%.4f,%.4f]\n",raw_imu_msg.linear_acceleration.x,raw_imu_msg.linear_acceleration.y,raw_imu_msg.linear_acceleration.z,raw_imu_msg.angular_velocity.x,raw_imu_msg.angular_velocity.y,raw_imu_msg.angular_velocity.z,raw_imu_msg.magnetic_field.x,raw_imu_msg.magnetic_field.y,raw_imu_msg.magnetic_field.z);
	//sprintf(buf,"%ld:[%.4f,%.4f,%.4f]\t%.2f\n",millis(),raw_imu_msg.magnetic_field.x,raw_imu_msg.magnetic_field.y,raw_imu_msg.magnetic_field.z,atan2(raw_imu_msg.magnetic_field.y,raw_imu_msg.magnetic_field.x)*(180/PI)+180);
	//sprintf(buf,"%ld:[%.4f,%.4f,%.4f]\t%.2f\n",millis(),raw_imu_msg.magnetic_field.x,raw_imu_msg.magnetic_field.y,raw_imu_msg.magnetic_field.z,headingRadians*(180/PI));
	//��ʾ�ٶ������Ϣ
//	sprintf(buf,"EX:[%.2f,%.2f] SP_LR[%.3f,%.3f]\nACSP[%.2f,%.5f]  RAW[%.4f,%.4f],%.2f\n",expectLineSpeed,expectAngleSpeed,leftwheel.getSpeed(),rightwheel.getSpeed(),
//			actualLineSpeed,actualAngleSpeed,	raw_imu_msg.angular_velocity.z,offsetmag,headingRadians*57.295779513);
//		sprintf(buf,"EX:[%.2f,%.2f] SP_LR[%.3f,%.3f]\nACSP[%.2f,%.5f]  RAW[%.4f,%.2f]\n",expectLineSpeed,expectAngleSpeed,leftwheel.getSpeed(),rightwheel.getSpeed(),
//			actualLineSpeed,actualAngleSpeed,	raw_imu_msg.angular_velocity.z,headingRadians);
//	sprintf(buf,"EX:[%.2f,%.2f] SP[%.2f,%.2f] AC[%.2f,%.3f]  H:%.1f\n",expectLineSpeed,expectAngleSpeed,leftwheel.getSpeed(),rightwheel.getSpeed(),actualLineSpeed,actualAngleSpeed,headingRadians);
		sprintf(buf,"EX:[%.2f,%.2f]AC[%.2f,%.3f] H: %.1f\n",expectLineSpeed,expectAngleSpeed,actualLineSpeed,actualAngleSpeed,headingRadians*57.29578);

}

//С�����˶���Ϣ
void Roscar::moveCar(){
	int16_t lpwm,rpwm;
	double l_expspeed,r_exspeed,angspeed;				//�����������ٶ�
	//С��ͨ�� PID��ʽ���������������ӵ�ת�٣���ת���� ͨ�� ������������
	//�Ȳ�����ת���⣬���ٶ���������Ϊ ָ�����ٶ����ޡ�ֱ�Ӽ��������ֵ� pwmֵ
	if( expectLineSpeed>MAXLINESPEED) expectLineSpeed=MAXLINESPEED;
	if( expectLineSpeed<-MAXLINESPEED) expectLineSpeed=-MAXLINESPEED;
	
	l_expspeed=r_exspeed=expectLineSpeed;			//���������ٶ�
	//��������������Լ�����
//  angspeed=expectAngleSpeed-actualAngleSpeed;	
	//���ٶȶ�Ӧ�����ٶȵļ���
//	if( angspeed<-1.2 || angspeed>1.2 ){			//�ж�Ϊ�����Ŀ�ʼ
//		angspeed=expectAngleSpeed*CAR_WIDTHS;		//����������ʱ���ܾ��������������ƫ��
//	}	else{		
//		angspeed=(expectAngleSpeed+expectAngleSpeed-actualAngleSpeed)*CAR_WIDTHS;			//���ٶȶ�Ӧ�����ٶ�
//		//angspeed=expectAngleSpeed*CAR_WIDTHS;	
//	}
	angspeed=expectAngleSpeed*CAR_WIDTHS;					//�����Ľ��ٶ� �������ٶ�
	l_expspeed-=angspeed;
	r_exspeed+=angspeed;

	//ͨ��pid����� ��Ҫ���õ�PWMֵ
	lpwm=(int16_t)leftpid.computeLoc(l_expspeed,leftwheel.getSpeed());
	rpwm=(int16_t)rightpid.computeLoc(r_exspeed,rightwheel.getSpeed());
	leftwheel.setPWM(lpwm);									//�������
	rightwheel.setPWM(rpwm);
}
