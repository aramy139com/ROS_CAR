//mpu6050.cpp
#include "mpu6050.h"
#include "millisecondtimer.h"
Mpu6050::Mpu6050(uint8_t gyrofsr,uint8_t accfsr) {
    uint8_t i;
    Wire.begin();
    gyro_fsr=gyrofsr;
    acc_fsr=accfsr;
    mpu_check=false;
    for(i=0; i<6; i++) {			//��ʼ�� ƫ����
        offset[i]=0;
    }
    orientation[0]=1;
    orientation[1]=1;
    orientation[2]=1;
}

void Mpu6050::write_to_register(int dev_addr, uint8_t reg_addr, uint8_t reg_value) {
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    Wire.write(reg_value);
    Wire.endTransmission();
}



//��ȡ6050������ ԭʼ���� 		���ٶ�
void Mpu6050::readGyroSource() {
    uint8_t i = 0;
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPU6050_GYRO_XOUTH_REG);
    Wire.endTransmission();
    Wire.requestFrom(MPU6050_ADDRESS, 6);
    while(Wire.available()) {
        gyro_buffer[i++] = Wire.read();
    }
}

//��ȡmpu6050���ٶ�ԭʼֵ
void Mpu6050::readAccSource() {
    uint8_t i = 0;
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPU6050_ACCEL_XOUTH_REG);
    Wire.endTransmission();
    Wire.requestFrom(MPU6050_ADDRESS, 6);
    while(Wire.available()) {
        acc_buffer[i++] = Wire.read();
    }
}
//����xyz�������������
void Mpu6050::setOrientation(int8_t x,int8_t y,int8_t z) {
    orientation[0]=x;
    orientation[1]=y;
    orientation[2]=z;
}
void Mpu6050::setOrientation(int8_t orient[]) {
    orientation[0]=orient[0];
    orientation[1]=orient[1];
    orientation[2]=orient[2];
}
void Mpu6050::setMegOrientation(int8_t orient[]) {
    megorientation[0]=orient[0];
    megorientation[1]=orient[1];
    megorientation[2]=orient[2];
}
//��ȡmpu6050ԭʼ���¶�
void Mpu6050::readTempSource() {
    uint8_t i = 0,buf[2];
    temperature=0;
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPU6050_TEMP_OUTH_REG);
    Wire.endTransmission();
    Wire.requestFrom(MPU6050_ADDRESS, 2);
    while(Wire.available()) {
        buf[i++] = Wire.read();
    }
    //((u16)buf[0]<<8)|buf[1])
    temperature=(short)((u16)buf[0]<<8|buf[1]);
}

bool Mpu6050::mpu6050_init() {
    write_to_register(MPU6050_ADDRESS,MPU6050_PWR_MGMT1_REG,0X80);	//��λMPU6050
    write_to_register(MPU6050_ADDRESS,MPU6050_PWR_MGMT1_REG,0X00);	//����MPU6050
    write_to_register(MPU6050_ADDRESS,MPU6050_GYRO_CFG_REG,gyro_fsr);//���������������̷�Χ
    write_to_register(MPU6050_ADDRESS,MPU6050_ACCEL_CFG_REG,acc_fsr);//���ü��ٶȴ����������̷�Χ   +-4G
    write_to_register(MPU6050_ADDRESS,MPU6050_INT_EN_REG,0X00);	//�ر������ж�
    write_to_register(MPU6050_ADDRESS,MPU6050_USER_CTRL_REG,0X00);	//I2C��ģʽ�ر�
    write_to_register(MPU6050_ADDRESS,MPU6050_FIFO_EN_REG,0X00);	//�ر�FIFO
    write_to_register(MPU6050_ADDRESS,MPU6050_INTBP_CFG_REG,0X80);	//INT���ŵ͵�ƽ��Ч
    //������mpu6050�Ĳ�����  ��û̫��������ʲô��
    write_to_register(MPU6050_ADDRESS,MPU6050_SAMPLE_RATE_REG,0x04);	//�������ֵ�ͨ�˲���  50HZ
    write_to_register(MPU6050_ADDRESS,MPU6050_CFG_REG,0x06);//�������ֵ�ͨ�˲���

    //��������Ƿ����
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPU6050_DEVICE_ID_REG);
    Wire.endTransmission();
    Wire.requestFrom(MPU6050_ADDRESS, 1);
    if( Wire.read()== MPU6050_ADDRESS) {
        write_to_register(MPU6050_ADDRESS,MPU6050_PWR_MGMT1_REG,0X01);	//����CLKSEL,PLL X��Ϊ�ο�
        write_to_register(MPU6050_ADDRESS,MPU6050_PWR_MGMT2_REG,0X00);	//���ٶ��������Ƕ�����
        mpu_check=true;
        return true;
    }
    else {
        mpu_check=false;
        return false;
    }
}
//hmc5883l ��ʼ��
void Mpu6050::hmc5883l_init() {
    //��ʼ�� ������ HMC5883L
    write_to_register(HMC5883L_MAG_ADDRESS,HMC5883L_MAG_REG_A,0x18);
    /*	bit0-bit1 xyz�Ƿ�ʹ��ƫѹ,Ĭ��Ϊ0��������  bit2-bit4 �����������, 110Ϊ���75HZ 100Ϊ15HZ ��С000 0.75HZ bit5-bit5ÿ�β���ƽ���� 11Ϊ8�� 00Ϊһ��		*/
    write_to_register(HMC5883L_MAG_ADDRESS,HMC5883L_MAG_REG_B,0x20);			//+-1.3ga  �����Χ 0xf800-0x07ff  �� -2048~2047 )
    //�������� 5/6/7 ��λ 000 0.88ga  1370
    // 										001 1.3ga��Ĭ�ϣ� 1090
    //										010 1.9ga  	820
    //										011 2.5ga 	660
    //										100 4ga  440
    //										101 4.7ga  390
    //										110  5.6ga  330
    //										111 8.1ga		230
    //�����Χ ����  0xf800~0x07ff
    write_to_register(HMC5883L_MAG_ADDRESS,HMC5883L_MAG_MODE,0x00);
    megorientation[0]=1;
    megorientation[1]=1;
    megorientation[2]=1;
}

//���ָ����Ľ��ٶ�   ��� ��  ��λ ���ڣ���Ӧ��Ľ��ٶ�  Ĭ�Ϸ���ԭʼ������
float Mpu6050::getGyroVal(uint8_t axis,uint8_t valunit ) {
    short reval;
    float val=0.0;
    readGyroSource();
    reval=((short)((gyro_buffer[axis*2]<<8)|(gyro_buffer[axis*2+1]))+offset[axis])*orientation[axis];
    //��ԭʼ���ݰ� ���� ת��
    val=(float)reval;
    if(valunit==ORGI) {		//ԭʼֵ
        return val;
    }
    //������ת��  ��λ ��/��
    switch(gyro_fsr) {
    case GYRO250:
        val=val/131.072;
        break;
    case GYRO500:
        val=val/65.536;
        break;
    case GYRO1000:
        val=val/32.768;
        break;
    case GYRO2000:
        val=val/16.384;
        break;
    }
    if(valunit==ANGLE)		//��/��
        return val;
    else
        return val/57.295779513;		//����/��
}

//���ָ���� ���ٶ�   ��� ��  ��λ ���ڣ���Ӧ��ļ��ٶ�  Ĭ�Ϸ���ԭʼ������
float Mpu6050::getAccVal(uint8_t axis,uint8_t valunit ) {
    short reval;
    float val=0.0;
    readAccSource();
    reval=((short)((acc_buffer[axis*2]<<8)|(acc_buffer[axis*2+1]))+offset[axis+3])*orientation[axis];
    //��ԭʼ���ݰ� ���� ת��
    val=(float)reval;
    if(valunit==ORGI) {		//ԭʼֵ
        return val;
    } else {
        //������ת��  ��λ g
        switch(acc_fsr) {
        case ACC2G:
            val=val/16384;
            break;
        case ACC4G:
            val=val/8192;
            break;
        case ACC8G:
            val=val/4096;
            break;
        case ACC16G:
            val=val/2048;
            break;
        }
        return val;
    }
}

//��� ������ٶ� ��� 3���� float���飬��λ  �
void Mpu6050::getGyroAllVal(float valBuf[],uint8_t valunit) {
    uint8_t i;
    short reval;
    float val=0.0;
    readGyroSource();
    for(i=0; i<3; i++) {
        reval=((short)((gyro_buffer[i*2]<<8)|(gyro_buffer[i*2+1]))+offset[i])*orientation[i];
        //��ԭʼ���ݰ� ���� ת��
        val=(float)reval;
        if(valunit==ORGI) {		//ԭʼֵ
            valBuf[i]=val;
        } else {
            //������ת��  ��λ ��/��
            switch(gyro_fsr) {
            case GYRO250:
                val=val/131.072;
                break;
            case GYRO500:
                val=val/65.536;
                break;
            case GYRO1000:
                val=val/32.768;
                break;
            case GYRO2000:
                val=val/16.384;
                break;
            }
            if(valunit==ANGLE)		//��/��
                valBuf[i]=val;
            else
                valBuf[i]=val/57.295779513;		//����/��
        }
    }
}

//��� ������ٶ� ��� 3���� float���飬��λ
void Mpu6050::getAccAllVal(float valBuf[],uint8_t valunit) {
    uint8_t i;
    short reval;
    float val=0.0;
    readAccSource();
    for(i=0; i<3; i++) {
        reval=((short)((acc_buffer[i*2]<<8)|(acc_buffer[i*2+1]))+offset[i+3])*orientation[i];
        //��ԭʼ���ݰ� ���� ת��
        val=(float)reval;
        if(valunit==ORGI) {		//ԭʼֵ
            valBuf[i]=val;
        } else {
            //������ת��  ��λ g
            switch(acc_fsr) {
            case ACC2G:
                val=val/16384;
                break;
            case ACC4G:
                val=val/8192;
                break;
            case ACC8G:
                val=val/4096;
                break;
            case ACC16G:
                val=val/2048;
                break;
            }
            valBuf[i]=val;
        }
    }
}
//��ȡ������ԭʼ����
//��ȡ6050������ ԭʼ���� 		���ٶ�
void Mpu6050::readMagnetometer() {
    uint8_t i = 0;
    Wire.beginTransmission(HMC5883L_MAG_ADDRESS);
    Wire.write(HMC5883L_MAG_DATAX0);
    Wire.endTransmission();
    Wire.requestFrom(HMC5883L_MAG_ADDRESS, 6);
    while(Wire.available()) {
        mag_buffer[i++] = Wire.read();
    }
}
//�����Ƶ�У׼
/*������⣬�õشų�У�����̵�ʵ���������ģ�
�ڲ�ͬ��������شų����õ�����[x y z]��
��������£���Щ[x y z]Ӧ���ڰ뾶Ϊ|�شų�ǿ��|�������ϡ�
������ƫ�ƺͱ�����ʵ����������һ�������档
����Ҫ���ľ���ȷ��У��ϵ�������������Ū��Բ��

�򵥵�У���Ĺ��̾��ǣ�
�ڿռ�����ת���ҳ������С��x��y��z����Щ�������Ϊ����������������Ľ��㡣
���Կ�����-(max+min)/2��������ƫ�ƣ�ע�⸺�ţ����൱��ԭ������������ĵ�ƫ�ơ������������У����ƫ�ơ�
��Ϊֻ��Ҫ���򣬿�����x��ı���ϵ��x_gainΪ1��
Ȼ��y�ı���ϵ��y_gain = x_gain * (y_max-y_min)/(x_max-x_min);
z�ı���ϵ�����ƣ�z_gain = x_gain * (z_max-z_min)/(x_max-x_min);
��Ȼ��ͨ��x_gain��ȡֵ�����԰�[x y z]�ĵ�λУ��Ϊ1��T���˹��
*/
/*
void Mpu6050::calibrateMag() {
    float mag[3];			// X Z Y
    float xmin=0,xmax=0,ymin=0,ymax=0,zmin=0,zmax=0;
    uint16_t i=0;
    //�ٶ�������ˮƽ�������ƵĽǶ� ��ֻ��X Y�йء�����У�� x,y  У�����̳��� 20�룬��ҪС����ͣ����ת
    for(i=0; i<2000; i++) {
        getAllMagnetometer(mag);
        if(xmin<mag[0]) xmin=mag[0];
        if(zmin<mag[1]) zmin=mag[0];
        if(ymin<mag[2]) ymin=mag[2];
        if(xmax>mag[0]) xmax=mag[0];
        if(zmax>mag[1]) zmax=mag[1];
        if(ymax>mag[2]) ymax=mag[2];
        delay(10);
    }
    megorientationoffset[0]=(xmax+xmin)/2.0;
    megorientationoffset[1]=(zmax+zmin)/2.0;
    megorientationoffset[2]=(ymax+ymin)/2.0;
}
*/
//��ô�������Ϣ   ��� װ�����ݵ�ָ�룬flag �ж��Ƿ���ҪУ��
void Mpu6050::getAllMagnetometer(float valBuf[],uint8_t flag) {
    uint8_t i;
    //short reval;
    float val=0.0;
    readMagnetometer();
    for(i=0; i<3; i++) {
        val=(float)(megorientation[i] * ((int16_t)((int)mag_buffer[2*i] << 8) | (mag_buffer[2*i+1]))) ;
        valBuf[i]=val*1000.00/1090.00;		//��ԭʼ���ݰ� ���� ת��  ������ֵ�й�
    }
    if(flag) {			//У������
        for(i=0; i<3; i++) {
            valBuf[i]-=megorientationoffset[i];
        }
    }
    //write_to_register(HMC5883L_MAG_ADDRESS,HMC5883L_MAG_MODE,0x01);
}