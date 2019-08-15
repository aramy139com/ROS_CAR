//mpu6050.cpp
#include "mpu6050.h"
#include "millisecondtimer.h"
#include "ioi2c.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu.h"
static signed char gyro_orientation[9] = {1, 0, 0,
                                          0,-1, 0,
                                          0, 0, 1
                                         };

static  unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static  unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

static void run_self_test(void) {
    int result;
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x7) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
    }
}
Mpu6050::Mpu6050(uint8_t gyrofsr,uint8_t accfsr) {
    uint8_t i;
    gyro_fsr=gyrofsr;
    acc_fsr=accfsr;
    mpu_check=false;
    for(i=0; i<6; i++) {			//初始化 偏移量
        offset[i]=0;
    }
    orientation[0]=1;
    orientation[1]=1;
    orientation[2]=1;
    IIC_Init();			//初始化IIC
}
//  Gyro FSR: +/- 2000DPS\n
//  Accel FSR +/- 2G\n
void Mpu6050::mpu6050_init() {
    if(!mpu_init()) {
        mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
        mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
        mpu_set_sample_rate(DEFAULT_MPU_HZ);
        dmp_load_motion_driver_firmware();
        dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
        dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
                           DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
                           DMP_FEATURE_GYRO_CAL);
        dmp_set_fifo_rate(DEFAULT_MPU_HZ);
        run_self_test();
        mpu_set_dmp_state(1);
    }
}

//读取6050陀螺仪 原始数据 		角速度
void Mpu6050::readGyroSource() {
    uint8_t i = 0;
    for(i=0; i<6; i++) {
        gyro_buffer[i] = I2C_ReadOneByte(devAddr,MPU6050_RA_TEMP_OUT_H+i);
    }
}

//读取mpu6050加速度原始值
void Mpu6050::readAccSource() {
    uint8_t i = 0;
    for(i=0; i<6; i++) {
        gyro_buffer[i] = I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H+i);
    }
}
//设置xyz轴向的修正方向
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
//读取mpu6050原始的温度
void Mpu6050::readTempSource() {
    temperature=(I2C_ReadOneByte(devAddr,MPU6050_RA_TEMP_OUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_TEMP_OUT_L);
    //if(temperature>32768) temperature-=65536;
    temperature=(36.53+temperature/340)*10;
}

//hmc5883l 初始化
void Mpu6050::hmc5883l_init() {
    //初始化 磁力计 HMC5883L
    //write_to_register(HMC5883L_MAG_ADDRESS,HMC5883L_MAG_REG_A,0x18);
    /*	bit0-bit1 xyz是否使用偏压,默认为0正常配置  bit2-bit4 数据输出速率, 110为最大75HZ 100为15HZ 最小000 0.75HZ bit5-bit5每次采样平均数 11为8次 00为一次		*/
    // write_to_register(HMC5883L_MAG_ADDRESS,HMC5883L_MAG_REG_B,0x20);			//+-1.3ga  输出范围 0xf800-0x07ff  （ -2048~2047 )
    //增益配置 5/6/7 三位 000 0.88ga  1370
    // 										001 1.3ga（默认） 1090
    //										010 1.9ga  	820
    //										011 2.5ga 	660
    //										100 4ga  440
    //										101 4.7ga  390
    //										110  5.6ga  330
    //										111 8.1ga		230
    //输出范围 都是  0xf800~0x07ff
//    write_to_register(HMC5883L_MAG_ADDRESS,HMC5883L_MAG_MODE,0x00);
//    megorientation[0]=1;
//    megorientation[1]=1;
//    megorientation[2]=1;
}

//获得指定轴的角速度   入口 轴  单位 出口：对应轴的角速度  默认返回原始的数据
float Mpu6050::getGyroVal(uint8_t axis,uint8_t valunit ) {
    short reval;
    float val=0.0;
    readGyroSource();
    reval=((short)((gyro_buffer[axis*2]<<8)|(gyro_buffer[axis*2+1]))+offset[axis])*orientation[axis];
    //将原始数据按 量程 转换
    val=(float)reval;
    if(valunit==ORGI) {		//原始值
        return val;
    }
    //按量程转换  单位 度/秒
    val=val/16.384;
    if(valunit==ANGLE)		//度/秒
        return val;
    else
        return val/57.295779513;		//弧度/秒
}

//获得指定轴 加速度   入口 轴  单位 出口：对应轴的加速度  默认返回原始的数据
float Mpu6050::getAccVal(uint8_t axis,uint8_t valunit ) {
    short reval;
    float val=0.0;
    readAccSource();
    reval=((short)((acc_buffer[axis*2]<<8)|(acc_buffer[axis*2+1]))+offset[axis+3])*orientation[axis];
    //将原始数据按 量程 转换
    val=(float)reval;
    if(valunit==ORGI) {		//原始值
        return val;
    } else {
        //按量程转换  单位 g
        val=val/16384;
        return val;
    }
}

//获得 三轴角速度 入口 3长度 float数组，单位  �
void Mpu6050::getGyroAllVal(float valBuf[],uint8_t valunit) {
    uint8_t i;
    short reval;
    float val=0.0;
    readGyroSource();
    for(i=0; i<3; i++) {
        reval=((short)((gyro_buffer[i*2]<<8)|(gyro_buffer[i*2+1]))+offset[i])*orientation[i];
        //将原始数据按 量程 转换
        val=(float)reval;
        if(valunit==ORGI) {		//原始值
            valBuf[i]=val;
        } else {           
            val=val/16.384;
            if(valunit==ANGLE)		//度/秒
                valBuf[i]=val;
            else
                valBuf[i]=val/57.295779513;		//弧度/秒
        }
    }
}

//获得 三轴加速度 入口 3长度 float数组，单位
void Mpu6050::getAccAllVal(float valBuf[],uint8_t valunit) {
    uint8_t i;
    short reval;
    float val=0.0;
    readAccSource();
    for(i=0; i<3; i++) {
        reval=((short)((acc_buffer[i*2]<<8)|(acc_buffer[i*2+1]))+offset[i+3])*orientation[i];
        //将原始数据按 量程 转换
        val=(float)reval;
        if(valunit==ORGI) {		//原始值
            valBuf[i]=val;
        } else {           
            val=val/16384;               
            valBuf[i]=val;
        }
    }
}
//获取磁力计原始数据
//读取6050陀螺仪 原始数据 		角速度
void Mpu6050::readMagnetometer() {
    uint8_t i = 0;
//    Wire.beginTransmission(HMC5883L_MAG_ADDRESS);
//    Wire.write(HMC5883L_MAG_DATAX0);
//    Wire.endTransmission();
//    Wire.requestFrom(HMC5883L_MAG_ADDRESS, 6);
//    while(Wire.available()) {
//        mag_buffer[i++] = Wire.read();
//    }
}
//磁力计的校准
/*依我理解，用地磁场校正罗盘的实质是这样的：
在不同方向测量地磁场，得到多组[x y z]。
理想情况下，这些[x y z]应该在半径为|地磁场强度|的球面上。
但由于偏移和比例误差，实际贴近的是一个椭球面。
我们要做的就是确定校正系数，把这个椭球弄成圆球。

简单的校正的过程就是：
在空间上旋转，找出最大最小的x、y、z。这些点可以认为是椭球面与坐标轴的交点。
所以可以用-(max+min)/2计算各轴的偏移（注意负号）。相当于原点相对于椭球心的偏移。加上这个，就校正了偏移。
因为只需要方向，可以令x轴的比例系数x_gain为1。
然后y的比例系数y_gain = x_gain * (y_max-y_min)/(x_max-x_min);
z的比例系数类似，z_gain = x_gain * (z_max-z_min)/(x_max-x_min);
当然，通过x_gain的取值，可以把[x y z]的单位校正为1、T或高斯。
*/
/*
void Mpu6050::calibrateMag() {
    float mag[3];			// X Z Y
    float xmin=0,xmax=0,ymin=0,ymax=0,zmin=0,zmax=0;
    uint16_t i=0;
    //假定磁力计水平，磁力计的角度 就只和X Y有关。这里校正 x,y  校正过程持续 20秒，需要小车不停的旋转
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
//获得磁力计信息   入口 装载数据的指针，flag 判断是否需要校正
void Mpu6050::getAllMagnetometer(float valBuf[],uint8_t flag) {
    uint8_t i;
    //short reval;
    float val=0.0;
    readMagnetometer();
    for(i=0; i<3; i++) {
        val=(float)(megorientation[i] * ((int16_t)((int)mag_buffer[2*i] << 8) | (mag_buffer[2*i+1]))) ;
        valBuf[i]=val*1000.00/1090.00;		//将原始数据按 量程 转换  与增益值有关
    }
    if(flag) {			//校正数据
        for(i=0; i<3; i++) {
            valBuf[i]-=megorientationoffset[i];
        }
    }
    //write_to_register(HMC5883L_MAG_ADDRESS,HMC5883L_MAG_MODE,0x01);
}