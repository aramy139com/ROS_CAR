//filename mpu6050.h
#ifndef _MPU6050_H_
#define _MPU6050_H_
#include "Gpio.h"
#include "mpu6050dmp.h"

//三轴磁场强
/** HMC5883L config**/
#define HMC5883L_MAG_ADDRESS 0x1E
#define HMC5883L_MAG_ID 0x10
#define HMC5883L_MAG_REG_A 0x00
#define HMC5883L_MAG_REG_B 0x01
#define HMC5883L_MAG_MODE 0x02
#define HMC5883L_MAG_DATAX0 0x03


//定义mpu6050相关的枚举类
enum MPU6050_FSR							
{
		//陀螺仪传感器 量程  角速度
    GYRO250=0X00,GYRO500=0X08,GYRO1000=0X10,GYRO2000=0X18,
		//加速度传感器 量程
		ACC2G=0X00,ACC4G=0X08,ACC8G=0X10,ACC16G=0X18,
};

enum MPU6050_AXIS							
{
	GX=0,GY,GZ,AX=0,AY,AZ,
	ORGI=0,RAD,ANGLE,G						//原始值 、弧度、角度, 
};

#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define MOTION          (0)
#define NO_MOTION       (1)
#define DEFAULT_MPU_HZ  (200)
#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)
#define q30  1073741824.0f


//完完整整写出一个mpu6050的类，利用IIC通讯。
class Mpu6050 {
private:
		bool mpu_check;
		uint8_t gyro_fsr;							//陀螺仪传感器 量程 
		uint8_t acc_fsr;							//角速度传感器 量程
    uint8_t gyro_buffer[6];				//原始角速度  陀螺仪
    uint8_t acc_buffer[6];				//原始的加速度
		uint8_t mag_buffer[6];				//原始的磁力计
		short temperature;					//原始温度
		
		float offset[6];						//修正值  前3为为角速度修正值，后3为为加速度修正值
		int8_t orientation[3];				//方向修正值，三个方向，分别是 X Y Z 取值为 1或 -1
		int8_t megorientation[3];				//磁力计方向修正值，三个方向，分别是 X Z Y 取值为 1或 -1
		float megorientationoffset[3];	//磁力计的修正值  X Z Y
		
		void readGyroSource();			//读取mpu6050陀螺仪原始数据		角速度原始值		
		void readAccSource();				//读取mpu6050加速度原始值
		void readTempSource();			//读取mpu6050的温度
		void readMagnetometer();		//获取磁力计原始数据

public:
		Mpu6050(uint8_t gyrofsr=GYRO2000,uint8_t accfsr=ACC2G);
    void mpu6050_init();			//mpu6050初始化
		void hmc5883l_init();			//hmc5883l_init初始化
		float getGyroVal(uint8_t axis,uint8_t valunit=ORGI );			//获得指定轴的角速度   入口 轴  单位 出口：对应轴的角速度
		float getAccVal(uint8_t axis,uint8_t valunit=ORGI );			//获得指定轴的加速度   入口 轴  单位 出口：对应轴 的加速度
		void getGyroAllVal(float valBuf[],uint8_t valunit=ORGI);	//获得 三轴角速度 入口 3长度 float数组，单位  
		void getAccAllVal(float valBuf[],uint8_t valunit=ORGI);	//获得 三轴加速度 入口 3长度 float数组，单位  
		//void getTestMpuVal(short info[]);					//测试用
		void setOrientation(int8_t x,int8_t y,int8_t z);
		void setOrientation(int8_t orient[]);
		void setMegOrientation(int8_t orient[]);
		float getRealTemperature(){		//获得真实温度 单位摄氏度
			return 36.53+(float)getTemperature()/340;
		}
		void setGyrofsr(uint8_t _gyrofsr){
			gyro_fsr=_gyrofsr;
		}
		void setAcc_fsr(uint8_t _acc_fsr){
			acc_fsr=_acc_fsr;
		}
		void calibrateMag();			//校正磁力计
		//获取磁力计信息
		void getAllMagnetometer(float valBuf[],uint8_t flag=0);
		bool mpuIsOk(){	
			return mpu_check;
		}
		uint8_t getGyroFsr(){
			return gyro_fsr;
		}
		uint8_t getAccFsr(){
			return acc_fsr;
		}
		short getTemperature(){
			readTempSource();
			return temperature;
		}
		uint8_t* getGyroBuffer(){
			readGyroSource();
			return gyro_buffer;
		}
		uint8_t* getAccBuffer(){
			readAccSource();
			return acc_buffer;
		}
		//设置6050的偏移量
		void setOffset(float* val){
			uint8_t i;
			for(i=0;i<6;i++){
				offset[i]=val[i];
			}
		}
		float* getOffset(){
			return offset;
		}
		//设置磁力计的偏移量
		void setMegorientationOffset(float of0,float of1,float of2){
			megorientationoffset[0]=of0;
			megorientationoffset[1]=of1;
			megorientationoffset[2]=of2;
		}
		void setMegorientationOffset(float *val){
			megorientationoffset[0]=val[0];
			megorientationoffset[1]=val[0];
			megorientationoffset[2]=val[0];
		}
		float* getMegorientationOffset(){
			return megorientationoffset;
		}
};

#endif //_MPU6050_H_

