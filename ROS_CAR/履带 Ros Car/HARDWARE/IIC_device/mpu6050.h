//filename mpu6050.h
#ifndef _MPU6050_H_
#define _MPU6050_H_
#include "Wire.h"
#include "Gpio.h"

#define MPU6050_DEVICE_ID_REG		0X75	//器件ID寄存器
//如果AD0脚(9脚)接地,IIC地址为0X68(不包含最低位).
//如果接V3.3,则IIC地址为0X69(不包含最低位).
#define MPU6050_ADDRESS						0X68
#define MPU6050_USER_CTRL_REG			0X6A	//用户控制寄存器
#define MPU6050_PWR_MGMT1_REG			0X6B	//电源管理寄存器1
#define MPU6050_PWR_MGMT2_REG			0X6C	//电源管理寄存器2 
#define MPU6050_CFG_REG						0X1A	//配置寄存器
#define MPU6050_GYRO_CFG_REG			0X1B	//陀螺仪配置寄存器
#define MPU6050_ACCEL_CFG_REG			0X1C	//加速度计配置寄存器
#define MPU6050_SAMPLE_RATE_REG		0X19	//采样频率分频器
#define MPU6050_INT_EN_REG				0X38	//中断使能寄存器
#define MPU6050_FIFO_EN_REG				0X23	//FIFO使能寄存器
#define MPU6050_INTBP_CFG_REG			0X37	//中断/旁路设置寄存器
#define MPU6050_ACCEL_XOUTH_REG		0X3B	//加速度值,X轴高8位寄存器
#define MPU6050_GYRO_XOUTH_REG		0X43	//陀螺仪值,X轴高8位寄存器
#define MPU6050_TEMP_OUTH_REG			0X41	//温度值高八位寄存器

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


//完完整整写出一个mpu6050的类，利用IIC通讯。
class Mpu6050 {
private:
		bool mpu_check;
		uint8_t gyro_fsr;							//陀螺仪传感器 量程 
		uint8_t acc_fsr;							//角速度传感器 量程
    uint8_t gyro_buffer[7];				//原始角速度  陀螺仪
    uint8_t acc_buffer[7];				//原始的加速度
		uint8_t mag_buffer[7];				//原始的磁力计
		short temperature;					//原始温度
		
		float offset[6];						//修正值  前3为为角速度修正值，后3为为加速度修正值
		int8_t orientation[3];				//方向修正值，三个方向，分别是 X Y Z 取值为 1或 -1
		int8_t megorientation[3];				//磁力计方向修正值，三个方向，分别是 X Z Y 取值为 1或 -1
		float megorientationoffset[3];	//磁力计的修正值  X Z Y
		
    TwoWire Wire;

		void write_to_register(int dev_addr, uint8_t reg_addr, uint8_t reg_value);		//向寄存器写入控制信息
		
		void readGyroSource();			//读取mpu6050陀螺仪原始数据		角速度原始值		
		void readAccSource();				//读取mpu6050加速度原始值
		void readTempSource();			//读取mpu6050的温度
		void readMagnetometer();		//获取磁力计原始数据

public:
		Mpu6050(uint8_t gyrofsr=GYRO2000,uint8_t accfsr=ACC2G);
    bool mpu6050_init();			//mpu6050初始化
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

