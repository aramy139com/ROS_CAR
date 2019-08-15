//filename mpu6050.h
#ifndef _MPU6050_H_
#define _MPU6050_H_
#include "Gpio.h"
#include "mpu6050dmp.h"

//����ų�ǿ
/** HMC5883L config**/
#define HMC5883L_MAG_ADDRESS 0x1E
#define HMC5883L_MAG_ID 0x10
#define HMC5883L_MAG_REG_A 0x00
#define HMC5883L_MAG_REG_B 0x01
#define HMC5883L_MAG_MODE 0x02
#define HMC5883L_MAG_DATAX0 0x03


//����mpu6050��ص�ö����
enum MPU6050_FSR							
{
		//�����Ǵ����� ����  ���ٶ�
    GYRO250=0X00,GYRO500=0X08,GYRO1000=0X10,GYRO2000=0X18,
		//���ٶȴ����� ����
		ACC2G=0X00,ACC4G=0X08,ACC8G=0X10,ACC16G=0X18,
};

enum MPU6050_AXIS							
{
	GX=0,GY,GZ,AX=0,AY,AZ,
	ORGI=0,RAD,ANGLE,G						//ԭʼֵ �����ȡ��Ƕ�, 
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


//��������д��һ��mpu6050���࣬����IICͨѶ��
class Mpu6050 {
private:
		bool mpu_check;
		uint8_t gyro_fsr;							//�����Ǵ����� ���� 
		uint8_t acc_fsr;							//���ٶȴ����� ����
    uint8_t gyro_buffer[6];				//ԭʼ���ٶ�  ������
    uint8_t acc_buffer[6];				//ԭʼ�ļ��ٶ�
		uint8_t mag_buffer[6];				//ԭʼ�Ĵ�����
		short temperature;					//ԭʼ�¶�
		
		float offset[6];						//����ֵ  ǰ3ΪΪ���ٶ�����ֵ����3ΪΪ���ٶ�����ֵ
		int8_t orientation[3];				//��������ֵ���������򣬷ֱ��� X Y Z ȡֵΪ 1�� -1
		int8_t megorientation[3];				//�����Ʒ�������ֵ���������򣬷ֱ��� X Z Y ȡֵΪ 1�� -1
		float megorientationoffset[3];	//�����Ƶ�����ֵ  X Z Y
		
		void readGyroSource();			//��ȡmpu6050������ԭʼ����		���ٶ�ԭʼֵ		
		void readAccSource();				//��ȡmpu6050���ٶ�ԭʼֵ
		void readTempSource();			//��ȡmpu6050���¶�
		void readMagnetometer();		//��ȡ������ԭʼ����

public:
		Mpu6050(uint8_t gyrofsr=GYRO2000,uint8_t accfsr=ACC2G);
    void mpu6050_init();			//mpu6050��ʼ��
		void hmc5883l_init();			//hmc5883l_init��ʼ��
		float getGyroVal(uint8_t axis,uint8_t valunit=ORGI );			//���ָ����Ľ��ٶ�   ��� ��  ��λ ���ڣ���Ӧ��Ľ��ٶ�
		float getAccVal(uint8_t axis,uint8_t valunit=ORGI );			//���ָ����ļ��ٶ�   ��� ��  ��λ ���ڣ���Ӧ�� �ļ��ٶ�
		void getGyroAllVal(float valBuf[],uint8_t valunit=ORGI);	//��� ������ٶ� ��� 3���� float���飬��λ  
		void getAccAllVal(float valBuf[],uint8_t valunit=ORGI);	//��� ������ٶ� ��� 3���� float���飬��λ  
		//void getTestMpuVal(short info[]);					//������
		void setOrientation(int8_t x,int8_t y,int8_t z);
		void setOrientation(int8_t orient[]);
		void setMegOrientation(int8_t orient[]);
		float getRealTemperature(){		//�����ʵ�¶� ��λ���϶�
			return 36.53+(float)getTemperature()/340;
		}
		void setGyrofsr(uint8_t _gyrofsr){
			gyro_fsr=_gyrofsr;
		}
		void setAcc_fsr(uint8_t _acc_fsr){
			acc_fsr=_acc_fsr;
		}
		void calibrateMag();			//У��������
		//��ȡ��������Ϣ
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
		//����6050��ƫ����
		void setOffset(float* val){
			uint8_t i;
			for(i=0;i<6;i++){
				offset[i]=val[i];
			}
		}
		float* getOffset(){
			return offset;
		}
		//���ô����Ƶ�ƫ����
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

