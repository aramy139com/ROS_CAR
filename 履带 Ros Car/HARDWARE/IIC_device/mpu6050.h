//filename mpu6050.h
#ifndef _MPU6050_H_
#define _MPU6050_H_
#include "Wire.h"
#include "Gpio.h"

#define MPU6050_DEVICE_ID_REG		0X75	//����ID�Ĵ���
//���AD0��(9��)�ӵ�,IIC��ַΪ0X68(���������λ).
//�����V3.3,��IIC��ַΪ0X69(���������λ).
#define MPU6050_ADDRESS						0X68
#define MPU6050_USER_CTRL_REG			0X6A	//�û����ƼĴ���
#define MPU6050_PWR_MGMT1_REG			0X6B	//��Դ����Ĵ���1
#define MPU6050_PWR_MGMT2_REG			0X6C	//��Դ����Ĵ���2 
#define MPU6050_CFG_REG						0X1A	//���üĴ���
#define MPU6050_GYRO_CFG_REG			0X1B	//���������üĴ���
#define MPU6050_ACCEL_CFG_REG			0X1C	//���ٶȼ����üĴ���
#define MPU6050_SAMPLE_RATE_REG		0X19	//����Ƶ�ʷ�Ƶ��
#define MPU6050_INT_EN_REG				0X38	//�ж�ʹ�ܼĴ���
#define MPU6050_FIFO_EN_REG				0X23	//FIFOʹ�ܼĴ���
#define MPU6050_INTBP_CFG_REG			0X37	//�ж�/��·���üĴ���
#define MPU6050_ACCEL_XOUTH_REG		0X3B	//���ٶ�ֵ,X���8λ�Ĵ���
#define MPU6050_GYRO_XOUTH_REG		0X43	//������ֵ,X���8λ�Ĵ���
#define MPU6050_TEMP_OUTH_REG			0X41	//�¶�ֵ�߰�λ�Ĵ���

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


//��������д��һ��mpu6050���࣬����IICͨѶ��
class Mpu6050 {
private:
		bool mpu_check;
		uint8_t gyro_fsr;							//�����Ǵ����� ���� 
		uint8_t acc_fsr;							//���ٶȴ����� ����
    uint8_t gyro_buffer[7];				//ԭʼ���ٶ�  ������
    uint8_t acc_buffer[7];				//ԭʼ�ļ��ٶ�
		uint8_t mag_buffer[7];				//ԭʼ�Ĵ�����
		short temperature;					//ԭʼ�¶�
		
		float offset[6];						//����ֵ  ǰ3ΪΪ���ٶ�����ֵ����3ΪΪ���ٶ�����ֵ
		int8_t orientation[3];				//��������ֵ���������򣬷ֱ��� X Y Z ȡֵΪ 1�� -1
		int8_t megorientation[3];				//�����Ʒ�������ֵ���������򣬷ֱ��� X Z Y ȡֵΪ 1�� -1
		float megorientationoffset[3];	//�����Ƶ�����ֵ  X Z Y
		
    TwoWire Wire;

		void write_to_register(int dev_addr, uint8_t reg_addr, uint8_t reg_value);		//��Ĵ���д�������Ϣ
		
		void readGyroSource();			//��ȡmpu6050������ԭʼ����		���ٶ�ԭʼֵ		
		void readAccSource();				//��ȡmpu6050���ٶ�ԭʼֵ
		void readTempSource();			//��ȡmpu6050���¶�
		void readMagnetometer();		//��ȡ������ԭʼ����

public:
		Mpu6050(uint8_t gyrofsr=GYRO2000,uint8_t accfsr=ACC2G);
    bool mpu6050_init();			//mpu6050��ʼ��
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

