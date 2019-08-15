//mpu6050.cpp
#include "mpu6050.h"
#include "millisecondtimer.h"
Mpu6050::Mpu6050(uint8_t gyrofsr,uint8_t accfsr) {
    uint8_t i;
    Wire.begin();
    gyro_fsr=gyrofsr;
    acc_fsr=accfsr;
    mpu_check=false;
    for(i=0; i<6; i++) {			//³õÊ¼»¯ Æ«ÒÆÁ¿
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



//¶ÁÈ¡6050ÍÓÂİÒÇ Ô­Ê¼Êı¾İ 		½ÇËÙ¶È
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

//¶ÁÈ¡mpu6050¼ÓËÙ¶ÈÔ­Ê¼Öµ
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
//ÉèÖÃxyzÖáÏòµÄĞŞÕı·½Ïò
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
//¶ÁÈ¡mpu6050Ô­Ê¼µÄÎÂ¶È
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
    write_to_register(MPU6050_ADDRESS,MPU6050_PWR_MGMT1_REG,0X80);	//¸´Î»MPU6050
    write_to_register(MPU6050_ADDRESS,MPU6050_PWR_MGMT1_REG,0X00);	//»½ĞÑMPU6050
    write_to_register(MPU6050_ADDRESS,MPU6050_GYRO_CFG_REG,gyro_fsr);//ÉèÖÃÍÓÂİÒÇÂúÁ¿³Ì·¶Î§
    write_to_register(MPU6050_ADDRESS,MPU6050_ACCEL_CFG_REG,acc_fsr);//ÉèÖÃ¼ÓËÙ¶È´«¸ĞÆ÷ÂúÁ¿³Ì·¶Î§   +-4G
    write_to_register(MPU6050_ADDRESS,MPU6050_INT_EN_REG,0X00);	//¹Ø±ÕËùÓĞÖĞ¶Ï
    write_to_register(MPU6050_ADDRESS,MPU6050_USER_CTRL_REG,0X00);	//I2CÖ÷Ä£Ê½¹Ø±Õ
    write_to_register(MPU6050_ADDRESS,MPU6050_FIFO_EN_REG,0X00);	//¹Ø±ÕFIFO
    write_to_register(MPU6050_ADDRESS,MPU6050_INTBP_CFG_REG,0X80);	//INTÒı½ÅµÍµçÆ½ÓĞĞ§
    //ÕâÀïÊÇmpu6050µÄ²ÉÑùÂÊ  »¹Ã»Ì«Ã÷°×ÊÇ×öÊ²Ã´µÄ
    write_to_register(MPU6050_ADDRESS,MPU6050_SAMPLE_RATE_REG,0x04);	//ÉèÖÃÊı×ÖµÍÍ¨ÂË²¨Æ÷  50HZ
    write_to_register(MPU6050_ADDRESS,MPU6050_CFG_REG,0x06);//ÉèÖÃÊı×ÖµÍÍ¨ÂË²¨Æ÷

    //¼ì²éÆ÷¼şÊÇ·ñ´æÔÚ
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPU6050_DEVICE_ID_REG);
    Wire.endTransmission();
    Wire.requestFrom(MPU6050_ADDRESS, 1);
    if( Wire.read()== MPU6050_ADDRESS) {
        write_to_register(MPU6050_ADDRESS,MPU6050_PWR_MGMT1_REG,0X01);	//ÉèÖÃCLKSEL,PLL XÖáÎª²Î¿¼
        write_to_register(MPU6050_ADDRESS,MPU6050_PWR_MGMT2_REG,0X00);	//¼ÓËÙ¶ÈÓëÍÓÂİÒÇ¶¼¹¤×÷
        mpu_check=true;
        return true;
    }
    else {
        mpu_check=false;
        return false;
    }
}
//hmc5883l ³õÊ¼»¯
void Mpu6050::hmc5883l_init() {
    //³õÊ¼»¯ ´ÅÁ¦¼Æ HMC5883L
    write_to_register(HMC5883L_MAG_ADDRESS,HMC5883L_MAG_REG_A,0x18);
    /*	bit0-bit1 xyzÊÇ·ñÊ¹ÓÃÆ«Ñ¹,Ä¬ÈÏÎª0Õı³£ÅäÖÃ  bit2-bit4 Êı¾İÊä³öËÙÂÊ, 110Îª×î´ó75HZ 100Îª15HZ ×îĞ¡000 0.75HZ bit5-bit5Ã¿´Î²ÉÑùÆ½¾ùÊı 11Îª8´Î 00ÎªÒ»´Î		*/
    write_to_register(HMC5883L_MAG_ADDRESS,HMC5883L_MAG_REG_B,0x20);			//+-1.3ga  Êä³ö·¶Î§ 0xf800-0x07ff  £¨ -2048~2047 )
    //ÔöÒæÅäÖÃ 5/6/7 ÈıÎ» 000 0.88ga  1370
    // 										001 1.3ga£¨Ä¬ÈÏ£© 1090
    //										010 1.9ga  	820
    //										011 2.5ga 	660
    //										100 4ga  440
    //										101 4.7ga  390
    //										110  5.6ga  330
    //										111 8.1ga		230
    //Êä³ö·¶Î§ ¶¼ÊÇ  0xf800~0x07ff
    write_to_register(HMC5883L_MAG_ADDRESS,HMC5883L_MAG_MODE,0x00);
    megorientation[0]=1;
    megorientation[1]=1;
    megorientation[2]=1;
}

//»ñµÃÖ¸¶¨ÖáµÄ½ÇËÙ¶È   Èë¿Ú Öá  µ¥Î» ³ö¿Ú£º¶ÔÓ¦ÖáµÄ½ÇËÙ¶È  Ä¬ÈÏ·µ»ØÔ­Ê¼µÄÊı¾İ
float Mpu6050::getGyroVal(uint8_t axis,uint8_t valunit ) {
    short reval;
    float val=0.0;
    readGyroSource();
    reval=((short)((gyro_buffer[axis*2]<<8)|(gyro_buffer[axis*2+1]))+offset[axis])*orientation[axis];
    //½«Ô­Ê¼Êı¾İ°´ Á¿³Ì ×ª»»
    val=(float)reval;
    if(valunit==ORGI) {		//Ô­Ê¼Öµ
        return val;
    }
    //°´Á¿³Ì×ª»»  µ¥Î» ¶È/Ãë
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
    if(valunit==ANGLE)		//¶È/Ãë
        return val;
    else
        return val/57.295779513;		//»¡¶È/Ãë
}

//»ñµÃÖ¸¶¨Öá ¼ÓËÙ¶È   Èë¿Ú Öá  µ¥Î» ³ö¿Ú£º¶ÔÓ¦ÖáµÄ¼ÓËÙ¶È  Ä¬ÈÏ·µ»ØÔ­Ê¼µÄÊı¾İ
float Mpu6050::getAccVal(uint8_t axis,uint8_t valunit ) {
    short reval;
    float val=0.0;
    readAccSource();
    reval=((short)((acc_buffer[axis*2]<<8)|(acc_buffer[axis*2+1]))+offset[axis+3])*orientation[axis];
    //½«Ô­Ê¼Êı¾İ°´ Á¿³Ì ×ª»»
    val=(float)reval;
    if(valunit==ORGI) {		//Ô­Ê¼Öµ
        return val;
    } else {
        //°´Á¿³Ì×ª»»  µ¥Î» g
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

//»ñµÃ ÈıÖá½ÇËÙ¶È Èë¿Ú 3³¤¶È floatÊı×é£¬µ¥Î»  ¡
void Mpu6050::getGyroAllVal(float valBuf[],uint8_t valunit) {
    uint8_t i;
    short reval;
    float val=0.0;
    readGyroSource();
    for(i=0; i<3; i++) {
        reval=((short)((gyro_buffer[i*2]<<8)|(gyro_buffer[i*2+1]))+offset[i])*orientation[i];
        //½«Ô­Ê¼Êı¾İ°´ Á¿³Ì ×ª»»
        val=(float)reval;
        if(valunit==ORGI) {		//Ô­Ê¼Öµ
            valBuf[i]=val;
        } else {
            //°´Á¿³Ì×ª»»  µ¥Î» ¶È/Ãë
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
            if(valunit==ANGLE)		//¶È/Ãë
                valBuf[i]=val;
            else
                valBuf[i]=val/57.295779513;		//»¡¶È/Ãë
        }
    }
}

//»ñµÃ ÈıÖá¼ÓËÙ¶È Èë¿Ú 3³¤¶È floatÊı×é£¬µ¥Î»
void Mpu6050::getAccAllVal(float valBuf[],uint8_t valunit) {
    uint8_t i;
    short reval;
    float val=0.0;
    readAccSource();
    for(i=0; i<3; i++) {
        reval=((short)((acc_buffer[i*2]<<8)|(acc_buffer[i*2+1]))+offset[i+3])*orientation[i];
        //½«Ô­Ê¼Êı¾İ°´ Á¿³Ì ×ª»»
        val=(float)reval;
        if(valunit==ORGI) {		//Ô­Ê¼Öµ
            valBuf[i]=val;
        } else {
            //°´Á¿³Ì×ª»»  µ¥Î» g
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
//»ñÈ¡´ÅÁ¦¼ÆÔ­Ê¼Êı¾İ
//¶ÁÈ¡6050ÍÓÂİÒÇ Ô­Ê¼Êı¾İ 		½ÇËÙ¶È
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
//´ÅÁ¦¼ÆµÄĞ£×¼
/*ÒÀÎÒÀí½â£¬ÓÃµØ´Å³¡Ğ£ÕıÂŞÅÌµÄÊµÖÊÊÇÕâÑùµÄ£º
ÔÚ²»Í¬·½Ïò²âÁ¿µØ´Å³¡£¬µÃµ½¶à×é[x y z]¡£
ÀíÏëÇé¿öÏÂ£¬ÕâĞ©[x y z]Ó¦¸ÃÔÚ°ë¾¶Îª|µØ´Å³¡Ç¿¶È|µÄÇòÃæÉÏ¡£
µ«ÓÉÓÚÆ«ÒÆºÍ±ÈÀıÎó²î£¬Êµ¼ÊÌù½üµÄÊÇÒ»¸öÍÖÇòÃæ¡£
ÎÒÃÇÒª×öµÄ¾ÍÊÇÈ·¶¨Ğ£ÕıÏµÊı£¬°ÑÕâ¸öÍÖÇòÅª³ÉÔ²Çò¡£

¼òµ¥µÄĞ£ÕıµÄ¹ı³Ì¾ÍÊÇ£º
ÔÚ¿Õ¼äÉÏĞı×ª£¬ÕÒ³ö×î´ó×îĞ¡µÄx¡¢y¡¢z¡£ÕâĞ©µã¿ÉÒÔÈÏÎªÊÇÍÖÇòÃæÓë×ø±êÖáµÄ½»µã¡£
ËùÒÔ¿ÉÒÔÓÃ-(max+min)/2¼ÆËã¸÷ÖáµÄÆ«ÒÆ£¨×¢Òâ¸ººÅ£©¡£Ïàµ±ÓÚÔ­µãÏà¶ÔÓÚÍÖÇòĞÄµÄÆ«ÒÆ¡£¼ÓÉÏÕâ¸ö£¬¾ÍĞ£ÕıÁËÆ«ÒÆ¡£
ÒòÎªÖ»ĞèÒª·½Ïò£¬¿ÉÒÔÁîxÖáµÄ±ÈÀıÏµÊıx_gainÎª1¡£
È»ºóyµÄ±ÈÀıÏµÊıy_gain = x_gain * (y_max-y_min)/(x_max-x_min);
zµÄ±ÈÀıÏµÊıÀàËÆ£¬z_gain = x_gain * (z_max-z_min)/(x_max-x_min);
µ±È»£¬Í¨¹ıx_gainµÄÈ¡Öµ£¬¿ÉÒÔ°Ñ[x y z]µÄµ¥Î»Ğ£ÕıÎª1¡¢T»ò¸ßË¹¡£
*/
/*
void Mpu6050::calibrateMag() {
    float mag[3];			// X Z Y
    float xmin=0,xmax=0,ymin=0,ymax=0,zmin=0,zmax=0;
    uint16_t i=0;
    //¼Ù¶¨´ÅÁ¦¼ÆË®Æ½£¬´ÅÁ¦¼ÆµÄ½Ç¶È ¾ÍÖ»ºÍX YÓĞ¹Ø¡£ÕâÀïĞ£Õı x,y  Ğ£Õı¹ı³Ì³ÖĞø 20Ãë£¬ĞèÒªĞ¡³µ²»Í£µÄĞı×ª
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
//»ñµÃ´ÅÁ¦¼ÆĞÅÏ¢   Èë¿Ú ×°ÔØÊı¾İµÄÖ¸Õë£¬flag ÅĞ¶ÏÊÇ·ñĞèÒªĞ£Õı
void Mpu6050::getAllMagnetometer(float valBuf[],uint8_t flag) {
    uint8_t i;
    //short reval;
    float val=0.0;
    readMagnetometer();
    for(i=0; i<3; i++) {
        val=(float)(megorientation[i] * ((int16_t)((int)mag_buffer[2*i] << 8) | (mag_buffer[2*i+1]))) ;
        valBuf[i]=val*1000.00/1090.00;		//½«Ô­Ê¼Êı¾İ°´ Á¿³Ì ×ª»»  ÓëÔöÒæÖµÓĞ¹Ø
    }
    if(flag) {			//Ğ£ÕıÊı¾İ
        for(i=0; i<3; i++) {
            valBuf[i]-=megorientationoffset[i];
        }
    }
    //write_to_register(HMC5883L_MAG_ADDRESS,HMC5883L_MAG_MODE,0x01);
}