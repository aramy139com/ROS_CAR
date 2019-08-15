#ifndef _HARDSPI_H_
#define _HARDSPI_H_
#include "Gpio.h"
//Ӳ��spi
class Hardspi {
public:
    Hardspi() {		//Ĭ��ѡ��SPI1
        SPIx=SPI1;
    }
    Hardspi(SPI_TypeDef* _SPIx) {
        SPIx=_SPIx;
    }
    void setSpix(SPI_TypeDef* _SPIx) {
        SPIx=_SPIx;
    }
    void initialize(uint16_t spi_cp=SPI_CPOL_High,uint16_t spi_edg=SPI_CPHA_2Edge,uint16_t spi_baudrate=SPI_BaudRatePrescaler_256);			//��ʼ����������
		void setSpeed(uint16_t spi_baudrate);
		
		uint8_t readWriteByte(uint8_t TxData); 		// ��дһ���ֽ�

private:
    SPI_TypeDef* SPIx;
		Gpio sck;
		Gpio miso;
		Gpio mosi;
		
};
#endif // _HARDSPI_H_