#ifndef _HARDSPI_H_
#define _HARDSPI_H_
#include "Gpio.h"
//硬件spi
class Hardspi {
public:
    Hardspi() {		//默认选择SPI1
        SPIx=SPI1;
    }
    Hardspi(SPI_TypeDef* _SPIx) {
        SPIx=_SPIx;
    }
    void setSpix(SPI_TypeDef* _SPIx) {
        SPIx=_SPIx;
    }
    void initialize(uint16_t spi_cp=SPI_CPOL_High,uint16_t spi_edg=SPI_CPHA_2Edge,uint16_t spi_baudrate=SPI_BaudRatePrescaler_256);			//初始化各个参数
		void setSpeed(uint16_t spi_baudrate);
		
		uint8_t readWriteByte(uint8_t TxData); 		// 读写一个字节

private:
    SPI_TypeDef* SPIx;
		Gpio sck;
		Gpio miso;
		Gpio mosi;
		
};
#endif // _HARDSPI_H_