//硬件spi实现
#include "hardspi.h"
//
void Hardspi::initialize(uint16_t spi_cp,uint16_t spi_edg,uint16_t spi_baudrate) {
    SPI_InitTypeDef  SPI_InitStructure;
    //初始化时钟  & io口初始化
    if(SPIx==SPI1) {
        RCC_APB2PeriphClockCmd(	RCC_APB2Periph_SPI1,  ENABLE );//SPI1时钟使能
        sck=Gpio(PA,5,GM_AFPP);
        miso=Gpio(PA,6,GM_AFPP);
        mosi=Gpio(PA,7,GM_AFPP);
    } else {
        RCC_APB1PeriphClockCmd(	RCC_APB1Periph_SPI2,  ENABLE );//SPI2时钟使能
        sck=Gpio(PB,13,GM_AFPP);
        miso=Gpio(PB,14,GM_AFPP);
        mosi=Gpio(PB,15,GM_AFPP);
    }
    sck.high();
    miso.high();
    mosi.high();  //上拉

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//设置SPI工作模式:设置为主SPI
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//设置SPI的数据大小:SPI发送接收8位帧结构
    SPI_InitStructure.SPI_CPOL = spi_cp;		//串行同步时钟的空闲状态为高电平
    SPI_InitStructure.SPI_CPHA = spi_edg;	//串行同步时钟的第二个跳变沿（上升或下降）数据被采样
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
    SPI_InitStructure.SPI_BaudRatePrescaler = spi_baudrate;		//定义波特率预分频的值:波特率预分频值为256
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
    SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC值计算的多项式

    if(SPIx==SPI1) {
        SPI_Init(SPI1, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
        SPI_Cmd(SPI1, ENABLE); //使能SPI外设
        readWriteByte(0xff);//启动传输
    } else {
        SPI_Init(SPI2, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
        SPI_Cmd(SPI2, ENABLE); //使能SPI外设
        readWriteByte(0xff);//启动传输
    }
}

//SPI 速度设置函数
//SpeedSet:
//SPI_BaudRatePrescaler_2   2分频
//SPI_BaudRatePrescaler_8   8分频
//SPI_BaudRatePrescaler_16  16分频
//SPI_BaudRatePrescaler_256 256分频
void Hardspi::setSpeed(uint16_t spi_baudrate) {
    assert_param(IS_SPI_BAUDRATE_PRESCALER(spi_baudrate));
    SPI1->CR1&=0XFFC7;
    SPI1->CR1|=spi_baudrate;	//设置SPI1速度
    SPI_Cmd(SPI1,ENABLE);
}

//SPIx 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
uint8_t Hardspi::readWriteByte(uint8_t TxData) {
    u8 retry=0;
    if(SPIx==SPI1) {
        while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) //检查指定的SPI标志位设置与否:发送缓存空标志位
        {
            retry++;
            if(retry>200)return 0;
        }
        SPI_I2S_SendData(SPI1, TxData); //通过外设SPIx发送一个数据
        retry=0;

        while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)//检查指定的SPI标志位设置与否:接受缓存非空标志位
        {
            retry++;
            if(retry>200)return 0;
        }
        return SPI_I2S_ReceiveData(SPI1); //返回通过SPIx最近接收的数据
    } else {
        while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) //检查指定的SPI标志位设置与否:发送缓存空标志位
        {
            retry++;
            if(retry>200)return 0;
        }
        SPI_I2S_SendData(SPI2, TxData); //通过外设SPIx发送一个数据
        retry=0;

        while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)//检查指定的SPI标志位设置与否:接受缓存非空标志位
        {
            retry++;
            if(retry>200)return 0;
        }
        return SPI_I2S_ReceiveData(SPI2); //返回通过SPIx最近接收的数据
    }
}