
/***************************************************************************************************
						Bk2423 各子函数
***************************************************************************************************/
#include "board.h"

//================================================================================================================	
//================================================================================================================ 
//SPI(Bk2423) commands 
 #define 	BK242X_READ_REG     0x00  // Define read command to register 
 #define	BK242X_WRITE_REG    0x20  // Define write command to register 
 #define	RD_RX_PLOAD	        0x61  // Define RX payload register address 
 #define	WR_TX_PLOAD	        0xA0  // Define TX payload register address 
 #define	FLUSH_TX	          0xE1  // Define flush TX register command 
 #define	FLUSH_RX	          0xE2  // Define flush RX register command 
 #define	REUSE_TX_PL	        0xE3  // Define reuse TX payload register command 
 #define	W_TX_PAYLOAD_NOACK	0xb0 
 #define	W_ACK_PAYLOAD	      0xa8 
 #define	ACTIVATE	          0x50 
 #define	R_RX_PL_WID	        0x60 
 #define	BK242X_NOP          0xFF  // Define No Operation, might be used to read status register 

// SPI(Bk2423) registers(addresses) 
 #define	CONFIG							0x00  // 'Config' register address 
		#define	bMaskAllIRQ				0x70	//01110000b
		#define	bEnCrc						0x08	//00001000b
		#define	bCrc1byte					0			//00000000b
		#define	bCrc2byte					0x04	//00000100b
		#define	bPwrUp						0x02	//00000010b
		#define	bPRX						  0x01	//00000001b
		#define	bPTX						  0			//00000000b
 #define	EN_AA	              0x01  //'Enable Auto Acknowledgment' register address 
 #define	EN_RXADDR	          0x02  //'Enabled RX addresses' register address 
 #define	SETUP_AW	          0x03  //'Setup address width' register address 
 #define	SETUP_RETR	        0x04  //'Setup Auto. Retrans' register address 
 #define	BK242X_CH           0x05  //'RF channel' register address 
 #define	BK242X_SETUP        0x06  //'RF setup' register address 
 #define	BK242X_STATUS       0x07  //'Status' register address 
		#define	bRXDR							0x40	//01000000b
		#define	bTXDS							0x20	//00100000b
		#define	bMAXRT						0x10	//00010000b
 #define	OBSERVE_TX					0x08  //'Observe TX' register address 
 #define	CD									0x09  //'Carrier Detect' register address 
 #define	RX_ADDR_P0 					0x0A  //'RX address pipe0' register address 
 #define	RX_ADDR_P1					0x0B  //'RX address pipe1' register address 
 #define	RX_ADDR_P2					0x0C  //'RX address pipe2' register address 
 #define	RX_ADDR_P3 					0x0D  //'RX address pipe3' register address 
 #define	RX_ADDR_P4 					0x0E  //'RX address pipe4' register address 
 #define	RX_ADDR_P5					0x0F  //'RX address pipe5' register address 
 #define	TX_ADDR 						0x10  //'TX address' register address 
 #define	RX_PW_P0						0x11  //'RX payload width, pipe0' register address 
 #define	RX_PW_P1						0x12  //'RX payload width, pipe1' register address 
 #define	RX_PW_P2						0x13  //'RX payload width, pipe2' register address 
 #define	RX_PW_P3						0x14  //RX payload width, pipe3' register address 
 #define	RX_PW_P4		        0x15  //'RX payload width, pipe4' register address 
 #define	RX_PW_P5		        0x16  //'RX payload width, pipe5' register address 
 #define	FIFO_STATUS		      0x17  //'FIFO Status Register' register address 
 
 #define	DYNPD			          0x1c
 #define	FEATURE			        0x1d
 
 //bank1
 #define	RAMP 			          0x0e
 
 // user data
 #define	BANK0			          0			//00000000b
 #define	BANK1			          0x80	//10000000b
 				  
 #define	CONFIG_INIT		      0x0f  //Enb 中断、2bytes CRC,RX,Power On

//================================================================================================================
//寄存器配置
//================================================================================================================
#define RF_STYLE_BK2423

//In the array Bank1_Reg0_13,all[] the register value is the byte reversed!!!!!!!!!!!!!!!!!!!!!
const uint32_t Bank1_Reg0_13[]={  
#ifndef RF_STYLE_BK2423   //Bk2425  
	0xE2014B40,						//REG0
	0x00004BC0,						//REG1
	0x028CFCD0,						//REG2
	0x21390099,						//REG3,120517	
	0x1B8296F9,						/*REG4,120517	 		      1Mbps/2Mbps	       250Kbps
										      Normal Mode		        0x1B8296D9	       0x1B8AB6D9
										      High Power Mode		    0x1B8296F9	       0x1B8AB6F9
										      CW Normal Mode		    0x218296D9	       0x218AB6D9
										      CW High Power Mode	  0x218296F9	       0x218AB6F9	*/	 	
	0xA60F0624,						//REG5,120517,to enable RSSI measurement,use 0xA67F023C(maxium vthcd)
									      //0xA60F0624(20140813)
	0x00000000,						//6
	0x00000000,						//7
	0x00000000,						//8
	0x00000000,						//9
	0x00000000,						//10
	0x00000000,						//11
	0x00127300, 					/*REG12,120517
										      0x00127300:PLL locking time 120us compatible with BK2421;
										      0x00127305(chip default):PLL locking time 130us compatible with nRF24L01;	*/
	0x36B48000,						//REG13,120517	 	 0x46B48000
									      //	0x36B48000(20140813)	
#else  					        //Bk2423				
	0xE2014B40,						//REG0
	0x00004BC0,						//REG1
	0x028CFCD0,						//REG2
	0x21390099,						//REG3,120517	
	0x1B8296F9,						/*REG4,120517	 		1Mbps/2Mbps	        250Kbps
										      Normal Mode		    0x1B8296D9	       0x1B8AB6D9
										      High Power Mode		0x1B8296F9	       0x1B8AB6F9
										      CW Normal Mode		0x218296D9	       0x218AB6D9
										      CW High Power Mode	0x218296F9	       0x218AB6F9	*/	 	
	0xA67F0624,						//REG5,120517,to enable RSSI measurement,use 0xA67F023C(maxium vthcd)
	0x00000000,						//6
	0x00000000,						//7
	0x00000000,						//8
	0x00000000,						//9
	0x00000000,						//10
	0x00000000,						//11
	0x00127300, 					/*REG12,120517
										      0x00127300:PLL locking time 120us compatible with BK2421;
										      0x00127305(chip default):PLL locking time 130us compatible with nRF24L01;	*/		
	0x46B48000,						//REG13,120517
#endif	
};

const uint8_t Bank1_Reg14[] =
{
#ifndef RF_STYLE_BK2423	//Bk2425
	0x41,0x20,0x08,0x04,0x81,0x20,0xcf,0xf7,0xfe,0xff,0xff		//0x41 20 08 04 81 20 CF F7 FE FF FF(20140815)
#else						        //Bk2423  		  
	0x41,0x10,0x04,0x82,0x20,0x08,0x08,0xF2,0x7D,0xEF,0xFF		//0x41,0x10,0x04,0x82,0x20,0x08,0x08,0xF2,0x7D,0xEF,0xFF
#endif
};

//Bank0 register initialization value
const uint8_t Bank0_Reg[10][2] = {
	{RX_PW_P0,		  BK242X_PAYLOAD_LENGTH}, 	//0		数据长度6~32字节	
	{BK242X_STATUS,	0x70}, 			              //1		清所有状态标志
	{BK242X_SETUP,	0x05}, 			              //2		1Mbps, 0dBm，High gain
	{SETUP_RETR,	  0x53}, 			              //3		间隔1500us 重发3次
	{SETUP_AW,		  0x03}, 			              //4		地址宽度5字节
	{EN_RXADDR,		  0x01}, 			              //5		只使能pip0 接收
	{EN_AA,			    0x01},  		              //6		Enb Auto ACK
	{CONFIG,		    0x0d}, 			              //7		Enable中断,2 Byte CRC、Rx、Power Off

	{FEATURE,		    0x04}, 			              //8		no dpl,no ack_pay,dis W_TX_PAYLOAD_NOACK command
	{DYNPD,			    0x01}, 			              //9
};		    


const uint8_t bk242xFrequencyTable[BK242X_CHANNEL_SUM] = {	
				46,70,62,54,
        46,70,62,54,	   
	      46,70,62,54,
	      46,70,62,54,	
      };			   
							


//================================================================================================================	
//						子函数
//================================================================================================================ 
/* 端口操作 ----------------------------------------------------------------*/
/** @Brief 端口操作
  *
  */
void bk242xChipEnable(Bk242xDevice *Dev, uint8_t enable)
{
    if (!Dev)
        return;
		if (!Dev->CE_Port || !Dev->CE_Pin) 
			  return;
 
    if (enable) {
        digitalHi(Dev->CE_Port, Dev->CE_Pin);
    } else {
        digitalLo(Dev->CE_Port, Dev->CE_Pin);
    }
}
void bk242xRXEnable(Bk242xDevice *Dev, uint8_t enable)
{
    if (!Dev)
        return;
		if (!Dev->RXEN_Port || !Dev->RXEN_Pin) 
			  return;
 
    if (enable) {
        digitalHi(Dev->RXEN_Port, Dev->RXEN_Pin);
    } else {
        digitalLo(Dev->RXEN_Port, Dev->RXEN_Pin);
    }
}

		
/* 寄存器读写函数 ----------------------------------------------------------------*/
/** @Brief RF读写函数
  *
  */
//写寄存器
void bk242xWriteReg(Bk242xDevice *Dev, uint8_t Addr, uint8_t Data) {	
    if (!Dev)
        return;
		
    spiSelect(true, Dev->CS_Port, Dev->CS_Pin);
	  spiTransferByte(BK242X_WRITE_REG | Addr);
	  spiTransferByte(Data);	
    spiSelect(false, Dev->CS_Port, Dev->CS_Pin);	
}

void bk242xWriteBuf(Bk242xDevice *Dev, uint8_t Addr, uint8_t* Buf, uint8_t Length) {
    if (!Dev)
        return;
		
    spiSelect(true, Dev->CS_Port, Dev->CS_Pin);
	  spiTransferByte(BK242X_WRITE_REG | Addr);
	  spiTransfer(0, Buf, Length);	
    spiSelect(false, Dev->CS_Port, Dev->CS_Pin);	
}

//写命令
void bk242xWriteCmd(Bk242xDevice *Dev, uint8_t CMD, uint8_t ARG) {
    if (!Dev)
        return;
		
    spiSelect(true, Dev->CS_Port, Dev->CS_Pin);
	  spiTransferByte(CMD);
	  spiTransferByte(ARG);	
    spiSelect(false, Dev->CS_Port, Dev->CS_Pin);	
}

//读寄存器
uint8_t bk242xReadReg(Bk242xDevice *Dev, uint8_t Addr) {
	  uint8_t data;
	
    if (!Dev)
        return 0;
		
    spiSelect(true, Dev->CS_Port, Dev->CS_Pin);
	  spiTransferByte(BK242X_READ_REG | Addr);
	  data = spiTransferByte(BK242X_NOP);	
    spiSelect(false, Dev->CS_Port, Dev->CS_Pin);	
	
    return data;
}

//读写收发缓冲区
void bk242xWritePayload(Bk242xDevice *Dev, uint8_t Style, uint8_t* Buf, uint8_t Length){
    if (!Dev)
        return;
		
    spiSelect(true, Dev->CS_Port, Dev->CS_Pin);
	  spiTransferByte(Style);
	  spiTransfer(0, Buf, Length);	
    spiSelect(false, Dev->CS_Port, Dev->CS_Pin);	
}
void bk242xReadPayload(Bk242xDevice *Dev, uint8_t Style, uint8_t* Buf, uint8_t Length){
    if (!Dev)
        return;
		
    spiSelect(true, Dev->CS_Port, Dev->CS_Pin);
	  spiTransferByte(Style);
	  spiTransfer(Buf, 0, Length);	
    spiSelect(false, Dev->CS_Port, Dev->CS_Pin);	
}


/* 功能函数 ----------------------------------------------------------------*/
/** @brief 功能函数
  *
  */  
//休眠
void bk242xPowerDown(Bk242xDevice *Dev)
{	
	bk242xChipEnable(Dev, false);
	bk242xRXEnable(Dev, false);	
	bk242xWriteReg(Dev, CONFIG, 0x7c); 
}	

//待机
void bk242xStandby(Bk242xDevice *Dev) {	  
	bk242xChipEnable(Dev, false);
	bk242xRXEnable(Dev, false);	
	bk242xWriteReg(Dev, CONFIG, 0x0d);
	delay(1);
}
											
//清标志位	   
void bk242xClearFlags(Bk242xDevice *Dev)
{							  
	bk242xWriteReg(Dev, BK242X_STATUS, 0x70);
}
 
//清发射缓冲区
void bk242xFlushTx(Bk242xDevice *Dev)
{	
	bk242xWriteCmd(Dev, FLUSH_TX, BK242X_NOP);
}

//清接收缓冲区
void bk242xFlushRx(Bk242xDevice *Dev)
{	
	bk242xWriteCmd(Dev, FLUSH_RX, BK242X_NOP);
}

//启动发射
void bk242xTxStart(Bk242xDevice *Dev)
{
	bk242xWriteReg(Dev, CONFIG, CONFIG_INIT&0xfe);		
	bk242xChipEnable(Dev, true);
	bk242xRXEnable(Dev, true);
}

//启动接收
void bk242xRxStart(Bk242xDevice *Dev)
{
	bk242xWriteReg(Dev, CONFIG, CONFIG_INIT|0x01);		
	bk242xChipEnable(Dev, true);
	bk242xRXEnable(Dev, true);
}

void bk242xWriteAddr(Bk242xDevice* Dev, uint8_t* buf, uint8_t length) {	   
	bk242xWriteBuf(Dev, TX_ADDR,    buf, 5);	
	bk242xWriteBuf(Dev, RX_ADDR_P0, buf, 5); 
}

//--------------------------------------------------------------
//				bank切换
//		读0x07.7,判断当前bank与目标值是否相同
//		若相同,退出;不同,切换bank
//--------------------------------------------------------------
void bk242xBankSelect(Bk242xDevice *Dev, uint8_t bank)
{	
	uint8_t bank_status;

	bank_status = bk242xReadReg(Dev, 0x07); 
	bank_status &= (uint8_t)0x80;
	
	if(bank_status != bank)
	{
		//当前bank与目标bank不同
		//写入ACTIVATE_CMD+0x53切换 				
		bk242xWriteCmd(Dev, ACTIVATE, 0x53);
	}
}


//================================================================================================================ 
//芯片初始化
//================================================================================================================ 
uint8_t bk242xGetRandomFreq(Bk242xDevice *Dev);
uint8_t bk242xGetNextRandomFreq(Bk242xDevice *Dev);
uint8_t bk242xGetNNextRandomFreq(Bk242xDevice *Dev);

/** @Brief 初始化Bk242x 2.4G芯片
  * 1 初始化控制口(除SPI)
  * 2 初始化Bk242x内部寄存器。1M速率、5dB
  * 3 写入五位地址(由函数传参Dev->addr[5])
  * 4 待机模式返回
  */ 
void bk242xInit(Bk242xDevice *Dev)
{				
  gpio_config_t gpio;
 	uint8_t WriteArr[4];
	uint8_t i,j; 

	if(!Dev)
		return;
	
	//1) 上电延时
	//2) IO初始化，CS\CE\RXEN 	
	if(Dev->CS_Port && Dev->CS_Pin) {
    gpio.pin = Dev->CS_Pin;
    gpio.mode = Mode_Out_PP;
    gpio.speed = Speed_50MHz;
    gpioInit(Dev->CS_Port, &gpio);
		digitalHi(Dev->CS_Port, Dev->CS_Pin);
	}
	if(Dev->CE_Port && Dev->CE_Pin) {
    gpio.pin = Dev->CE_Pin;
    gpio.mode = Mode_Out_PP;
    gpioInit(Dev->CE_Port, &gpio);		
    digitalLo(Dev->CE_Port, Dev->CE_Pin);
	}
	if(Dev->RXEN_Port && Dev->RXEN_Pin) {
    gpio.pin = Dev->RXEN_Pin;
    gpio.mode = Mode_Out_PP;
    gpioInit(Dev->RXEN_Port, &gpio);		
    digitalLo(Dev->RXEN_Port, Dev->RXEN_Pin);
	}
	delay(100);  	
  	
	//1) Bank0
	//2) 定义数据长度工作模式等，详见数据定义 
	bk242xBankSelect(Dev, BANK0);	 	
	for(i=0; i<8; i++)
		bk242xWriteReg(Dev, Bank0_Reg[i][0], Bank0_Reg[i][1]);

	i = bk242xReadReg(Dev, FEATURE);	
	if(i==0) 								            // i!=0 showed that chip has been actived.so do not active again.
		bk242xWriteCmd(Dev, ACTIVATE, 0x73);	// Active
	
	for(i=8; i<10; i++)
		bk242xWriteReg(Dev, Bank0_Reg[i][0], Bank0_Reg[i][1]);

	//1) Bank1
	//2) 按照芯片要求，固定值	
	bk242xBankSelect(Dev, BANK1);
	for(i=0; i<=8; i++){					      //reverse
		for(j=0; j<4; j++) WriteArr[j] = (Bank1_Reg0_13[i]>>(8*(j))) & 0xff;	 		
		bk242xWriteBuf(Dev, i, &(WriteArr[0]), 4);
	}
	for(i=9; i<=13; i++){
		for(j=0;j<4;j++) WriteArr[j] = (Bank1_Reg0_13[i]>>(8*(3-j))) & 0xff;
		bk242xWriteBuf(Dev, i, &(WriteArr[0]), 4);
	}
	bk242xWriteBuf(Dev, 14, (uint8_t *)&(Bank1_Reg14[0]), 11);

	//toggle REG4<25,26>
	for(j=0; j<4; j++) WriteArr[j]=( Bank1_Reg0_13[4]>>(8*(j)) ) & 0xff;

	WriteArr[0] = WriteArr[0]|0x06;
	bk242xWriteBuf(Dev, 4, &(WriteArr[0]), 4);	
	WriteArr[0] = WriteArr[0] & 0xf9;
	bk242xWriteBuf(Dev, 4, &(WriteArr[0]), 4);

	//1) 切换回Bank0
	//2) 写入地址，待机，返回	
	delay(20);   	
	bk242xBankSelect(Dev, BANK0);
	delay(20);   	
								   
	bk242xWriteBuf(Dev, TX_ADDR,    Dev->addr, 5);	 
	bk242xWriteBuf(Dev, RX_ADDR_P0, Dev->addr, 5);		 
} 

/** @brief 数据发送(Dev->txBuffer)
  * 1  每隔Dev->txPeriod调用一次
  * 2  若发射成功置标志位Dev->bTxReday = 1
  * 3  持续发射
  */
void bk242xTxCtrl(Bk242xDevice *Dev) {	  	 
    if(Dev->txTimer >= Dev->txPeriod) {
      Dev->txTimer = 0;
      Dev->txPeriod = 7;                                             //TXCtrl调用周期7ms
			Dev->frequencyHopPeriod = 40 + ((uint8_t)micros() & 0x0f);		 //跳频周期40~55ms
			
		  //Part1 检测发射状态
		  //  1) 发射成功，更新标志位
      if(bk242xReadReg(Dev, BK242X_STATUS) & (uint8_t)bTXDS){
        Dev->bTxReday = 1;
        Dev->frequencyHopTimer = 0;					
      }
      bk242xFlushTx(Dev);		 	 
      bk242xStandby(Dev);
      bk242xClearFlags(Dev);		
			 
		  //Part2 跳频
      //  1) 跳频
	    //  2) 若要禁止跳频(对码时)，可使:
		  //     aDev->frequencyHopTimer = 0;
		  //     bDev->frequencyOccupyTimer = 0;
      if((Dev->frequencyHopTimer >= Dev->frequencyHopPeriod) || (Dev->frequencyOccupyTimer >= 360)) {		
        Dev->frequencyHopTimer = 0;						 
        Dev->frequencyOccupyTimer = 0;	 

        Dev->channel = bk242xGetRandomFreq(Dev);
        bk242xWriteReg(Dev, BK242X_CH, Dev->channel);
      }	

		  //Part3 重启发射或进入待机状态
		  //  1) 
      //  2) 				
      bk242xWritePayload(Dev, WR_TX_PLOAD, Dev->txBuffer, BK242X_PAYLOAD_LENGTH);
      bk242xTxStart(Dev);	
		} 
}
  		
/** @brief 数据接收(Dev->rxBuffer)
  * 1 持续接收，接收成功置标志位 Dev->bRxReday
  *
  */
void bk242xRxCtrl(Bk242xDevice *Dev) {
	  if(Dev->rxTimer >= Dev->rxPeriod){
		  Dev->rxTimer = 0;
      Dev->rxPeriod = 5;
		
		  //Part1 读接收状态
      //  1) 读状态寄存器，判断是否接收完成
		  //  2) 若接收完成，置完成标志位、清跳频计时
		  //  3) 更新有效频点为当前工作频点
		  if(bk242xReadReg(Dev, BK242X_STATUS) & (uint8_t)bRXDR){	 	 
        bk242xStandby(Dev);
        bk242xReadPayload(Dev, RD_RX_PLOAD, Dev->rxBuffer, BK242X_PAYLOAD_LENGTH);	 
							
        Dev->bRxReday = 1;
			  Dev->frequencyHopTimer = 0;
			  Dev->bFrequencyAlternative = 0;
			
			  if(Dev->channel != bk242xFrequencyTable[Dev->channelIndex]){
			  	if(Dev->channel == Dev->channelNext) bk242xGetRandomFreq(Dev); 
			  	else if(Dev->channel == Dev->channelNextNext){
			  		bk242xGetRandomFreq(Dev);
				  	bk242xGetRandomFreq(Dev);
			  	} 
			  } 
		  	Dev->channelEffect = Dev->channel;	
		  }		
		
 		//1) 若没有成功接收，开始跳频
		//2) 前50ms，循环工作于当前频点和下一个频点
		//3) 50~100ms，循环工作于下一个频点和下下个频点
		//4) 大于100ms，逐个扫描频率表
		//5) 若要禁止跳频(对码时)，可使:
		//*    a、Dev->frequencyHopTimer = 0;
		//*    b、rfFlag->bFrequencyAlternative = 0;
		//*    c、Dev->channelEffect = Dev->channel;
		//*
		if(!Dev->bRxReday){	
      bk242xStandby(Dev);  
      bk242xFlushRx(Dev);	
      bk242xClearFlags(Dev);				
		
			Dev->channelNext = bk242xGetNextRandomFreq(Dev);		 	 	
			Dev->channelNextNext = bk242xGetNNextRandomFreq(Dev);   

			if(Dev->frequencyHopTimer < 50){				  				
				Dev->channel = Dev->channelEffect;
				if(Dev->bFrequencyAlternative) Dev->channel = Dev->channelNext;	
			} else if(Dev->frequencyHopTimer < 100){ 
				Dev->channel = Dev->channelNext;
				if(Dev->bFrequencyAlternative) Dev->channel = Dev->channelNextNext;			
			} else {
				Dev->frequencyHopTimer = 100;
				Dev->channel = bk242xGetRandomFreq(Dev);
			}	
		
			Dev->bFrequencyAlternative = ~Dev->bFrequencyAlternative;
			bk242xWriteReg(Dev, BK242X_CH, Dev->channel);
		}
		
		//1) 重启接收
		bk242xFlushRx(Dev);
 		bk242xRxStart(Dev);			 
	}
} 		

//================================================================================================================ 
//================================================================================================================ 
/** @brief 查表返回频率
  */
uint8_t bk242xGetRandomFreq(Bk242xDevice *Dev) {	 	    //更新指针
  if(++Dev->channelIndex >= BK242X_CHANNEL_SUM) Dev->channelIndex = 0; 
  return bk242xFrequencyTable[Dev->channelIndex];
} 
uint8_t bk242xGetNextRandomFreq(Bk242xDevice *Dev) {	 	//不更新指针
  uint8_t i;	
	i = Dev->channelIndex;
	if(++i > BK242X_CHANNEL_SUM) i = 0;
  return bk242xFrequencyTable[i];
} 
uint8_t bk242xGetNNextRandomFreq(Bk242xDevice *Dev) {	 	//不更新指针 
  uint8_t i;	
	i = Dev->channelIndex;
	if(++i > BK242X_CHANNEL_SUM) i = 0;
	if(++i > BK242X_CHANNEL_SUM) i = 0;
  return bk242xFrequencyTable[i];
} 

void bk242xSendPackage(Bk242xDevice* Dev, uint8_t* buffer, uint16_t length){
	uint16_t i;
	if(length > BK242X_PAYLOAD_LENGTH) length = BK242X_PAYLOAD_LENGTH;	
	for(i = 0; i < length; i++){
		Dev->txBuffer[i] = buffer[i];
	}
}

