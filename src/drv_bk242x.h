#pragma once	  		 
				
/* 参数配置 ------------------------------------------------------------------*/
#define BK242X_PAYLOAD_LENGTH  15     //射频帧长度
#define BK242X_ADDR_LENGTH     5
#define BK242X_CHANNEL_SUM     16     

/*----------------------------------------------------------------------------*/		
typedef struct {   	
} RFFlags;	
 
typedef struct {
} RFVariable;			

typedef struct {
	GPIO_TypeDef *CS_Port, *CE_Port, *RXEN_Port;             //GPIO
	GPIO_Pin CS_Pin, CE_Pin, RXEN_Pin;	
	
  uint8_t addr[BK242X_ADDR_LENGTH];			                   //Address
	uint8_t txBuffer[BK242X_PAYLOAD_LENGTH];
	uint8_t rxBuffer[BK242X_PAYLOAD_LENGTH];
	
  uint8_t  txTimer;                                        //Variables
  uint8_t  rxTimer;
  uint8_t  frequencyHopTimer;
  uint16_t frequencyOccupyTimer;
  uint8_t  bindingTimer;
 
  uint8_t  txPeriod;
  uint8_t  rxPeriod;
  uint8_t  frequencyHopPeriod;
 
  uint8_t  channelIndex;
  uint8_t  channel;
  uint8_t  channelNext;
  uint8_t  channelNextNext;
	uint8_t  channelEffect;
		
	uint8_t  bTxReday:1;                                     //Flags
	uint8_t  bRxReday:1;
  uint8_t  bBinding:1; 	
	uint8_t  bFrequencyAlternative:1;	
	
} Bk242xDevice;	 

/*----------------------------------------------------------------------------*/				
void bk242xWriteReg(Bk242xDevice *Dev, uint8_t Addr, uint8_t Data);
void bk242xWriteBuf(Bk242xDevice *Dev, uint8_t Addr, uint8_t* Buf, uint8_t Length);
void bk242xWriteCmd(Bk242xDevice *Dev, uint8_t CMD, uint8_t ARG);
uint8_t bk242xReadReg(Bk242xDevice *Dev, uint8_t Addr);
void bk242xWritePayload(Bk242xDevice *Dev, uint8_t Style, uint8_t* Buf, uint8_t Length);
void bk242xReadPayload(Bk242xDevice *Dev, uint8_t Style, uint8_t* Buf, uint8_t Length);

void bk242xPowerDown(Bk242xDevice *Dev);
void bk242xStandby(Bk242xDevice *Dev); 
void bk242xClearFlags(Bk242xDevice *Dev);
void bk242xFlushTx(Bk242xDevice *Dev);
void bk242xFlushRx(Bk242xDevice *Dev);
void bk242xTxStart(Bk242xDevice *Dev);
void bk242xRxStart(Bk242xDevice *Dev);

void bk242xWriteAddr(Bk242xDevice* Dev, uint8_t* buf, uint8_t length);

void bk242xInit(Bk242xDevice *Dev);
void bk242xTxCtrl(Bk242xDevice *Dev);
void bk242xRxCtrl(Bk242xDevice *Dev);
void bk242xSendPackage(Bk242xDevice* Dev, uint8_t* buffer, uint16_t length);


