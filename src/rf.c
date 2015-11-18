#include "board.h"
#include "mw.h"
#include "buzzer.h"

#define	RF_CODE_NULL					0
#define	RF_SHANKHAND_FIRST		0xf1
#define	RF_SHANKHAND_SECOND		0xf2
#define	RF_SHANKHAND_THRID		0xf3
#define	RF_CODE_NORMAL				0xf4
#define	RF_CODE_DATA_RETURN		0xf5

#define	RF_PUBLIC_ADDR0       0x75
#define	RF_PUBLIC_ADDR1       0x75
#define	RF_PUBLIC_ADDR2       0x75
#define	RF_PUBLIC_ADDR3       0x75
#define	RF_PUBLIC_ADDR4       0x04

#define	RF_PUBLIC_ADDR_TX	 	  0x81  	
#define RF_PUBLIC_ADDR_RX     0x01


RFDevice RFDev[RF_DEV_SUM];  

//================================================================================================================ 
//
//================================================================================================================ 
void rfInit(void) 
{
	  uint8_t i;
	
	  //-----------------------------------------------------------------------------------------
	  //Bk2423PA 
    RFDev[RFPA].CE_Port = GPIOB;
	  RFDev[RFPA].CE_Pin = Pin_4;
    RFDev[RFPA].CS_Port = GPIOB;
	  RFDev[RFPA].CS_Pin = Pin_3;
    RFDev[RFPA].RXEN_Port = GPIOD;
	  RFDev[RFPA].RXEN_Pin = Pin_2;	
	  for(i = 0; i < BK242X_ADDR_LENGTH; i++) {
			  RFDev[RFPA].addr[i] = cfg.RFTxAddr[i];
		}
	
	  //Bk2423 不带PA
    RFDev[RFRx].CE_Port = GPIOC;
	  RFDev[RFRx].CE_Pin = Pin_12;
    RFDev[RFRx].CS_Port = GPIOC;
	  RFDev[RFRx].CS_Pin = Pin_11;
    RFDev[RFRx].RXEN_Port = NULL;
	  RFDev[RFRx].RXEN_Pin = NULL;	
	  for(i = 0; i < BK242X_ADDR_LENGTH; i++) {
			  RFDev[RFRx].addr[i] = cfg.RFRxAddr[i];
		}	
		
	  //-----------------------------------------------------------------------------------------
	  bk242xInit(&RFDev[RFPA]);
		bk242xInit(&RFDev[RFRx]);
		
		RFDev[RFPA].bBinding = 0;
		RFDev[RFRx].bBinding = 1;
}

//================================================================================================================ 
//
//================================================================================================================ 
typedef enum {
	  SH_RESET = 0,
		SH_FIRST,
		SH_SECOND,
		SH_THIRD,
	  SH_CMP
} Shankhand;

/** @Brief 对码
  *   1) 用在发射端(通常指遥控器)
  *   2) 与之对应的接收端对码程序为 RFRxBind()
  *
  * @Note 
  * @ RF_PUBLIC_ADDR_TX, RF_PUBLIC_ADDR_RX，有两个作用:
  *   1)、区分遥控端和接收端。
  *   2)、区分不同遥控器，不同接收端。
  *   取值范围: RF_PUBLIC_ADDR_TX = 10000001 ~ 11111111; RF_PUBLIC_ADDR_RX = 00000001 ~ 01111111。
  *
  */ 
void RFTxBind(Bk242xDevice *Dev){	
	  static uint8_t buffer[8];
	  static Shankhand TxBindStat = SH_RESET;
	  static uint8_t noSignalTimer = 0;
	  uint8_t i;
	
	  if(!Dev->bBinding) return;
	
		switch(TxBindStat) {
		  	case SH_RESET:
		      bk242xFlushTx(Dev);
		      bk242xStandby(Dev);
		      bk242xFlushRx(Dev);
		      bk242xClearFlags(Dev);			  
			
	        buffer[0] = RF_PUBLIC_ADDR0; 
		      buffer[1] = RF_PUBLIC_ADDR1;
		      buffer[2] = RF_PUBLIC_ADDR2;
		      buffer[3] = RF_PUBLIC_ADDR3;
		      buffer[4] = RF_PUBLIC_ADDR4;  								
			    bk242xWriteAddr(Dev, buffer, 5);
			
		      buffer[0] = RF_SHANKHAND_FIRST;
		      buffer[1] = Dev->addr[0];
	        buffer[2] = Dev->addr[1];
		      buffer[3] = Dev->addr[2];
	        buffer[4] = Dev->addr[3];
		      buffer[5] = RF_PUBLIC_ADDR_TX;			
		      bk242xSendPackage(Dev, buffer, 6);
			
					noSignalTimer = 0;
					Dev->bindingTimer = 0;
			    Dev->bTxReday = 0;
			    TxBindStat = SH_FIRST;					
				  break;
				case SH_FIRST:
					bk242xTxCtrl(Dev); 
				
				  if(Dev->bTxReday){
		        bk242xFlushTx(Dev);
		        bk242xStandby(Dev);
		        bk242xClearFlags(Dev);
		        buffer[0] = Dev->addr[0];
	          buffer[1] = Dev->addr[1];
		        buffer[2] = Dev->addr[2];
	          buffer[3] = Dev->addr[3];
		        buffer[4] = RF_PUBLIC_ADDR_TX;							
			      bk242xWriteAddr(Dev, buffer, 5);	
						
						Dev->rxPeriod = Dev->rxTimer;
						Dev->bRxReday = 0;					
						Dev->bTxReday = 0;							
		        Dev->bindingTimer = 0; 
						TxBindStat = SH_SECOND;
						break;
					}
					
					if(Dev->bindingTimer >= 100) {
						Dev->bindingTimer = 0;						
					  if(noSignalTimer++ >= 10) {
							noSignalTimer = 0;
              bk242xInit(&RFDev[RFPA]);							
					  }
					}
				  break;
				case SH_SECOND:	
		      Dev->channelEffect = Dev->channel;                      //禁用跳频
		      Dev->frequencyHopTimer = 0;
		      Dev->bFrequencyAlternative = 0;				
					bk242xRxCtrl(Dev);  
				  
					if(Dev->bRxReday && Dev->rxBuffer[0] == RF_SHANKHAND_SECOND){
						buffer[1] = Dev->rxBuffer[1];
						buffer[2] = Dev->rxBuffer[2];
						buffer[3] = Dev->rxBuffer[3];
						buffer[4] = Dev->rxBuffer[4];
						buffer[5] = Dev->rxBuffer[5];
						Dev->bTxReday = 1;
					}
					
					if(Dev->bindingTimer > 100){
						if(!Dev->bRxReday || !Dev->bTxReday) TxBindStat = SH_RESET;
						else {
		          bk242xStandby(Dev);
		          bk242xFlushRx(Dev);
		          bk242xClearFlags(Dev);
											
			        bk242xWriteAddr(Dev, &buffer[1], 5);		
			        buffer[0] = RF_SHANKHAND_THRID;
		          bk242xSendPackage(Dev, buffer, 6);		
							
			        Dev->bTxReday = 0;		
              Dev->txPeriod = Dev->txTimer;							
		          Dev->bindingTimer = 0; 
			        TxBindStat = SH_THIRD;							
						}
					}							
					break;
				case SH_THIRD:
					Dev->frequencyHopTimer = 0;
				  Dev->frequencyOccupyTimer = 0;
					bk242xTxCtrl(Dev);
				
				  if(Dev->bindingTimer > 100){
						if(!Dev->bTxReday) TxBindStat = SH_RESET;
						else {
		          bk242xFlushTx(Dev);
		          bk242xStandby(Dev);
		          bk242xClearFlags(Dev);
							
							Dev->addr[0] = buffer[1];
							Dev->addr[1] = buffer[2];
							Dev->addr[2] = buffer[3];
							Dev->addr[3] = buffer[4];
							Dev->addr[4] = buffer[5];
							
							Dev->bTxReday = 0;
							Dev->bBinding = 0;
							buffer[0] = RF_CODE_NULL;
		          bk242xSendPackage(Dev, buffer, 6);	   
              
	            //对码完成 新地址保存至EEPROM
	            for(i = 0; i < 5; i++) {
	              cfg.RFTxAddr[i] = Dev->addr[i];
	            }
	            writeEEPROM(1, 1);								
						}
					}								
					break;
				default:
					TxBindStat = SH_RESET;
				  break;
		}			
} 

/** @Brief 对码
  *   1) 用在接收端(通常指接收机)
  *   2) 与之对应的发射端对码程序为 RFTxBind()
  *
  * @Note 
  * @ RF_PUBLIC_ADDR_TX, RF_PUBLIC_ADDR_RX，有两个作用:
  *   1)、区分遥控端和接收端。
  *   2)、区分不同遥控器，不同接收端。
  *   取值范围: RF_PUBLIC_ADDR_TX = 10000001 ~ 11111111; RF_PUBLIC_ADDR_RX = 00000001 ~ 01111111。
  *
  */ 
void RFRxBind(RFDevice *Dev){	
	  static uint8_t buffer[8];
	  static Shankhand RxBindStat = SH_RESET;
	  static uint8_t bindAddrPublic = 0;
	  static uint8_t randomAddr;
	  uint8_t i;
	
	  if(!Dev->bBinding) return;
	
		switch(RxBindStat) {
		  	case SH_RESET:
		      bk242xFlushTx(Dev);
		      bk242xStandby(Dev);
		      bk242xFlushRx(Dev);
		      bk242xClearFlags(Dev);			  
			
				  if(bindAddrPublic) {
	          buffer[0] = RF_PUBLIC_ADDR0; 
		        buffer[1] = RF_PUBLIC_ADDR1;
		        buffer[2] = RF_PUBLIC_ADDR2;
		        buffer[3] = RF_PUBLIC_ADDR3;
		        buffer[4] = RF_PUBLIC_ADDR4;  								
					} else {
	          buffer[0] = Dev->addr[0]; 
		        buffer[1] = Dev->addr[1];
		        buffer[2] = Dev->addr[2];
		        buffer[3] = Dev->addr[3];
		        buffer[4] = Dev->addr[4];  	
					}
			    bk242xWriteAddr(Dev, buffer, 5);			
			
					Dev->bindingTimer = 0;
			    Dev->bRxReday = 0;
			    RxBindStat = SH_FIRST;								
				  break;
				case SH_FIRST:				
					bk242xRxCtrl(Dev); 									
						
					if(Dev->bRxReday){
						if(bindAddrPublic && (Dev->rxBuffer[0] == RF_SHANKHAND_FIRST)){
		          bk242xStandby(Dev);
		          bk242xFlushRx(Dev);
		          bk242xClearFlags(Dev);	
						
					    buffer[1] = Dev->rxBuffer[1];
					    buffer[2] = Dev->rxBuffer[2];
					    buffer[3] = Dev->rxBuffer[3];
					    buffer[4] = Dev->rxBuffer[4];
					    buffer[5] = Dev->rxBuffer[5];				
			        bk242xWriteAddr(Dev, &buffer[1], 5);	
					  
							randomAddr = micros();
		          buffer[0] = RF_SHANKHAND_SECOND;
		          buffer[1] = Dev->addr[0] + randomAddr;
	            buffer[2] = Dev->addr[1] + randomAddr;
		          buffer[3] = Dev->addr[2] + randomAddr;
	            buffer[4] = Dev->addr[3] + randomAddr;
		          buffer[5] = RF_PUBLIC_ADDR_RX;				
		          bk242xSendPackage(Dev, buffer, 6);						
						
						  Dev->txPeriod = Dev->txTimer;
						  Dev->bTxReday = 0;						
		          Dev->bindingTimer = 0; 
						  RxBindStat = SH_SECOND;		
              break;		
						} else if((!bindAddrPublic) && (Dev->rxBuffer[0] == RF_CODE_NORMAL)){							
		          bk242xStandby(Dev);
		          bk242xFlushRx(Dev);
		          bk242xClearFlags(Dev);
											
			        Dev->bRxReday = 0;		
			        Dev->bTxReday = 0;	
							Dev->bBinding = 0;
							break;
						}							
					}		

				  if(Dev->bindingTimer > 200){	
						Dev->bindingTimer = 0;
						bindAddrPublic = ~bindAddrPublic;
						RxBindStat = SH_RESET;
          }					
				  break;
				case SH_SECOND:						
					Dev->frequencyHopTimer = 0;
				  Dev->frequencyOccupyTimer = 0;
					bk242xTxCtrl(Dev);
				
				  if(Dev->bindingTimer > 100){
						if(!Dev->bTxReday) RxBindStat = SH_RESET;
						else {
		          bk242xFlushTx(Dev);
		          bk242xStandby(Dev);
		          bk242xClearFlags(Dev);
							
		          buffer[1] = Dev->addr[0] + randomAddr;
	            buffer[2] = Dev->addr[1] + randomAddr;
		          buffer[3] = Dev->addr[2] + randomAddr;
	            buffer[4] = Dev->addr[3] + randomAddr;
		          buffer[5] = RF_PUBLIC_ADDR_RX;	
			        bk242xWriteAddr(Dev, &buffer[1], 5);	
							
						  Dev->rxPeriod = Dev->rxTimer;
						  Dev->bRxReday = 0;			
						  Dev->bTxReday = 0;									
		          Dev->bindingTimer = 0; 	
						  RxBindStat = SH_THIRD;											
						}
					}						
				  break;				
				case SH_THIRD:
		      Dev->channelEffect = Dev->channel;                      //禁用跳频
		      Dev->frequencyHopTimer = 0;
		      Dev->bFrequencyAlternative = 0;				
					bk242xRxCtrl(Dev);  
				  
					if(Dev->bRxReday && (Dev->rxBuffer[0] == RF_SHANKHAND_THRID)){
						buffer[1] = Dev->rxBuffer[1];
						buffer[2] = Dev->rxBuffer[2];
						buffer[3] = Dev->rxBuffer[3];
						buffer[4] = Dev->rxBuffer[4];
						buffer[5] = Dev->rxBuffer[5];
						Dev->bTxReday = 1;
					}
					
					if(Dev->bindingTimer > 100){
						if(!Dev->bRxReday || !Dev->bTxReday) RxBindStat = SH_RESET;
						else {
		          bk242xStandby(Dev);
		          bk242xFlushRx(Dev);
		          bk242xClearFlags(Dev);
											
			        Dev->bRxReday = 0;		
			        Dev->bTxReday = 0;	
							Dev->bBinding = 0;										
							
	            //对码完成 新地址保存至EEPROM
		          Dev->addr[0] += randomAddr;
	            Dev->addr[1] += randomAddr;
		          Dev->addr[2] += randomAddr;
	            Dev->addr[3] += randomAddr;
		          Dev->addr[4] = RF_PUBLIC_ADDR_RX;	
	            for(i = 0; i < 5; i++) {
	              cfg.RFRxAddr[i] = Dev->addr[i];
	            }
	            writeEEPROM(1, 1);	
						}
					}							
					break;
				default:
					RxBindStat = SH_RESET;
				  break;
		}				
}

//================================================================================================================ 
//================================================================================================================ 
void RFLoop(void){
	  struct sbus_dat {
      unsigned int chan0 : 11;
      unsigned int chan1 : 11;
      unsigned int chan2 : 11;
      unsigned int chan3 : 11;
      unsigned int chan4 : 11;
      unsigned int chan5 : 11;
      unsigned int chan6 : 11;
      unsigned int chan7 : 11;
      unsigned int chan8 : 11;
      unsigned int chan9 : 11;
    } __attribute__ ((__packed__));
		
		#define SBUS_FRAME_SIZE 14
		
    union {
      uint8_t in[SBUS_FRAME_SIZE];
      struct sbus_dat msg;
    } sbus_msg;
		
	  static uint32_t currentTime, loopTime;
	  uint8_t i;
		uint16_t stickKnobValue[5];
		static uint16_t ledCyc, ledTimer, txNoSignalTimer;
		static uint8_t rfReadyBeep = 1;
		
	
	  //各控制变量，每隔1ms，累加1
	  currentTime = micros();
    if ((int32_t)(currentTime - loopTime) >= 0) {
        loopTime = currentTime + 1000;			
			  for(i = 0; i < RF_DEV_SUM; i++){
				  RFDev[i].txTimer++;
				  RFDev[i].rxTimer++;
				  RFDev[i].bindingTimer++;
				  RFDev[i].frequencyHopTimer++;
				  RFDev[i].frequencyOccupyTimer++;					
		    }
				ledTimer++;
				txNoSignalTimer++;
		}
		
		//RFPA作为发射，RFRx作为接收
		if(RFDev[RFPA].bBinding){
			RFTxBind(&RFDev[RFPA]);
			if(ledTimer >= 50) {
				ledTimer = 0;
				LED0_TOGGLE;
			}				
		} else {
			  bk242xTxCtrl(&RFDev[RFPA]); 
			
			  if(RFDev[RFPA].bTxReday) {
					  RFDev[RFPA].bTxReday = 0;
					  txNoSignalTimer = 0;
					  if(rfReadyBeep) {
						    rfReadyBeep = 0;						
					      buzzer(BUZZER_ACC_CALIBRATION);
					  }
				} else if(txNoSignalTimer >= 1000) {
					  txNoSignalTimer = 0;
					  bk242xInit(&RFDev[RFPA]);
				}				
				
			  stickKnobUpdate(stickKnobValue);
			  sbus_msg.msg.chan0  = stickKnobValue[0];
			  sbus_msg.msg.chan1  = stickKnobValue[1];
			  sbus_msg.msg.chan2  = stickKnobValue[2];
			  sbus_msg.msg.chan3  = stickKnobValue[3];
			  sbus_msg.msg.chan4  = stickKnobValue[4];
			  sbus_msg.msg.chan5  = constrain(btnChannel[0], 1000, 2000);
			  sbus_msg.msg.chan6  = constrain(btnChannel[1], 1000, 2000);
			  sbus_msg.msg.chan7  = constrain(btnChannel[2], 1000, 2000);
			  sbus_msg.msg.chan8  = constrain(btnChannel[3], 1000, 2000);
			  sbus_msg.msg.chan9  = constrain(btnChannel[4], 1000, 2000);
			
			  RFDev[RFPA].txBuffer[0]  = RF_CODE_NORMAL;
			  for(i = 0; i < SBUS_FRAME_SIZE; i++) {
					  RFDev[RFPA].txBuffer[i+1] = sbus_msg.in[i];
				}									
		}
		
		#define DEBUGE_RX_ENB
		#if defined(DEBUGE_RX_ENB)
		if(RFDev[RFRx].bBinding) RFRxBind(&RFDev[RFRx]);
		else {
			  bk242xRxCtrl(&RFDev[RFRx]); 
			
			  if(RFDev[RFRx].bRxReday) {
					 RFDev[RFRx].bRxReday = 0;
					 if(RFDev[RFRx].rxBuffer[0] == RF_CODE_NORMAL) {
						 if(ledCyc++%50 == 0) LED0_TOGGLE;
						 
			       for(i = 0; i < SBUS_FRAME_SIZE; i++) {
					       sbus_msg.in[i] = RFDev[RFRx].rxBuffer[i+1];
				     }					
			       servo[0] = sbus_msg.msg.chan0;
			       servo[1] = sbus_msg.msg.chan1;
			       servo[2] = sbus_msg.msg.chan2;
			       servo[3] = sbus_msg.msg.chan3;
			       servo[4] = sbus_msg.msg.chan4;
			       servo[5] = sbus_msg.msg.chan5;
			       servo[6] = sbus_msg.msg.chan6;
			       servo[7] = sbus_msg.msg.chan7;
					 }
				}			  
		}
		#endif
}









