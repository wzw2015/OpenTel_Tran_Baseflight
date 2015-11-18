#include "board.h"
#include "mw.h"

#define STICK_KNOB_SUM        5

struct {
	uint16_t adcLimitHigh;       //ADC��λ
	uint16_t adcLimitLow;
	uint16_t adcLimitMiddle;	
	AdcChannel adcChannel;
	
	uint8_t  reverse;	           //�Ƿ�ȡ��
	uint16_t output;             //��� 1000~2000
} StickKnob[STICK_KNOB_SUM];	  	 


uint16_t StickKnobCalTimer;


/** @Brief ҡ��\��ť��ʼ��
  *   1) �����λ������
  *   2) �����λ����ӦADCͨ��
	*   3) ������λ���˵㼰��ֵ���ڶ�ȡEEPROMʱ����
  */
void stickKnobInit(void) {
    uint8_t i;

    for(i=0; i<STICK_KNOB_SUM; i++) {
			  StickKnob[i].adcLimitHigh = cfg.adcLimitHigh[i];
			  StickKnob[i].adcLimitLow  = cfg.adcLimitLow[i];
			  StickKnob[i].adcLimitMiddle = cfg.adcLimitMiddle[i]; 
			
			  StickKnob[i].reverse = 0;
			  StickKnob[i].output  = 1500;
		}			
		
		StickKnob[0].adcChannel = ADC_STICK_RIGHT_ROLL;
	  StickKnob[1].adcChannel = ADC_STICK_RIGHT_PITCH;
	  StickKnob[2].adcChannel = ADC_STICK_LEFT_PITCH;
	  StickKnob[3].adcChannel = ADC_STICK_LEFT_ROLL;
	  StickKnob[4].adcChannel = ADC_TRIM_RIGHT_ROLL; 
}


/** @Brief ��ȡ��λ��\��ť��ֵ
  * @Return StickKnob.output = 1000~2000
  */
void stickKnobScan(void) {
	uint16_t adc, result;
	float def, range;
	uint8_t i;
	
	//1) ���θ������е�λ��
	for(i=0; i<STICK_KNOB_SUM; i++) {
		adc = adcGetChannel(StickKnob[i].adcChannel);
		
		//2) ����ֵΪ�磬�������ηֱ����
		if(adc >= StickKnob[i].adcLimitMiddle){
			def = adc - StickKnob[i].adcLimitMiddle;
			range = StickKnob[i].adcLimitHigh - StickKnob[i].adcLimitMiddle;
			if(range <= 0) range = 1;
			result = def / range * 500.0f;
			if(result > 500) result = 500;
			StickKnob[i].output = 500 + result;
		} else {
			def = StickKnob[i].adcLimitMiddle - adc;
			range = StickKnob[i].adcLimitMiddle - StickKnob[i].adcLimitLow;
			if(range <= 0) range = 1;
			result = def / range * 500.0f;
			if(result > 500) result = 500;
			StickKnob[i].output = 500 - result;
		}			
		
		//3) ����λ�����򣬵����������Ϊ(��С�Ҵ󡢺�Сǰ��)
		if(StickKnob[i].reverse) StickKnob[i].output = 1000 - StickKnob[i].output;
		StickKnob[i].output += 1000;
	}	
}


/** @Brief ��λ��\��ť�������±�Ե
  *
  */
void stickKnobCalibrationEdge(void) {
	uint32_t currentTime;
	uint8_t i;				
	uint16_t adc;
	int16_t def;	
	
	for(i=0; i<STICK_KNOB_SUM; i++) {
		StickKnob[i].adcLimitHigh = 2048;
		StickKnob[i].adcLimitLow  = 2048;
	}	
	
	//1) ��ȡ��λ��ADC��ֵ���������±�Ե
	//2) 30s���˳�
	currentTime = micros() + 30000000;
	while((int32_t)(currentTime - micros()) >= 0) {
			for(i=0; i<STICK_KNOB_SUM; i++) {
		    adc = adcGetChannel(StickKnob[i].adcChannel);
		    if(adc > StickKnob[i].adcLimitHigh){
					StickKnob[i].adcLimitHigh = adc;
				}
				if(adc < StickKnob[i].adcLimitLow){
					StickKnob[i].adcLimitLow = adc;
				}
	    }
			delay(20);			
			LED0_TOGGLE;
	}
	
	//3) ���±�Ե������5%����λ
	for(i=0; i<STICK_KNOB_SUM; i++) {
		def = StickKnob[i].adcLimitHigh - StickKnob[i].adcLimitLow;
		def /= 20;
		if(def < 0) def = 0;
		
		StickKnob[i].adcLimitHigh -= def;
		StickKnob[i].adcLimitLow += def;
	}	
	
	//4) ������EEPROM
	for(i = 0; i < STICK_KNOB_SUM; i++) {
	  cfg.adcLimitHigh[i] = StickKnob[i].adcLimitHigh;
	  cfg.adcLimitLow[i] = StickKnob[i].adcLimitLow;
	}
	writeEEPROM(1, 1);	
}

/** @Brief ��λ��\��ť��ȡ��ֵ
  * @ ��β���ȡƽ��
  *
  * @Note
  *   1) ���ô˳���ǰ������λ�������м�λ��
  */
void stickKnobCalibrationMiddle(void) {	
	uint32_t currentTime;
	uint8_t i, count;			
	uint16_t adc;	
	uint32_t adcSum[STICK_KNOB_SUM];
	
	for(i=0; i<STICK_KNOB_SUM; i++) {
		adcSum[i] = 0;	
		StickKnob[i].adcLimitMiddle = 2048;
	}
	count = 0;
	
	//1) ÿ20ms����һ�Σ�1s������ֵ
	currentTime = micros() + 300000;
	while((int32_t)(currentTime - micros()) >= 0) {
	    for(i=0; i<STICK_KNOB_SUM; i++) {	
	      adc = adcGetChannel(StickKnob[i].adcChannel);
		    adcSum[i] += adc;
			}
			count++;
			delay(20);		
			LED0_TOGGLE;	
	}
	
	for(i=0; i<STICK_KNOB_SUM; i++) {	
		StickKnob[i].adcLimitMiddle = adcSum[i]/count;
	}	
	
	//2) ������EEPROM
	for(i = 0; i < STICK_KNOB_SUM; i++) {
	  cfg.adcLimitMiddle[i] = StickKnob[i].adcLimitMiddle;
	}
	writeEEPROM(1, 1);	
}


//----------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------
void stickKnobUpdate(uint16_t* value)
{
    static uint32_t currentTime, loopTime;
	  uint8_t i;		
	
	  //�����Ʊ�����ÿ��1ms���ۼ�1
	  currentTime = micros();
    if ((int32_t)(currentTime - loopTime) >= 0) {
        loopTime = currentTime + 10000;			
			  
			  stickKnobScan();			  
		}
		
		for(i=0; i<STICK_KNOB_SUM; i++) {
			value[i] = StickKnob[i].output;
		}
}







