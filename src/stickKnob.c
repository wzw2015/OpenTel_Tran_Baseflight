#include "board.h"
#include "mw.h"

#define STICK_KNOB_SUM        5

struct {
	uint16_t adcLimitHigh;       //ADC限位
	uint16_t adcLimitLow;
	uint16_t adcLimitMiddle;	
	AdcChannel adcChannel;
	
	uint8_t  reverse;	           //是否取反
	uint16_t output;             //输出 1000~2000
} StickKnob[STICK_KNOB_SUM];	  	 


uint16_t StickKnobCalTimer;


/** @Brief 摇杆\旋钮初始化
  *   1) 定义电位器方向
  *   2) 定义电位器对应ADC通道
	*   3) 另，各电位器端点及中值已在读取EEPROM时更新
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


/** @Brief 获取电位器\旋钮数值
  * @Return StickKnob.output = 1000~2000
  */
void stickKnobScan(void) {
	uint16_t adc, result;
	float def, range;
	uint8_t i;
	
	//1) 依次更新所有电位器
	for(i=0; i<STICK_KNOB_SUM; i++) {
		adc = adcGetChannel(StickKnob[i].adcChannel);
		
		//2) 以中值为界，上下两段分别解算
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
		
		//3) 若电位器反向，调整输出数据为(左小右大、后小前大)
		if(StickKnob[i].reverse) StickKnob[i].output = 1000 - StickKnob[i].output;
		StickKnob[i].output += 1000;
	}	
}


/** @Brief 电位器\旋钮矫正上下边缘
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
	
	//1) 读取电位器ADC数值，更新上下边缘
	//2) 30s后退出
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
	
	//3) 上下边缘各留出5%的虚位
	for(i=0; i<STICK_KNOB_SUM; i++) {
		def = StickKnob[i].adcLimitHigh - StickKnob[i].adcLimitLow;
		def /= 20;
		if(def < 0) def = 0;
		
		StickKnob[i].adcLimitHigh -= def;
		StickKnob[i].adcLimitLow += def;
	}	
	
	//4) 保存至EEPROM
	for(i = 0; i < STICK_KNOB_SUM; i++) {
	  cfg.adcLimitHigh[i] = StickKnob[i].adcLimitHigh;
	  cfg.adcLimitLow[i] = StickKnob[i].adcLimitLow;
	}
	writeEEPROM(1, 1);	
}

/** @Brief 电位器\旋钮获取中值
  * @ 多次采用取平均
  *
  * @Note
  *   1) 调用此程序前，各电位器拨到中间位置
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
	
	//1) 每20ms采样一次，1s后计算均值
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
	
	//2) 保存至EEPROM
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
	
	  //各控制变量，每隔1ms，累加1
	  currentTime = micros();
    if ((int32_t)(currentTime - loopTime) >= 0) {
        loopTime = currentTime + 10000;			
			  
			  stickKnobScan();			  
		}
		
		for(i=0; i<STICK_KNOB_SUM; i++) {
			value[i] = StickKnob[i].output;
		}
}







