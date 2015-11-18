#pragma once

typedef enum {
    ADC_BATTERY = 0,
    ADC_STICK_RIGHT_ROLL,
	  ADC_STICK_RIGHT_PITCH,
	  ADC_STICK_LEFT_PITCH,
	  ADC_STICK_LEFT_ROLL,
	  ADC_TRIM_RIGHT_ROLL,
	
	  ADC_RSSI,
    ADC_CHANNEL_MAX              //ADC_CHANNEL_MAX 放在最后
} AdcChannel;

typedef struct drv_adc_config_t {
    uint8_t powerAdcChannel;     // which channel used for current monitor, allowed PA1, PB1 (ADC_Channel_1, ADC_Channel_9)
    uint8_t rssiAdcChannel;      // which channel used for analog-rssi (RC-filter), allowed PA1, PB1 (ADC_Channel_1, ADC_Channel_9)
} drv_adc_config_t;

void adcInit(drv_adc_config_t *init);
uint16_t adcGetChannel(uint8_t channel);
