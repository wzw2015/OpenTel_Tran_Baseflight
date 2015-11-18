/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */
#include "board.h"
#include "mw.h"
#include "buzzer.h"

#include "telemetry_common.h"

core_t core;
int hw_revision = 0;

extern rcReadRawDataPtr rcReadRawFunc;

// receiver read function
extern uint16_t pwmReadRawRC(uint8_t chan);

// from system_stm32f10x.c
void SetSysClock(bool overclock);

#ifdef USE_LAME_PRINTF
// gcc/GNU version
static void _putc(void *p, char c)
{
    (void)p;
    serialWrite(core.mainport, c);
}
#else
// keil/armcc version
int fputc(int c, FILE *f)
{
    // let DMA catch up a bit when using set or dump, we're too fast.
    while (!isSerialTransmitBufferEmpty(core.mainport));
    serialWrite(core.mainport, c);
    return c;
}
#endif

int main(void)
{
    drv_adc_config_t adc_params;
#ifdef SOFTSERIAL_LOOPBACK
    serialPort_t *loopbackPort1 = NULL;
    serialPort_t *loopbackPort2 = NULL;
#endif

    initEEPROM();
    checkFirstTime(false);
    readEEPROM();

    // Configure clock, this figures out HSE for hardware autodetect
    SetSysClock(mcfg.emf_avoidance);	
    hw_revision = NAZE32;
    systemInit();
	
		beepPWMInit();
    adcInit(&adc_params);
		spiInit();
		btnInit();
    serialInit(mcfg.serial_baudrate);				
		stickKnobInit();
		rfInit();
		
    LED0_ON;
    LED1_OFF;		 
		buzzer(BUZZER_TX_SET);
		
    // loopy
    while (1) {
        loop();
#ifdef SOFTSERIAL_LOOPBACK
        if (loopbackPort1) {
            while (serialTotalBytesWaiting(loopbackPort1)) {
                uint8_t b = serialRead(loopbackPort1);
                serialWrite(loopbackPort1, b);
                //serialWrite(core.mainport, 0x01);
                //serialWrite(core.mainport, b);
            };
        }

        if (loopbackPort2) {
            while (serialTotalBytesWaiting(loopbackPort2)) {
                serialRead(loopbackPort2);
            };
        }
#endif
    }
}

void HardFault_Handler(void)
{
    // fall out of the sky
    writeAllMotors(mcfg.mincommand);
    while (1);
}
