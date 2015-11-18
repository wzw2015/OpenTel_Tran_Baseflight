#include "board.h"
#include "mw.h"
#include "buzzer.h"

#define BTN_LTOP_GPIO     GPIOC
#define BTN_LTOP_PIN      Pin_4  
#define BTN_RTOP_GPIO     GPIOC
#define BTN_RTOP_PIN      Pin_5 
#define BTN_LBLOW_GPIO    GPIOA
#define BTN_LBLOW_PIN     Pin_3 
#define BTN_RBLOW_GPIO    GPIOB
#define BTN_RBLOW_PIN     Pin_0 
#define BTN_RRIGHT_GPIO   GPIOB
#define BTN_RRIGHT_PIN    Pin_1 

uint16_t btnChannel[5] = {1000, 1000, 1000, 1000, 1000};

void btnInit(void)
{    
    struct {
        GPIO_TypeDef *gpio;
        gpio_config_t cfg;
    } gpio_setup[] = {
        {
            .gpio = BTN_LTOP_GPIO,
            .cfg = { BTN_LTOP_PIN, Mode_IPU, Speed_2MHz }
        },
        {
            .gpio = BTN_RTOP_GPIO,
            .cfg = { BTN_RTOP_PIN, Mode_IPU, Speed_2MHz }
        },
        {
            .gpio = BTN_LBLOW_GPIO,
            .cfg = { BTN_LBLOW_PIN, Mode_IPU, Speed_2MHz }
        },
        {
            .gpio = BTN_RBLOW_GPIO,
            .cfg = { BTN_RBLOW_PIN, Mode_IPU, Speed_2MHz }
        },
        {
            .gpio = BTN_RRIGHT_GPIO,
            .cfg = { BTN_RRIGHT_PIN, Mode_IPU, Speed_2MHz }
        },
    };
    int i, gpio_count = sizeof(gpio_setup) / sizeof(gpio_setup[0]);		
		
		//�������������Ϊ��������
		for (i = 0; i < gpio_count; i++) {
        gpioInit(gpio_setup[i].gpio, &gpio_setup[i].cfg);
    }
}

/**
  *  ���ϽǶ���
  *  ���Ͻǹ����л���TK16С����ֵΪ������ת�����Գ�ʼֵ1000��ÿ��ȡ��
  *  ���½ǻ�ǹ��TK16������ֵʹ�ܻ�ǹ�����Գ�ʼֵ1000����ס2000
  *  ���½ǿ��ڣ�TK16������ֵʹ�ܣ����Գ�ʼֵ1000����ס2000
  *  ���Ҳ�������ÿ��ȡ������ֵ1000
  */
void btnScan(void)
{
    static uint32_t currentTime, loopTime;
	  static struct {
			uint32_t ltopCkEnb:1;
			uint32_t ltopPush:1;
			uint32_t rtopCkEnb:1;
			uint32_t rtopPush:1;
			uint32_t lblowCkEnb:1;
			uint32_t lblowPush:1;
			uint32_t rblowCkEnb:1;
			uint32_t rblowPush:1;
			uint32_t rrightCkEnb:1;
			uint32_t rrightPush:1;			
			
			uint32_t filterLtopPush:1;
			uint32_t filterRtopPush:1;
			uint32_t filterLblowPush:1;
			uint32_t filterRblowPush:1;
			uint32_t filterRrightPush:1;
		} BtnFlags = {0};
		static int8_t ltopFilterCounter = 0, rtopFilterCounter = 0, lblowFilterCounter = 0, rblowFilterCounter = 0, rrightFilterCounter = 0;
		static uint32_t LTOPlongPushTimer = 0, RTOPlongPushTimer = 0;
	
	  //ÿ20ms����һ��
	  currentTime = micros();
    if ((int32_t)(currentTime - loopTime) >= 0) {
        loopTime = currentTime + 20000;			
			
			  //-----------------------------------------------------------------------------------			
			  //����ȥ�����������μ����ͬ״̬��ʹ��
			  #define FILTER_FREQUENCY    2   
			  if(!digitalIn(BTN_LTOP_GPIO, BTN_LTOP_PIN)) {              //���½ӵ� ���Ϸ�����
					if(ltopFilterCounter < 0) ltopFilterCounter = 0;
					if(++ltopFilterCounter >= FILTER_FREQUENCY) {
						ltopFilterCounter = FILTER_FREQUENCY;
						BtnFlags.filterLtopPush = 1;
					}
				} else {
					if(ltopFilterCounter > 0) ltopFilterCounter = 0;
					if(--ltopFilterCounter <= -FILTER_FREQUENCY) {
						ltopFilterCounter = -FILTER_FREQUENCY;
						BtnFlags.filterLtopPush = 0;
					}
				}								
			  if(!digitalIn(BTN_RTOP_GPIO, BTN_RTOP_PIN)) {              //���½ӵ�  ���Ϸ�����
					if(rtopFilterCounter < 0) rtopFilterCounter = 0;
					if(++rtopFilterCounter >= FILTER_FREQUENCY) {
						rtopFilterCounter = FILTER_FREQUENCY;
						BtnFlags.filterRtopPush = 1;
					}
				} else {
					if(rtopFilterCounter > 0) rtopFilterCounter = 0;
					if(--rtopFilterCounter <= -FILTER_FREQUENCY) {
						rtopFilterCounter = -FILTER_FREQUENCY;
						BtnFlags.filterRtopPush = 0;
					}
				}
				
			  if(!digitalIn(BTN_LBLOW_GPIO, BTN_LBLOW_PIN)) {              //���½ӵ�  ���·�����
					if(lblowFilterCounter < 0) lblowFilterCounter = 0;
					if(++lblowFilterCounter >= FILTER_FREQUENCY) {
						lblowFilterCounter = FILTER_FREQUENCY;
						BtnFlags.filterLblowPush = 1;
					}
				} else {
					if(lblowFilterCounter > 0) lblowFilterCounter = 0;
					if(--lblowFilterCounter <= -FILTER_FREQUENCY) {
						lblowFilterCounter = -FILTER_FREQUENCY;
						BtnFlags.filterLblowPush = 0;
					}
				}
			  if(!digitalIn(BTN_RBLOW_GPIO, BTN_RBLOW_PIN)) {              //���½ӵ�  ���·�����
					if(rblowFilterCounter < 0) rblowFilterCounter = 0;
					if(++rblowFilterCounter >= FILTER_FREQUENCY) {
						rblowFilterCounter = FILTER_FREQUENCY;
						BtnFlags.filterRblowPush = 1;
					}
				} else {
					if(rblowFilterCounter > 0) rblowFilterCounter = 0;
					if(--rblowFilterCounter <= -FILTER_FREQUENCY) {
						rblowFilterCounter = -FILTER_FREQUENCY;
						BtnFlags.filterRblowPush = 0;
					}
				}
				
			  if(!digitalIn(BTN_RRIGHT_GPIO, BTN_RRIGHT_PIN)) {              //���½ӵ�  ���Ҳఴ��
					if(rrightFilterCounter < 0) rrightFilterCounter = 0;
					if(++rrightFilterCounter >= FILTER_FREQUENCY) {
						rrightFilterCounter = FILTER_FREQUENCY;
						BtnFlags.filterRrightPush = 1;
					}
				} else {
					if(rrightFilterCounter > 0) rrightFilterCounter = 0;
					if(--rrightFilterCounter <= -FILTER_FREQUENCY) {
						rrightFilterCounter = -FILTER_FREQUENCY;
						BtnFlags.filterRrightPush = 0;
					}
				}				
			
			  //�˿ڼ�⼰��Ч���ж�			
			  BtnFlags.ltopPush   = 0;
			  BtnFlags.rtopPush   = 0;
			  BtnFlags.lblowPush  = 0;
			  BtnFlags.rblowPush  = 0;
			  BtnFlags.rrightPush = 0;		
			  if (!BtnFlags.filterLtopPush)   BtnFlags.ltopCkEnb   = 1;
				else  BtnFlags.ltopPush = 1;
			  if (!BtnFlags.filterRtopPush)   BtnFlags.rtopCkEnb   = 1;
			  else  BtnFlags.rtopPush = 1;
			  if (!BtnFlags.filterLblowPush)  BtnFlags.lblowCkEnb  = 1;
			  else  BtnFlags.lblowPush = 1;
			  if (!BtnFlags.filterRblowPush)  BtnFlags.rblowCkEnb  = 1;
			  else  BtnFlags.rblowPush = 1;
			  if (!BtnFlags.filterRrightPush) BtnFlags.rrightCkEnb = 1;	
			  else  BtnFlags.rrightPush = 1;		
			  
			  if(!BtnFlags.ltopCkEnb)   BtnFlags.ltopPush    = 0;
			  if(!BtnFlags.rtopCkEnb)   BtnFlags.rtopPush    = 0;
			  if(!BtnFlags.lblowCkEnb)  BtnFlags.lblowPush   = 0;
			  if(!BtnFlags.rblowCkEnb)  BtnFlags.rblowPush   = 0;
			  if(!BtnFlags.rrightCkEnb) BtnFlags.rrightPush  = 0;					
			  if(BtnFlags.ltopPush)     BtnFlags.ltopCkEnb   = 0;
			  if(BtnFlags.rtopPush)     BtnFlags.rtopCkEnb   = 0;
			  if(BtnFlags.lblowPush)    BtnFlags.lblowCkEnb  = 0;
			  if(BtnFlags.rblowPush)    BtnFlags.rblowCkEnb  = 0;
			  if(BtnFlags.rrightPush)   BtnFlags.rrightCkEnb = 0;
				
			  //-----------------------------------------------------------------------------------
				//��������						
			  if(BtnFlags.ltopPush) {
					btnChannel[0] = btnChannel[0] == 1000 ? 2000 : 1000;
					buzzer(BUZZER_TX_SET); 
				}
			  if(BtnFlags.rtopPush) {
					btnChannel[1] = btnChannel[1] == 1000 ? 2000 : 1000;
					buzzer(BUZZER_TX_SET);
				}				
				
			  if(BtnFlags.lblowPush || BtnFlags.rblowPush) {
					buzzer(BUZZER_TX_SET);
				}		
				btnChannel[2] = BtnFlags.filterLblowPush ? 2000 : 1000;
				btnChannel[3] = BtnFlags.filterRblowPush ? 2000 : 1000;
				
			  if(BtnFlags.rrightPush)  {
					btnChannel[4] = btnChannel[4] == 1000 ? 2000 : 1000;					
					buzzer(BUZZER_TX_SET);
				}		

        //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        //�������
				//1  ���Ͻǰ��� --- ������λ����ֵ����Ե
				//   1) ����ǰ������λ�������м�(��Ҫ��΢��)
				//   2) ����3s��ָʾ�ƿ���(20Hz)������λ������С���������������
        if(digitalIn(BTN_RTOP_GPIO, BTN_RTOP_PIN)) RTOPlongPushTimer = 0;
        else if(RTOPlongPushTimer++ >= 150) {
					RTOPlongPushTimer = 0;					
					stickKnobCalibrationMiddle();
					stickKnobCalibrationEdge();
				}			

        //2  ���Ͻǰ��� --- ����
				//   1) ����2s��ָʾ�ƿ���(5Hz)����ʼ����
        if(digitalIn(BTN_LTOP_GPIO, BTN_LTOP_PIN)) LTOPlongPushTimer = 0;
        else if(LTOPlongPushTimer++ >= 100) {
					LTOPlongPushTimer = 0;			
          RFDev[RFPA].bBinding = 1;	
				}				
        //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++	
		}
}



