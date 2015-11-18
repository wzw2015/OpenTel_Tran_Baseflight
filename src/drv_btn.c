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
		
		//五个按键都配置为上拉输入
		for (i = 0; i < gpio_count; i++) {
        gpioInit(gpio_setup[i].gpio, &gpio_setup[i].cfg);
    }
}

/**
  *  左上角对码
  *  右上角功能切换，TK16小于中值为炮塔旋转，所以初始值1000，每按取反
  *  左下角机枪，TK16大于中值使能机枪，所以初始值1000，按住2000
  *  右下角开炮，TK16大于中值使能，所以初始值1000，按住2000
  *  最右侧启动，每按取反，初值1000
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
	
	  //每20ms调用一次
	  currentTime = micros();
    if ((int32_t)(currentTime - loopTime) >= 0) {
        loopTime = currentTime + 20000;			
			
			  //-----------------------------------------------------------------------------------			
			  //按键去抖，连续三次检测相同状态，使能
			  #define FILTER_FREQUENCY    2   
			  if(!digitalIn(BTN_LTOP_GPIO, BTN_LTOP_PIN)) {              //按下接地 左上方按键
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
			  if(!digitalIn(BTN_RTOP_GPIO, BTN_RTOP_PIN)) {              //按下接地  右上方按键
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
				
			  if(!digitalIn(BTN_LBLOW_GPIO, BTN_LBLOW_PIN)) {              //按下接地  左下方按键
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
			  if(!digitalIn(BTN_RBLOW_GPIO, BTN_RBLOW_PIN)) {              //按下接地  右下方按键
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
				
			  if(!digitalIn(BTN_RRIGHT_GPIO, BTN_RRIGHT_PIN)) {              //按下接地  最右侧按键
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
			
			  //端口检测及有效性判断			
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
				//按键更新						
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
        //长按检测
				//1  右上角按键 --- 矫正电位器中值及边缘
				//   1) 按下前，各电位器放在中间(主要是微调)
				//   2) 长按3s，指示灯快闪(20Hz)。各电位器由最小到最大往返两个。
        if(digitalIn(BTN_RTOP_GPIO, BTN_RTOP_PIN)) RTOPlongPushTimer = 0;
        else if(RTOPlongPushTimer++ >= 150) {
					RTOPlongPushTimer = 0;					
					stickKnobCalibrationMiddle();
					stickKnobCalibrationEdge();
				}			

        //2  左上角按键 --- 对码
				//   1) 长按2s，指示灯快闪(5Hz)，开始对码
        if(digitalIn(BTN_LTOP_GPIO, BTN_LTOP_PIN)) LTOPlongPushTimer = 0;
        else if(LTOPlongPushTimer++ >= 100) {
					LTOPlongPushTimer = 0;			
          RFDev[RFPA].bBinding = 1;	
				}				
        //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++	
		}
}



