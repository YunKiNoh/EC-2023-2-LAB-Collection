/**
******************************************************************************
* @Content: Embeded Controller_Timer_InputCapture_Ultrasonic LAB
* @Date: 23/11/07     
* @Name: NohYunKi
* 
******************************************************************************
*/

#include "stm32f411xe.h"
#include "math.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecTIM.h"
#include "ecPWM.h"
#include "ecUART.h"
#include "ecSysTIck.h"


uint32_t ovf_cnt = 0;
float distance = 0;
float timeInterval = 0;
float time1 = 0;
float time2 = 0;

#define TRIG PA_6 // PWM signal
#define ECHO PB_6 // ECHO signal

void setup(void);
void TIM4_IRQHandler(void);

int main(void){
   
   setup();
   
   while(1){
		 if(distance > 30 || 0 > distance) printf("Bouncing Value \r\n"); // to recognize bouncing value
		 else if(30>distance>0) printf("%f cm\r\n", distance);
      delay_ms(500);
   }
}

void TIM4_IRQHandler(void){
   if(is_UIF(TIM4)){                     // Update interrupt
      ovf_cnt++;                                       // overflow count
      clear_UIF(TIM4);                           // clear update interrupt flag
   }
   if(is_CCIF(TIM4, 1)){                         // TIM4_Ch1 (IC1) Capture Flag. Rising Edge Detect
		  time1 = ICAP_capture(TIM4,IC_1);            // Capture TimeStart
      clear_CCIF(TIM4, 1);                // clear capture/compare interrupt flag 
   }                                              
   else if(is_CCIF(TIM4, 2)){                   // TIM4_Ch2 (IC2) Capture Flag. Falling Edge Detect
		  time2 = ICAP_capture(TIM4,IC_2);            // Capture TimeEnd
      timeInterval = 10*((time2 - time1)+(TIM3->ARR+1)*ovf_cnt);                // (10us * counter pulse -> [msec] unit) Total time of echo pulse
      distance = (float) timeInterval/58.0;    // [mm] -> [cm]
      ovf_cnt = 0;                        // overflow reset
      clear_CCIF(TIM4,2);                          // clear capture/compare interrupt flag 
   }
}

void setup(){

   RCC_PLL_init(); 
   SysTick_init();
   UART2_init();
  
// PWM configuration ---------------------------------------------------------------------   
   PWM_init(TRIG);                         // PA_6: Ultrasonic trig pulse
   GPIO_setting(GPIOA,6,AF,EC_PUSH_PULL,EC_NONE,EC_FAST);
   PWM_period_ms(TRIG, 1000);    // PWM of 50ms period. Use period_us()
   PWM_pulsewidth_ms(TRIG, 500);   // PWM pulse width of 10us
	 
   
// Input Capture configuration -----------------------------------------------------------------------   
   ICAP_init(PB_6);                      // PB_6 as input caputre
   GPIO_pupd(GPIOB,6,EC_NONE);
   
   ICAP_counter_us(ECHO, 10);       // ICAP counter step time as 10us
   ICAP_setup(ECHO, 1, IC_RISE);  // TIM4_CH1 as IC1 , rising edge detect
   ICAP_setup(ECHO, 2, IC_FALL);  // TIM4_CH2 as IC2 , falling edge detect
	
}
