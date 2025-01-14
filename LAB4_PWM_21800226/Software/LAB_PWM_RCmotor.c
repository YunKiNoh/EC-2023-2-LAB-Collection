/*------------------------------------------------
Class: 23_2_Embeded_Controller
LAB: LAB_PWM_RCmotor
Name: NohYunKi
------------------------------------------------*/

#include "stm32f411xe.h"
#include "math.h"

// #include "ecSTM32F411.h"
#include "ecPinNames.h"
#include "ecGPIO.h"
#include "ecSysTick.h"
#include "ecEXTI.h"
#include "ecRCC.h"
#include "ecTIM.h"
#include "ecPWM.h"   // ecPWM2.h

// Definition Button Pin & PWM Port, Pin
#define PWM_PIN PA_1

void setup(void);
void EXTI15_10_IRQHandler(void);
void TIM3_IRQHandler(void);

int cnt = 0;

int main(void) {
	
  setup();   

	while(1){}
}


// Initialiization 
void setup(void) {   
	 // System clock setting
   RCC_PLL_init();
   
	 // Pin setting
   GPIO_init(GPIOA, 1, AF);
	 GPIO_init(GPIOA, LED_PIN, OUTPUT); 
   GPIO_init(GPIOC, BUTTON_PIN, INPUT);
   
	 // Timer setting 
   TIM_init(TIM3, 500); // TIMER period = 500ms
	 TIM3->DIER |= 1;	// Update Interrupt Enabled
	 NVIC_EnableIRQ(TIM3_IRQn);	// TIM3's interrupt request enabled	
	 NVIC_SetPriority(TIM3_IRQn, 2); // set TIM's interrupt priority
	 
	 // External Interrupt setting
	 EXTI_init(GPIOC, 13, FALL, 0); // set C_port's 13_pin as signal of EXTI
	 NVIC_EnableIRQ(EXTI15_10_IRQn); // enable request interrupt
	 NVIC_SetPriority(EXTI15_10_IRQn, 3); // set priority
	 
	 // PWM setting
   PWM_init(PWM_PIN); // set Port A's 1_pin as PWM's output pin
   PWM_period(PWM_PIN, 20); // 20 msec PWM period --> set PSC, ARR value to make PWM_period as 20ms
}

void TIM3_IRQHandler(void){
	if((TIM3->SR & TIM_SR_UIF) == 1){ 
		// 0.5ms = 0dgree, 1.5ms = 90degree, 2.5ms = 180degree
		PWM_duty(PWM_PIN, (float)0.025*(cnt*4/17.0 + 1));
		cnt ++;
		// count 
		if(cnt > 17) cnt = 0;
		// clear by writing 0
		TIM3->SR &= ~ TIM_SR_UIF;                    
	}
}

void EXTI15_10_IRQHandler(void) {
   if (is_pending_EXTI(BUTTON_PIN)) {
      PWM_duty(PWM_PIN, (float)0.025);
		  cnt = 0;
		  clear_pending_EXTI(BUTTON_PIN);
   }
} 
