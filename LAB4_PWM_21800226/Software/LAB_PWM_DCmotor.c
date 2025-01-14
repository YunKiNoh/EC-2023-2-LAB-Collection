/*------------------------------------------------
Class: 23_2_Embeded_Controller
LAB: LAB_PWM_DCmotor
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
float ratio = 0.75;
int pause = 0;
int button_signal = 0;

int main(void) {
	
  setup();   

	while(1){}
}


// Initialiization 
void setup(void) {   
	// System clock setting: frequency = 84MHz
   RCC_PLL_init();
   
	 // Button pin setting
	 GPIO_init(GPIOC, BUTTON_PIN, INPUT); 
	 GPIO_pupd(GPIOC, BUTTON_PIN,EC_PU);
	 // Direction pin setting
   GPIO_init(GPIOC, 2, OUTPUT);
	 GPIO_otype(GPIOC,2,EC_PUSH_PULL);
	 // PWM pin setting
	 GPIO_init(GPIOA, PA_0, AF);
	 GPIO_otype(GPIOA, PA_0,EC_PUSH_PULL);
	 GPIO_pupd(GPIOA, PA_0, EC_PU);
	 GPIO_ospeed(GPIOA, PA_0,EC_FAST);
   
	 // Timer setting 
   TIM_init(TIM3, 1); // TIMER period = 1ms
	 TIM3->DIER |= 1;	// Update Interrupt Enabled
	 NVIC_EnableIRQ(TIM3_IRQn);	// TIM3's interrupt request enabled	
	 NVIC_SetPriority(TIM3_IRQn, 2); // set TIM's interrupt priority
	 
	 // External Interrupt setting
	 EXTI_init(GPIOC, 13, FALL, 0); // set C_port's 13_pin as signal of EXTI
	 NVIC_EnableIRQ(EXTI15_10_IRQn); // enable request interrupt
	 NVIC_SetPriority(EXTI15_10_IRQn, 3); // set priority
	 
	 // PWM setting
   PWM_init(PWM_PIN); // set Port A's 1_pin as PWM's output pin
   PWM_period(PWM_PIN, 1); // PWM(PA0-->TIM2_CH1) period = 1ms
	 
	 // ?? even use GPIO_write to set 1 value into PC_2 pin, it's value is 0
	 GPIO_write(GPIOC, 2, 0);
}

void TIM3_IRQHandler(void){
	if((TIM3->SR & TIM_SR_UIF) == 1){ 
		// speed rate: 100% or 30% (if too low[ex.25%], DC motor didn't rotate)
		PWM_duty(PWM_PIN, ratio*pause);
		//PWM_duty(PWM_PIN, 0);
		cnt ++;
		// count 2s to change speed rate per 2 seconds
		if(cnt > 1999) 
		{
			cnt = 0;
			if(ratio == (float)0.75)  ratio = (float)0.25;
			else if(ratio == (float)0.25) ratio = (float)0.75;
		}
		// clear by writing 0
		TIM3->SR &= ~ TIM_SR_UIF;                    
	}
}

// Button external interrupt
void EXTI15_10_IRQHandler(void) {
   if (is_pending_EXTI(BUTTON_PIN)) {
		  if(GPIO_read(GPIOC,2) == 0) pause ^= 1;
		  clear_pending_EXTI(BUTTON_PIN);
   }
} 
