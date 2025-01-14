/**
******************************************************************************
* @author  SSSLAB
* @Mod		 2021-8-30 by YKKIM  	
* @brief   Embedded Controller:  LAB Systick&EXTI with API
*					 - 7 segment
* 
******************************************************************************
*/

#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecSysTick.h"
#include "ecEXTI.h"

void EXTI15_10_IRQHandler(void);
void setup(void);

int count = 0;
// Initialiization 

int main(void) { 
	// Initialiization --------------------------------------------------------
		setup();
	
	// Inifinite Loop ----------------------------------------------------------
	while(1){
		sevensegment_display(count);
		delay_ms(1000);
		count++;
		if (count >9) count =0;
		SysTick_reset();
	}
}

void setup(void)
{
	RCC_PLL_init();
	SysTick_init();
	GPIO_init(GPIOC,BUTTON_PIN,INPUT);
	GPIO_pupd(GPIOC,BUTTON_PIN,EC_PU);
	
	EXTI_init(GPIOC,13,FALL,0);
	NVIC_EnableIRQ(EXTI15_10_IRQn); // enable request interrupt
	NVIC_SetPriority(EXTI15_10_IRQn, 0); // set priority
	
	sevensegment_init();
}
	
void EXTI15_10_IRQHandler(void){
   if (is_pending_EXTI(BUTTON_PIN)){
      count = 0;
		 	sevensegment_display(count);
		  clear_pending_EXTI(BUTTON_PIN);
   }
}
