/**
******************************************************************************
* @author  NohYunKi 21800226
* @content   Embedded Controller_EXTI LAB 
*
******************************************************************************
*/

#include "stm32f4xx.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecEXTI.h"
#include "ecSysTick.h"



#define LED_PIN    5
#define BUTTON_PIN 13

void setup(void);
void EXTI15_10_IRQHandler(void);


unsigned int cnt = 0;


int main(void) {

   // System CLOCK, GPIO Initialiization ----------------------------------------
   setup();


   // EXTI Initialiization ------------------------------------------------------   

   EXTI_init(GPIOC, 13, FALL, 0);
		
   
   
   while (1);
}


void EXTI15_10_IRQHandler(void) {
   if (is_pending_EXTI(BUTTON_PIN)) {
      cnt++; 
		  if (cnt > 9) cnt = 0;
	    sevensegment_display(cnt);
		  //delay_ms(1);
		  for(volatile int i = 0; i < 300000;i++){} 
			
      clear_pending_EXTI(BUTTON_PIN); // cleared by writing '1'
   }
}

// Initialiization 
void setup(void)
{
   RCC_PLL_init();                         // System Clock = 84MHz
   // Initialize GPIOA_5 for Output
   GPIO_init(GPIOA, LED_PIN, OUTPUT);    // calls RCC_GPIOA_enable()   
   // Initialize GPIOC_13 for Input Button
   GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
	 // Initialize sevensegment
	 sevensegment_display_init();
}