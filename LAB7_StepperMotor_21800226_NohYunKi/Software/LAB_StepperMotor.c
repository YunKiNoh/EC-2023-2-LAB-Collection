/*--------------------------------------------------------------
Hnadong Global University_Embeded Controller_Kim Yung Keun Prof.
Student: NohYunKi
Modified: 23/11/10
--------------------------------------------------------------*/

#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecEXTI.h"
#include "ecSysTick.h"
#include "ecStepper.h"

void setup(void);
void EXTI15_10_IRQHandler(void);
volatile int signal = 0;
   
int main(void) { 
   // Initialiization --------------------------------------------------------
   setup();
   
   // Inifinite Loop ----------------------------------------------------------
   while(1){
		 if(signal == 1){
		 Stepper_stop();
		 }
		 else if(signal == 0) Stepper_step(2048, 1, FULL);  // (Step : 2048, Direction : 0(CW) or 1(CCW), Mode : FULL or HALF)
	 }

}

// Initialiization 
void setup(void){
   
   RCC_PLL_init();                                 // System Clock = 84MHz
   SysTick_init();                                 // Systick init
   
   EXTI_init(GPIOC, BUTTON_PIN, FALL,0);           // External Interrupt Setting
   GPIO_init(GPIOC, BUTTON_PIN, INPUT);           // GPIOC pin13 initialization
   
   Stepper_init(GPIOB,10,GPIOB,4,GPIOB,5,GPIOB,3); // Stepper GPIO pin initialization
   Stepper_setSpeed(10);                             //  set stepper motor speed
}

void EXTI15_10_IRQHandler(void) {  
   if (is_pending_EXTI(BUTTON_PIN)) {
		  signal = 1;
      clear_pending_EXTI(BUTTON_PIN); // cleared by writing '1'
   }
}
