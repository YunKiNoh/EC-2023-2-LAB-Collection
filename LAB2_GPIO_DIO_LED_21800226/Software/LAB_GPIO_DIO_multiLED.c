/**
******************************************************************************
* @author   NohYunKi
* @Mod      23/09/23
* @brief   Embedded Controller:  LAB Digital In/Out
*                - Toggle multiple LEDs by Button B1 pressing
* 
******************************************************************************
*/

#include "stm32f4xx.h"
#include "ecRCC.h"
#include "ecGPIO.h"

#define LED_PIN    5 // PA5
#define BUTTON_PIN 13

#define PA6 6
#define PA7 7
#define PB6 6

void setup(void);
   
int main(void) { 
   // Initialiization --------------------------------------------------------
   setup();
	 int count = 0;
   int signal = 0;
	
   // Inifinite Loop ----------------------------------------------------------
    while(1){

       if(GPIO_read(GPIOC, BUTTON_PIN) == 0){
				 signal = count % 4;
				 
				 switch(signal){
						
					 case 0 :
						 GPIOA->ODR ^= (1<<LED_PIN); //ON
						 if(count > 3){
								GPIOB->ODR ^= (1<<PB6); //OFF
						 }
						 break;
						 
					 case 1:
							GPIOA->ODR ^= (1<<LED_PIN); //OFF
							GPIOA->ODR ^= (1<<PA6); //ON
							break;
					 
					 case 2:
							GPIOA->ODR ^= (1<<PA6); //OFF
							GPIOA->ODR ^= (1<<PA7); //ON
							break;
					 
					 case 3:
							GPIOA->ODR ^= (1<<PA7); //OFF
							GPIOB->ODR ^= (1<<PB6); //ON		
			}
				for(volatile int i = 0; i < 200000; i++){}
				count ++;
		} 
	}
}
 

// Initialiization 
void setup(void)
{
   RCC_HSI_init();
   
   /*-----------------------------------------------------
   BUTTON_PIN's settings: 
   Digital Input, GPIOC13, Pull-up
   ------------------------------------------------------*/
   GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // 0.Enalbe & 1. Input
   GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU);  // 3. Pull-up
   
   /*-----------------------------------------------------
   LED_PIN's settings: 
   Digital Output, (GPIOA5,GPIOA6,GPIOA7,GPIOB6), Push-Pull, Pull-up, Medium Speed 
   ------------------------------------------------------*/
   GPIO_init(GPIOA, LED_PIN, OUTPUT);    // PA5: 0.Enable & 1.Output
   GPIO_init(GPIOA, PA6, OUTPUT);            // PA6: 0.Enable & 1.Output
   GPIO_init(GPIOA, PA7, OUTPUT);            // PA7: 0.Enable & 1.Output
   GPIO_init(GPIOB, PB6, OUTPUT);            // PB6: 0.Enable & 1.Output
   
   /*GPIOA5*/
   GPIO_otype(GPIOA, LED_PIN, EC_PUSH_PULL);   // 2.Push-Pull
   GPIO_pupd(GPIOA, LED_PIN, EC_PU);               // 3.Pull-up
   GPIO_ospeed(GPIOA, LED_PIN, EC_MEDIUM);    // 4.Medium speed
	 GPIO_write(GPIOA, LED_PIN, LOW);
   
   /*GPIOA6*/
   GPIO_otype(GPIOA, PA6, EC_PUSH_PULL);   // 2.Push-Pull
   GPIO_pupd(GPIOA, PA6, EC_PU);               // 3.Pull-up
   GPIO_ospeed(GPIOA, PA6, EC_MEDIUM);    // 4.Medium speed
	 GPIO_write(GPIOA, PA6, HIGH);

   /*GPIOA7*/
   GPIO_otype(GPIOA, PA7, EC_PUSH_PULL);   // 2.Push-Pull
   GPIO_pupd(GPIOA, PA7, EC_PU);               // 3.Pull-up
   GPIO_ospeed(GPIOA, PA7, EC_MEDIUM);    // 4.Medium speed
	 GPIO_write(GPIOA, PA7, HIGH);

   /*GPIOB6*/
   GPIO_otype(GPIOB, PB6, EC_PUSH_PULL);   // 2.Push-Pull
   GPIO_pupd(GPIOB, PB6, EC_PU);               // 3.Pull-up
   GPIO_ospeed(GPIOB, PB6, EC_MEDIUM);    // 4.Medium speed
   GPIO_write(GPIOB, PB6, HIGH);

}

