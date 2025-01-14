/**
******************************************************************************
* @author   Noh YunKi
* @Mod      23/09/21
* @brief   Embedded Controller:  LAB Digital In/Out
*                - Toggle LED LD2 by Button B1 pressing
* 
******************************************************************************
*/

#include "stm32f4xx.h"
#include "ecRCC.h"
#include "ecGPIO.h"

#define LED_PIN    5
#define BUTTON_PIN 13



void setup(void);
   
int main(void) { 
   // Initialiization --------------------------------------------------------
   setup();
   


   // Inifinite Loop ----------------------------------------------------------
   while(1){
      if(GPIO_read(GPIOC, BUTTON_PIN) == 0){
         if(GPIO_read(GPIOA, LED_PIN) == LOW) GPIO_write(GPIOA, LED_PIN, HIGH);
         else GPIO_write(GPIOA, LED_PIN, LOW);
				 for(volatile int i = 0; i < 200000; i++){}
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
   Digital Output, GPIOA5, Open_darin, pull-up, Medium Speed 
   ------------------------------------------------------*/
   GPIO_init(GPIOA, LED_PIN, OUTPUT);    // 0.Enable & 1.Output
   GPIO_otype(GPIOA, LED_PIN, 0);            // 2.oepn_darin
   GPIO_pupd(GPIOA, LED_PIN, EC_PU);         // 3.Pull-up
   GPIO_ospeed(GPIOA, LED_PIN, EC_FAST); // 4.Fast speed
   
}