/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
name: NohYunKi
/----------------------------------------------------------------*/



#include "ecGPIO.h"
#include "ecSysTick.h"
#include "ecEXTI.h"


void EXTI_init(GPIO_TypeDef *Port, int Pin, int trig_type,int priority){

   // SYSCFG peripheral clock enable   
   RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;      
   
   // Connect External Line to the GPIO
   int EXTICR_port;
   if         (Port == GPIOA) EXTICR_port = 0;
   else if   (Port == GPIOB) EXTICR_port = 1;
   else if   (Port == GPIOC) EXTICR_port = 2;
   else if   (Port == GPIOD) EXTICR_port = 3;
   else                               EXTICR_port = 4;
   
   SYSCFG->EXTICR[Pin/4] &= ~(15UL<<4*(Pin%4));         // clear 4 bits
   SYSCFG->EXTICR[Pin/4] |= EXTICR_port<<4*(Pin%4);         // set connect with proposed Port
   
   // Configure Trigger edge
   if (trig_type == FALL) EXTI->FTSR |= 1UL<<Pin;   // Falling trigger enable 
   else if   (trig_type == RISE) EXTI->RTSR |= 1UL<<Pin;   // Rising trigger enable 
   else if   (trig_type == BOTH) {         // Both falling/rising trigger enable
      EXTI->RTSR |= 1UL<<Pin; 
      EXTI->FTSR |= 1UL<<Pin;
   } 
   
   // Configure Interrupt Mask (Interrupt enabled)
   EXTI_enable(Pin);     // not masked
   
   
   // NVIC(IRQ) Setting
   int EXTI_IRQn = 0;
   
   if (Pin < 5)    EXTI_IRQn = Pin+6;
   else if   (Pin < 10)    EXTI_IRQn = 23;
   else          EXTI_IRQn = 40;
                        
   NVIC_SetPriority(EXTI_IRQn, priority);   // EXTI priority
   NVIC_EnableIRQ(EXTI_IRQn);    // EXTI IRQ enable
}


void EXTI_enable(uint32_t pin) {
   EXTI->IMR |= 1UL<<pin;     // not masked (i.e., Interrupt enabled)
}
void EXTI_disable(uint32_t pin) {
   EXTI->IMR &= ~(1UL<<pin);     // masked (i.e., Interrupt disabled)
}

uint32_t is_pending_EXTI(uint32_t pin){
   uint32_t EXTI_PRx = 1<<pin;        // check  EXTI pending    
   return ((EXTI->PR & EXTI_PRx) == EXTI_PRx);
}

void clear_pending_EXTI(uint32_t pin){
   EXTI->PR |= 1<<pin;     // clear EXTI pending 
}