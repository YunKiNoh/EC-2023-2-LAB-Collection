/*----------------------------------------------------------------\
@ 23_2_Embedded Controller 
@ Student: NohYunKi
/----------------------------------------------------------------*/



#include "ecSysTick.h"
#define MCU_CLK_PLL 84000000 //	84MHz
#define MCU_CLK_HSI 16000000 //	16MHz

volatile uint32_t msTicks=0;

// SysTick' period = 1ms
void SysTick_init(void){	
	//  SysTick Control and Status Register
	SysTick->CTRL = 0;											// Disable SysTick IRQ and SysTick Counter

	// Select processor clock
	// 1 = processor clock;  0 = external clock
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;

	// uint32_t MCU_CLK=EC_SYSTEM_CLK
	// SysTick Reload Value Register
	SysTick->LOAD = MCU_CLK_PLL / 1000 - 1;						// Interrupt Time Period = 1ms(Update Event Period), Clock Period = 1 / MCU_CLK_PLL(countig Period)
	//SysTick->LOAD = MCU_CLK_PLL / 2000 - 1;						// Interrupt Time Period = 0.5ms
	
	// SysTick Current Value Register
	SysTick->VAL = 0;

	// Enables SysTick exception request
	// 1 = counting down to zero asserts the SysTick exception request
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
	
	// Enable SysTick IRQ and SysTick Timer
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
		
	NVIC_SetPriority(SysTick_IRQn, 1);		// Set Priority to 1
	NVIC_EnableIRQ(SysTick_IRQn);			// Enable interrupt in NVIC
}



void SysTick_Handler(void){
	SysTick_counter();	
}


void SysTick_counter(){
	msTicks++;
}	


void delay_ms (uint32_t mesc){
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < mesc);
	
	msTicks = 0;
}


void SysTick_reset(void)
{
	// SysTick Current Value Register
	SysTick->VAL = 0;
}

uint32_t SysTick_val(void) {
	return SysTick->VAL;
}

//void SysTick_counter(){
//	msTicks++;
//	if(msTicks%1000 == 0) count++;
//}	
