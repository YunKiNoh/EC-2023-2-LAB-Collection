/*----------------------------------------------------------------\
@ 23_2_Embedded Controller 
@ Student: NohYunKi
/----------------------------------------------------------------*/



#include "ecTIM.h"
#include "ecGPIO.h"
#include "math.h"
#include "ecPinNames.h"


/* Timer Configuration */
// msec = Timer's UEV Period
void TIM_init(TIM_TypeDef* TIMx, uint32_t msec){ 
	
// 1. Enable Timer CLOCK
  Tim_enable(TIMx);	
	
// 2. Set CNT period
	TIM_period_ms(TIMx, msec); 
	
// 3. CNT Direction
	//TIMx->CR1 &= ~(1<<4);				// Upcounter	
	TIMx->CR1 &= (1<<4);				// Downcounter	

	
// 4. Enable Timer Counter
	TIMx->CR1 |= TIM_CR1_CEN;		
}

//	Q. Which combination of PSC and ARR for msec unit?
// 	Q. What are the possible range (in sec ?)
void TIM_period_us(TIM_TypeDef *TIMx, uint32_t usec){   
	// Period usec = 1 to 1000

	// 1us(1MHz, ARR=1) to 65msec (ARR=0xFFFF)
	uint16_t PSCval;
	uint32_t Sys_CLK;
	
	if((RCC->CFGR & RCC_CFGR_SW_PLL) == RCC_CFGR_SW_PLL)
		Sys_CLK = 84000000;
	
	else if((RCC->CFGR & RCC_CFGR_SW_HSI) == RCC_CFGR_SW_HSI) 
		Sys_CLK = 16000000;
	
	
	if (TIMx == TIM2 || TIMx == TIM5){
		uint32_t ARRval;
		
		PSCval = Sys_CLK/1000000;									// 84 or 16	--> PSCval which makes counting frequency as 1MHz 
		ARRval = Sys_CLK/PSCval/1000000 * usec;		// this timer's counting period(= 1/counting frequency) = 1microsecond --> ARRval = timer's period = usec
		TIMx->PSC = PSCval - 1; // PSC
		TIMx->ARR = ARRval - 1; // ARR
	}
	else{
		uint16_t ARRval;

		PSCval = Sys_CLK/1000000;										// 84 or 16	--> f_cnt = 1MHz
		ARRval = Sys_CLK/PSCval/1000000 * usec;		// 1MHz(f_cnt)*usec = f_UEV(Update Event Value frequency)
		TIMx->PSC = PSCval - 1;
		TIMx->ARR = ARRval - 1;
	}			
}


void TIM_period_ms(TIM_TypeDef* TIMx, uint32_t msec){ 
	// Period msec = 1 to 6000
	
	 // 0.1ms(10kHz, ARR = 1, PSC = 8399) to 6.5sec (ARR = 0xFFFF)
	 /*uint16_t PSCval = Sys_CLK/10000;
	 uint16_t ARRval = Sys_CLK/PSCval/10000 * msec;  			// 84MHz/1000ms

	 TIMx->PSC = PSCval - 1;
	 TIMx->ARR = ARRval - 1;
	*/
	
	 uint16_t PSCval;
	 uint32_t Sys_CLK;
	
	if((RCC->CFGR & RCC_CFGR_SW_PLL) == RCC_CFGR_SW_PLL )
		Sys_CLK = 84000000; // Timer's frequency
	
	else if((RCC->CFGR & RCC_CFGR_SW_HSI) == RCC_CFGR_SW_HSI) 
		Sys_CLK = 16000000; // Timer's frequency
	
	
	if (TIMx == TIM2 || TIMx == TIM5){
		uint32_t ARRval;
		
		PSCval = Sys_CLK/100000;									// 840 or 160	--> f_cnt = 100kHz
		ARRval = Sys_CLK/PSCval/1000 * msec;		// 100kHz(f_cnt)*msec = f_UEV
		TIMx->PSC = PSCval - 1;
		TIMx->ARR = ARRval - 1;
	}
	else{
		uint16_t ARRval;

		PSCval = Sys_CLK/100000;									// 8400 or 1600	--> f_cnt = 10kHz
		ARRval = Sys_CLK/PSCval/1000 * msec;			// 10kHz*msec
		TIMx->PSC = PSCval - 1;
		TIMx->ARR = ARRval - 1;
	}
}


// Update Event Interrupt
void TIM_UI_init(TIM_TypeDef* TIMx, uint32_t msec){
// 1. Initialize Timer	
	TIM_init(TIMx,msec);
	
// 2. Enable Update Interrupt
	TIM_UI_enable(TIMx);
	
// 3. NVIC Setting
	uint32_t IRQn_reg = 0;
	if(TIMx == TIM1)       IRQn_reg = TIM1_UP_TIM10_IRQn;
	else if(TIMx == TIM2)  IRQn_reg = TIM2_IRQn;
	else if(TIMx == TIM3)  IRQn_reg = TIM3_IRQn;
	else if(TIMx == TIM4)  IRQn_reg = TIM4_IRQn;
	else if(TIMx == TIM5)  IRQn_reg = TIM5_IRQn;
	else if(TIMx == TIM9)  IRQn_reg = TIM1_BRK_TIM9_IRQn;
	else if(TIMx == TIM10)  IRQn_reg = TIM1_UP_TIM10_IRQn;
	else if(TIMx == TIM11)  IRQn_reg = TIM1_TRG_COM_TIM11_IRQn;
	// repeat for TIM3, TIM4, TIM5, TIM9, TIM10, TIM11
 
	NVIC_EnableIRQ(IRQn_reg);				
	NVIC_SetPriority(IRQn_reg,2);
}



void TIM_UI_enable(TIM_TypeDef* TIMx){
	TIMx->DIER |= 1;			// Enable Timer Update Interrupt		
}

void TIM_UI_disable(TIM_TypeDef* TIMx){
	TIMx->DIER &= 0;				// Disable Timer Update Interrupt		
}

uint32_t is_UIF(TIM_TypeDef *TIMx){
	return TIMx->SR & TIM_SR_UIF;
}

void clear_UIF(TIM_TypeDef *TIMx){
	TIMx->SR &= ~ TIM_SR_UIF;
}

void Tim_enable(TIM_TypeDef* TIMx){
	if(TIMx ==TIM1) RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	else if(TIMx ==TIM2) RCC->APB1ENR |= 1;
	else if(TIMx ==TIM3) RCC->APB1ENR |= 1<<1;
	else if(TIMx ==TIM4) RCC->APB1ENR |= 1<<2;
	else if(TIMx ==TIM5) RCC->APB1ENR |= 1<<3;
	else if(TIMx ==TIM9) RCC->APB2LPENR |= 1<<16;
	else if(TIMx ==TIM10) RCC->APB2LPENR |= 1<<17;
	else if(TIMx ==TIM11) RCC->APB2LPENR |= 1<<18;
}



/* -------- Timer Input Capture -------- */

void ICAP_init(PinName_t pinName){
// 0. Match Input Capture Port and Pin for TIMx
	GPIO_TypeDef *port;
	unsigned int pin;	
	ecPinmap(pinName, &port, &pin);	
	TIM_TypeDef *TIMx;
	int TIn;
	
	ICAP_pinmap(pinName, &TIMx, &TIn);
	int ICn = TIn;												        // (default) TIx=ICx

// GPIO configuration ---------------------------------------------------------------------	
// 1. Initialize GPIO port and pin as AF
	GPIO_setting(port,pin,AF,EC_PUSH_PULL, EC_PU, EC_HIGH);

// 2. Configure GPIO AFR by Pin num.
	if(TIMx == TIM1 || TIMx == TIM2){
		port->AFR[pin >> 3]	&= ~(15 << (4*(pin % 8))); 
		port->AFR[pin >> 3] |= 0x01 << (4*(pin % 8)); // TIM1~2
	}
	else if(TIMx == TIM3 || TIMx == TIM4 || TIMx == TIM5){
		port->AFR[pin >> 3]	&= ~(15 << (4*(pin % 8)));  
		port->AFR[pin >> 3] |= 0x02 << (4*(pin % 8)); // TIM3~5 
	}
	else if(TIMx == TIM9 || TIMx == TIM10|| TIMx == TIM11){
		port->AFR[pin >> 3]	&= ~(15 << (4*(pin % 8))); 
		port->AFR[pin >> 3] |= 0x03 << (4*(pin % 8)); // TIM9~11 
	}
	
// TIMER configuration ---------------------------------------------------------------------			
// 1. Initialize Timer Interrpt 
	TIM_UI_init(TIMx, 1);        					  // TIMx Interrupt initialize 

// 2. Modify ARR Maxium for 1MHz
	TIMx->PSC = 84-1;						  					// Timer counter clock: 1MHz(1us)  for PLL
	TIMx->ARR = 0xFFFF;											// Set auto reload register to maximum (count up to 65535)

// 3. Disable Counter during configuration
	TIMx->CR1 &= ~TIM_CR1_CEN;  						// Disable Counter during configuration


	
// Input Capture configuration ---------------------------------------------------------------------			
// 1. Select Timer channel(TIx) for Input Capture channel(ICx)
	// Default Setting
	TIMx->CCMR1 &= 	~TIM_CCMR1_CC1S;
	TIMx->CCMR1 &= 	~TIM_CCMR1_CC2S;
	TIMx->CCMR2 &= 	~TIM_CCMR2_CC3S;
	TIMx->CCMR2 &= 	~TIM_CCMR2_CC4S;
	TIMx->CCMR1 |= 	TIM_CCMR1_CC1S_0;      					//01<<0   CC1S    TI1=IC1
	TIMx->CCMR1 |= 	TIM_CCMR1_CC2S_0;  							//01<<8   CC2s    TI2=IC2
	TIMx->CCMR2 |= 	TIM_CCMR2_CC3S_0;        				//01<<0   CC3s    TI3=IC3
	TIMx->CCMR2 |= 	TIM_CCMR2_CC4S_0;  							//01<<8   CC4s    TI4=IC4


// 2. Filter Duration (use default)
	
// 3. IC Prescaler (use default)

// 4. Activation Edge: CCyNP/CCyP	
	TIMx->CCER &= ~(15UL<<((ICn-1)*4));     // clear
	TIMx->CCER &= ~(5UL<<(((ICn-1)*4)+1));  // Capture Operation : non-inverting and rising edge
	
// 5.	Enable CCy Capture, Capture/Compare interrupt
	TIMx->CCER |= 1UL << (4*(ICn - 1));							// CCn(ICn) Capture Enable	

// 6.	Enable Interrupt of CC(CCyIE), Update (UIE)
	TIMx->DIER |= 1 << (ICn);										// Capture/Compare Interrupt Enable	for ICn
	TIMx->DIER |= TIM_DIER_UIE;						  		// Update Interrupt enable	

// 7.	Enable Counter 
	TIMx->CR1	 |= TIM_CR1_CEN;	
}


// set TIMx's Chy as 'rising / falling / both' edge tpye
void ICAP_setup(PinName_t pinName, int ICn, int edge_type){
// 0. Match Input Capture Port and Pin for TIMx
	GPIO_TypeDef *port;
	unsigned int pin;	
	ecPinmap(pinName, &port, &pin);	
	TIM_TypeDef *TIMx;
	int CHn;		
	ICAP_pinmap(pinName, &TIMx, &CHn);

// 1. Disable  CC. Disable CCInterrupt for ICn. 
	TIMx->CCER &= ~(1 << (4*(ICn - 1))); 													// Capture Enable
	TIMx->DIER &= ~(1 << ICn); 																// CCn Interrupt enabled	
	
	
// 2. Configure  IC number(user selected) with given IC pin(TIMx_CHn)
	switch(ICn){
			case 1:
					TIMx->CCMR1 &= ~TIM_CCMR1_CC1S;											//reset   CC1S
					if (ICn==CHn) TIMx->CCMR1 |= 	TIM_CCMR1_CC1S_0;     //01<<0   CC1S    Tx_Ch1=IC1
					else TIMx->CCMR1 |= 	TIM_CCMR1_CC1S_1;      				//10<<0   CC1S    Tx_Ch2=IC1
					break;
			case 2:
					TIMx->CCMR1 &= ~TIM_CCMR1_CC2S;											//reset   CC2S
					if (ICn==CHn) TIMx->CCMR1 |= 	TIM_CCMR1_CC2S_0;     //01<<0   CC2S    Tx_Ch2=IC2
					else TIMx->CCMR1 |= 	TIM_CCMR1_CC2S_1;      				//10<<0   CC2S    Tx_Ch1=IC2
					break;
			case 3:
					TIMx->CCMR2 &= ~TIM_CCMR2_CC3S;											//reset   CC3S
					if (ICn==CHn) TIMx->CCMR2 |= TIM_CCMR2_CC3S_0;	   	//01<<0   CC3S    Tx_Ch3=IC3
					else TIMx->CCMR2 |= TIM_CCMR2_CC3S_1;		     				//10<<0   CC3S    Tx_Ch4=IC3
					break;
			case 4:
					TIMx->CCMR2 &= ~TIM_CCMR2_CC4S;										  //reset   CC4S
					if (ICn==CHn) TIMx->CCMR2 |= TIM_CCMR2_CC4S_0;	   	//01<<0   CC4S    Tx_Ch4=IC4
					else TIMx->CCMR2 |= TIM_CCMR2_CC4S_1;	     					//10<<0   CC4S    Tx_Ch3=IC4
					break;
			default: break;
		}


// 3. Configure Activation Edge direction
	TIMx->CCER  &= ~(0b1010 << 4*(ICn - 1));  					            // Clear CCnNP/CCnP bits
	switch(edge_type){
		case IC_RISE: TIMx->CCER &= ~(0b1010 << 4*(ICn - 1)); break;  //rising:  00
		case IC_FALL: TIMx->CCER |= 0b0010 << 4*(ICn-1);	 break;    //0b0010 << 4*(ICn-1)       		  //falling: 01
		case IC_BOTH: TIMx->CCER |= 0b1010 << 4*(ICn-1);	 break;    //ob1010 << 4*(ICn-1)    				//both:    11
	}
	//TIM_CCER_CC1NP | TIM_CCER_CC1P
// 4. Enable CC. Enable CC Interrupt. 
	TIMx->CCER |= 1 << (4*(ICn - 1)); 										// Capture Enable
	TIMx->DIER |= 1 << ICn; 															// CCn Interrupt enabled	
}

// Set TIMx_Chy's counter step value as 'usec'
void ICAP_counter_us(PinName_t pinName, int usec){
// 0. Match Input Capture Port and Pin for TIMx	
	GPIO_TypeDef *port;
	unsigned int pin;	
	ecPinmap(pinName, &port, &pin);	
	TIM_TypeDef *TIMx;
	int CHn;		
	ICAP_pinmap(pinName, &TIMx, &CHn);

// 1. Configuration Timer Prescaler and ARR
	TIMx->PSC = 84*usec-1;						  // Timer counter clock: 1us * usec
	TIMx->ARR = 0xFFFF;									// Set auto reload register to maximum (count up to 65535)
}

uint32_t is_CCIF(TIM_TypeDef *TIMx, uint32_t ccNum){
	return (TIMx->SR & (0x1UL << ccNum)) != 0;	
}

void clear_CCIF(TIM_TypeDef *TIMx, uint32_t ccNum){
	TIMx->SR &= ~(1 << ccNum);
}

uint32_t ICAP_capture(TIM_TypeDef* TIMx, uint32_t ICn){
	uint32_t capture_Value;
	
	if (ICn == 1)
		capture_Value = TIMx->CCR1;
	else if (ICn == 2)
		capture_Value = TIMx->CCR2;
	else if (ICn == 3)
		capture_Value = TIMx->CCR3;
	else
		capture_Value = TIMx->CCR4;

	return capture_Value;
}

//DO NOT MODIFY THIS
void ICAP_pinmap(PinName_t pinName, TIM_TypeDef **TIMx, int *chN){
	 GPIO_TypeDef *port;
	 unsigned int pin;		
	 ecPinmap(pinName, &port, &pin);	
   
   if(port == GPIOA) {
      switch(pin){
         case 0 : *TIMx = TIM2; *chN = 1; break;
         case 1 : *TIMx = TIM2; *chN = 2; break;
         case 5 : *TIMx = TIM2; *chN = 1; break;
         case 6 : *TIMx = TIM3; *chN = 1; break;
         //case 7: *TIMx = TIM1; *chN = 1N; break;
         case 8 : *TIMx = TIM1; *chN = 1; break;
         case 9 : *TIMx = TIM1; *chN = 2; break;
         case 10: *TIMx = TIM1; *chN = 3; break;
         case 15: *TIMx = TIM2; *chN = 1; break;
         default: break;
      }         
   }
   else if(port == GPIOB) {
      switch(pin){
         //case 0: *TIMx = TIM1; *chN = 2N; break;
         //case 1: *TIMx = TIM1; *chN = 3N; break;
         case 3 : *TIMx = TIM2; *chN = 2; break;
         case 4 : *TIMx = TIM3; *chN = 1; break;
         case 5 : *TIMx = TIM3; *chN = 2; break;
         case 6 : *TIMx = TIM4; *chN = 1; break;
         case 7 : *TIMx = TIM4; *chN = 2; break;
         case 8 : *TIMx = TIM4; *chN = 3; break;
         case 9 : *TIMx = TIM4; *chN = 3; break;
         case 10: *TIMx = TIM2; *chN = 3; break;
         
         default: break;
      }
   }
   else if(port == GPIOC) {
      switch(pin){
         case 6 : *TIMx = TIM3; *chN = 1; break;
         case 7 : *TIMx = TIM3; *chN = 2; break;
         case 8 : *TIMx = TIM3; *chN = 3; break;
         case 9 : *TIMx = TIM3; *chN = 4; break;
         
         default: break;
      }
   }
}
