/*----------------------------------------------------------------\
@ 23_2_Embedded Controller 
@ Student: NohYunKi
/----------------------------------------------------------------*/



#include "stm32f4xx.h"
#include "stm32f411xe.h"
#include "ecGPIO.h"

void GPIO_init(GPIO_TypeDef *Port, int pin, int mode){     
	// mode  : Input(0), Output(1), AlterFunc(2), Analog(3)   
	if (Port == GPIOA)
		RCC_GPIOA_enable();
	if (Port == GPIOB)
		RCC_GPIOB_enable();
	if (Port == GPIOC)
		RCC_GPIOC_enable();
	if (Port == GPIOD)
		RCC_GPIOD_enable();
	//[TO-DO] YOUR CODE GOES HERE
	// Make it for GPIOB, GPIOD..GPIOH

	// You can also make a more general function of
	// void RCC_GPIO_enable(GPIO_TypeDef *Port); 

	GPIO_mode(Port, pin, mode);
	
}


// GPIO Mode          : Input(00), Output(01), AlterFunc(10), Analog(11)
void GPIO_mode(GPIO_TypeDef *Port, int pin, int mode){
   Port->MODER &= ~(3UL<<(2*pin));     
   Port->MODER |= mode<<(2*pin);    
}


// GPIO Speed          : Low speed (00), Medium speed (01), Fast speed (10), High speed (11)
void GPIO_ospeed(GPIO_TypeDef *Port, int pin, int speed){
	Port->OSPEEDR &= ~(3UL<<(pin*2));  
	Port->OSPEEDR |= speed<<(pin*2);
}

// GPIO Output Type: Output push-pull (0, reset), Output open drain (1)
void GPIO_otype(GPIO_TypeDef *Port, int pin, int type){
   	//[TO-DO] YOUR CODE GOES HERE
	//[TO-DO] YOUR CODE GOES HERE
	Port->OTYPER &= ~(1UL<<pin);
	Port->OTYPER |= (type<<pin);
}

// GPIO Push-Pull    : No pull-up, pull-down (00), Pull-up (01), Pull-down (10), Reserved (11)
void GPIO_pupd(GPIO_TypeDef *Port, int pin, int pupd){
   	//[TO-DO] YOUR CODE GOES HERE
	//[TO-DO] YOUR CODE GOES HERE
	Port->PUPDR &= ~(3UL<<(pin*2));
	Port->PUPDR |= pupd<<(pin*2);
}

int GPIO_read(GPIO_TypeDef *Port, int pin){
   	//[TO-DO] YOUR CODE GOES HERE
	//[TO-DO] YOUR CODE GOES HERE
	int Val = (Port->IDR)>>pin & 1;
	return Val;    	
}

void GPIO_write(GPIO_TypeDef *Port, int pin, int Output){
	Port->ODR &= ~(1UL<<pin);
	Port->ODR |= Output<< pin;
}

void GPIO_setting(GPIO_TypeDef *Port, int pin, int mode, int otype, int pupd, int ospeed){
	GPIO_init(Port, pin, mode);
	GPIO_otype(Port, pin, otype);
	GPIO_pupd(Port, pin, pupd);
	GPIO_ospeed(Port, pin, ospeed);
}

void LED_toggle(void){
	GPIOA->ODR ^= (1<<LED_PIN);
}

// Setting of Decoder Software
void sevensegment_init(void)
{
	// Array for Port's pin number
	int PA[5]={2,PA6,PA7,PA8,PA9};
	int PB[2]={PB6,PB10};
	
	//--------- 'GPIO_setting' is in 'ecGPIO.c' ---------//
	
	/*-----------------------------------------------------
   GPIOA setting: 
   Digital Out, Push-Pull, No Pull-up-Pull-down, Medium Speed
   ------------------------------------------------------*/
	for(int i = 0; i < 5; i++){
		GPIO_setting(GPIOA, PA[i], OUTPUT, EC_PUSH_PULL, EC_NONE, EC_MEDIUM);
	}
	GPIO_setting(GPIOA, 2, OUTPUT, EC_PUSH_PULL, EC_NONE, EC_MEDIUM);
	/*-----------------------------------------------------
   GPIOB setting: 
   Digital Out, Push-Pull, No Pull-up-Pull-down, Medium Speed
   ------------------------------------------------------*/
	for(int i = 0; i < 2; i ++){
		GPIO_setting(GPIOB, PB[i], OUTPUT, EC_PUSH_PULL, EC_NONE, EC_MEDIUM);
	}
	
	/*-----------------------------------------------------
   GPIOC7 setting: 
   Digital Out, Push-Pull, No Pull-up-Pull-down, Medium Speed
   ------------------------------------------------------*/
	GPIO_setting(GPIOC, PC7, OUTPUT, EC_PUSH_PULL, EC_NONE, EC_MEDIUM);
	
	/*-----------------------------------------------------
   GPIOC13 setting: 
   Digital In, Pull-up	
   ------------------------------------------------------*/
	GPIO_init(GPIOC, 13, INPUT);
	GPIO_pupd(GPIOA, PA7, EC_PU);				  // 3.Pull-up
	
	// Initial setting '0'
	GPIO_write(GPIOA, 2, LOW);
	GPIO_write(GPIOA, PA6, LOW);
	GPIO_write(GPIOA, PA7, LOW);
	GPIO_write(GPIOA, PA8, LOW);
	GPIO_write(GPIOA, PA9, LOW);
	GPIO_write(GPIOB, PB6, LOW);
	GPIO_write(GPIOB, PB10, LOW);
	GPIO_write(GPIOC, PC7, HIGH);
}

// Decoder Software
void sevensegment_decoder(uint8_t  num)
{	
	/* 
	Matrix for each number
	: {PA5,PA6,PA7,PA8,PA9,PB6,PB10,PC10}x10
	*/
	int number[10][8]={
		{0,0,0,0,0,0,0,1},  //	zero
		{1,1,0,1,1,0,0,1},  //	one
		{0,0,1,0,1,0,0,0},  //	two
		{1,0,0,0,1,0,0,0},  //	three
		{1,1,0,1,0,0,0,0},  //	four
		{1,0,0,0,0,0,1,0},  //	five
		{0,0,0,0,0,0,1,0},  //	six
		{1,1,0,0,1,0,0,1},  //	seven
		{0,0,0,0,0,0,0,0},  //	eight
		{1,1,0,0,0,0,0,0}   //	nine
	};
	
	// Array for Port pin's number
	int PA[5]={PA5,PA6,PA7,PA8,PA9};
	int PB[2]={PB6,PB10};
	
	// Turn on or off each LED pins
	for(int i=0; i < 8; i++){
		if(i < 5) GPIO_write(GPIOA, PA[i], number[num][i]); // PA5 ~ PA9
		else if(i == 5 || i == 6) GPIO_write(GPIOB, PB[i-5], number[num][i]); // PB6, PB10
		else GPIO_write(GPIOC, PC7, number[num][i]); // PC10
	}
}

// Setting for Decoder Hardware
void sevensegment_display_init(void)
{
	/*-----------------------------------------------------
   GPIOC13 setting: 
   Digital In, Pull-up	
   ------------------------------------------------------*/
	GPIO_init(GPIOC, 13, INPUT);
	GPIO_pupd(GPIOA, PA7, EC_PU);				  // 3.Pull-up

	
	/*-----------------------------------------------------
   GPIOA7,9 & GPIOB6 & GPIOC7 setting: 
   Digital Out, Push-Pull, No Pull-up-Pull-down, Medium Speed
   ------------------------------------------------------*/
	GPIO_setting(GPIOA, PA7, OUTPUT, EC_PUSH_PULL, EC_NONE, EC_MEDIUM);
	GPIO_setting(GPIOA, PA9, OUTPUT, EC_PUSH_PULL, EC_NONE, EC_MEDIUM);
	GPIO_setting(GPIOB, PB6, OUTPUT, EC_PUSH_PULL, EC_NONE, EC_MEDIUM);
	GPIO_setting(GPIOC, PC7, OUTPUT, EC_PUSH_PULL, EC_NONE, EC_MEDIUM);
}

// Decoder Hardware
void sevensegment_display(uint8_t  num)
{
	int DCBA[10][4] = {
		{0,0,0,0},	// 0
		{1,0,0,0},	// 1
		{0,1,0,0},	// 2
		{1,1,0,0},	// 3
		{0,0,1,0},	// 4
		{1,0,1,0},	// 5
		{0,1,1,0},	// 6
		{1,1,1,0},	// 7
		{0,0,0,1},	// 8
		{1,0,0,1},	// 9
	};
	
	GPIO_write(GPIOA, PA7, DCBA[num][0]);
	GPIO_write(GPIOB, PB6, DCBA[num][1]);
	GPIO_write(GPIOC, PC7, DCBA[num][2]);
	GPIO_write(GPIOA, PA9, DCBA[num][3]);
}

void sevensegment_decoder_Midterm(uint8_t  state)
{	
	 
	//Matrix for each number
	//: {PA5,PA6,PA7,PA8,PA9,PB6,PB10,PC10}x10
	
	if(state == 0) 
	{
		GPIO_write(GPIOA,2,0);
		GPIO_write(GPIOA,6,0);
		GPIO_write(GPIOA,7,0);
		GPIO_write(GPIOA,8,0);
		GPIO_write(GPIOA,9,0);
		GPIO_write(GPIOB,6,0);
		GPIO_write(GPIOB,10,0);
		GPIO_write(GPIOC,7,1);
	}
	else if(state == 1) 
	{
		GPIO_write(GPIOA,2,0);
		GPIO_write(GPIOA,6,0);
		GPIO_write(GPIOA,7,1);
		GPIO_write(GPIOA,8,1);
		GPIO_write(GPIOA,9,0);
		GPIO_write(GPIOB,6,0);
		GPIO_write(GPIOB,10,1);
		GPIO_write(GPIOC,7,1);
	}
	else if(state == 2) 
	{
		GPIO_write(GPIOA,2,0);
		GPIO_write(GPIOA,6,1);
		GPIO_write(GPIOA,7,0);
		GPIO_write(GPIOA,8,1);
		GPIO_write(GPIOA,9,0);
		GPIO_write(GPIOB,6,0);
		GPIO_write(GPIOB,10,0);
		GPIO_write(GPIOC,7,0);
	}
}

