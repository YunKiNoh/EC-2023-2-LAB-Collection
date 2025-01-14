/**
******************************************************************************
* @author:  NohYunKi 	
* @date: 23-10-03
* @content:   23_2_Embedded Controller[LAB Digital In/Out 7-segment Display]
******************************************************************************
*/

#include "stm32f4xx.h"
#include "ecRCC.h"
#include "ecGPIO.h"

#define PA5 5
#define PA6 6
#define PA7 7
#define PA8 8
#define PA9 9
#define PB6 6
#define PB10 10
#define PC7 7
#define PC13 13

void setup(void);
void sevensegment_init(void);
void sevensegment_decoder(uint8_t  num);
void sevensegment_display_init(void);
void sevensegment_display(uint8_t  num);

int main(void) {	
	// Initialiization --------------------------------------------------------
	setup();
	unsigned int cnt = 0;
	
	// 7-Segment Decoder Programming(When using this part, mark under '7-Segment Decoder Hardware' part as a annotation) ----------------------------------------------------------
	/*while(1){
		// Input: 0 ~ 9
		sevensegment_decoder(cnt % 10);
		// Button pressed --> count 
		if(GPIO_read(GPIOC, BUTTON_PIN) == 0) cnt++; 
		// count > 9 --> reset
		if (cnt > 9) cnt = 0;
		// set volatile variable to maintain 'i' value in infivite loop.
		for(volatile int i = 0; i < 250000;i++){} 
	}*/
	
	// 7-Segment Decoder Hardware(When using this part, mark over '7-Segment Decoder Programming' part as a annotation) ----------------------------------------------------------
	while(1){
		// Input: 0 ~ 9
		sevensegment_display(cnt % 10);
		// Button pressed --> count 
		if(GPIO_read(GPIOC, BUTTON_PIN) == 0) cnt++; 
		// count > 9 --> reset
		if (cnt > 9) cnt = 0;
		// set volatile variable to maintain 'i' value in infivite loop.
		for(volatile int i = 0; i < 250000;i++){} 
	}
}

void setup(void){
	// common setting
	RCC_HSI_init();
	// In the case of Decoder Programming
	/*sevensegment_init();*/
	// In the case of Decoder Hardware
	sevensegment_display_init();
}

void sevensegment_init(void)
{
	// Array for Port's pin number
	int PA[5]={PA5,PA6,PA7,PA8,PA9};
	int PB[2]={PB6,PB10};
	
	//--------- 'GPIO_setting' is in 'ecGPIO.c' ---------//
	
	/*-----------------------------------------------------
   GPIOA setting: 
   Digital Out, Push-Pull, No Pull-up-Pull-down, Medium Speed
   ------------------------------------------------------*/
	for(int i = 0; i < 5; i++){
		GPIO_setting(GPIOA, PA[i], OUTPUT, EC_PUSH_PULL, EC_NONE, EC_MEDIUM);
	}
	
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
	GPIO_write(GPIOA, PA5, LOW);
	GPIO_write(GPIOA, PA6, LOW);
	GPIO_write(GPIOA, PA7, LOW);
	GPIO_write(GPIOA, PA8, LOW);
	GPIO_write(GPIOA, PA9, LOW);
	GPIO_write(GPIOB, PB6, LOW);
	GPIO_write(GPIOB, PB10, LOW);
	GPIO_write(GPIOC, PC7, HIGH);
}

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
