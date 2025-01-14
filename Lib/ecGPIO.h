/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : SSS LAB
Created          : 05-03-2021
Modified         : 09-20-2022
Language/ver     : C++ in Keil uVision

Description      : Distributed to Students for LAB_GPIO
/----------------------------------------------------------------*/


#include "stm32f411xe.h"
#include "ecRCC.h"

#ifndef __ECGPIO_H
#define __ECGPIO_H

#define INPUT  0x00
#define OUTPUT 0x01
#define AF     0x02
#define ANALOG 0x03

#define HIGH 1
#define LOW  0

#define LED_PIN 	5
#define BUTTON_PIN 13

#define EC_NONE 0 // no pull-up, pull-down
#define EC_PU 1 // pull-up(01)
#define EC_PD 2 // pull-down(10)

#define EC_PUSH_PULL 0
#define EC_OPEN_DRAIN 1

#define EC_LOW 0
#define EC_MEDIUM 1
#define EC_FAST 2
#define EC_HIGH 3

#define PA5 5
#define PA6 6
#define PA7 7
#define PA8 8
#define PA9 9
#define PB6 6
#define PB10 10
#define PC7 7
#define PC13 13

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
	 
void GPIO_init(GPIO_TypeDef *Port, int pin, int mode);
void GPIO_write(GPIO_TypeDef *Port, int pin, int Output);
int  GPIO_read(GPIO_TypeDef *Port, int pin);
void GPIO_mode(GPIO_TypeDef* Port, int pin, int mode);
void GPIO_ospeed(GPIO_TypeDef* Port, int pin, int speed);
void GPIO_otype(GPIO_TypeDef* Port, int pin, int type);
void GPIO_pupd(GPIO_TypeDef* Port, int pin, int pupd);
void GPIO_setting(GPIO_TypeDef *Port, int pin, int mode, int otype, int pupd, int ospeed);
void LED_toggle(void);
void sevensegment_init(void);
void sevensegment_decoder(uint8_t  num);
void sevensegment_display_init(void);
void sevensegment_display(uint8_t  num);
void sevensegment_decoder_Midterm(uint8_t  state);


 
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
