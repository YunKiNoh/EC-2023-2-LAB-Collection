/**
******************************************************************************
* @author  SSSLAB
* @Mod     2023-11-15 by NohYunKi
* @brief   Embedded Controller:  LAB_USART_Bluetooth
*
******************************************************************************
*/

#include "stm32f4xx.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecUART.h"
#include "ecPWM.h"
#include "ecPinNames.h"

#define UP       'W'
#define RIGHT    'D'
#define LEFT     'A'
#define STOP     'S'


PinName_t Front_Right_Pin = PA_1; 
PinName_t Front_Left_Pin = PA_0;  


uint8_t btData = 0;
static volatile uint8_t Command = 0;

void setup(void);
void Direction(uint8_t direction);
void Direction_display(uint8_t direction);
void Car_setting(void);

int signal = 0;

int main(void) {
   // Initialiization --------------------------------------------------------
   setup();
		
   // Inifinite Loop ----------------------------------------------------------
   while (1){
      Direction(btData);
   }
}

void USART1_IRQHandler(){         //USART1 INT 
   if(is_USART_RXNE(USART1)){
      Direction_display(btData);
   }
}

// Initialiization 
void setup(void)
{
   RCC_PLL_init();
   UART1_init();
   UART1_baud(BAUD_9600);
   
	// LED setting
   GPIO_setting(GPIOA,LED_PIN,OUTPUT,EC_PUSH_PULL,EC_PU,EC_FAST);
			
   Car_setting();
	
   PWM_duty(Front_Right_Pin,1);
   PWM_duty(Front_Left_Pin,1);
}

void Car_setting(void){
   PWM_init(Front_Left_Pin); //Front_Left
   PWM_period_us(Front_Left_Pin,100);
   PWM_init(Front_Right_Pin); //Front_Right
   PWM_period_us(Front_Right_Pin,100);
}



void Direction(uint8_t direction){
          // Up
        if(direction == UP){
            PWM_duty(Front_Right_Pin,0.2); // 80%
            PWM_duty(Front_Left_Pin,0.2); // 80%
        }        
       // Right
        else if(direction == RIGHT){
            PWM_duty(Front_Right_Pin,0.5); // 50%
            PWM_duty(Front_Left_Pin,0.2);  // 80%           
        }
       // Left
        else if(direction == LEFT){
            PWM_duty(Front_Right_Pin,0.2);  // 80%
            PWM_duty(Front_Left_Pin,0.5);   // 50%
        }
       // Stop
        else if(direction == STOP){
						PWM_duty(Front_Right_Pin,1);
            PWM_duty(Front_Left_Pin,1);
        }
}

void Direction_display(uint8_t direction){   
   btData = USART_read(USART1);
	
   if(btData == UP){
      USART_write(USART1, (uint8_t*) "UP   ", 5);
   }
   else if(btData == STOP){
      USART_write(USART1, (uint8_t*) "STOP ", 5);
   }
   else if(btData == RIGHT){
      USART_write(USART1, (uint8_t*)"RIGHT", 5);
   }
   else if(btData == LEFT){
      USART_write(USART1, (uint8_t*) "LEFT ", 5);
   }
	 else if(btData == 'L'){
       btData = USART_read(USART1);
       if(btData == '0'){
          GPIO_write(GPIOA,5, 0);
          USART_write(USART1, (uint8_t*) "L0   ", 5);
       }
       else if(btData == '1'){
          GPIO_write(GPIOA,5, 1);
          USART_write(USART1, (uint8_t*) "L1   ", 5);
       }
   }
	 
   USART_write(USART1, "\r\n", 1);
}

