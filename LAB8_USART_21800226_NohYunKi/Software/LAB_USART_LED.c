/**
******************************************************************************
* @author  SSSLAB
* @Mod     2023-11-15 by NohYunKi
* @brief   Embedded Controller:  LAB_USART_LED
*
******************************************************************************
*/

#define LED_PIN    5
#include "stm32f4xx.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecUART.h"
#include "ecSysTick.h"

static volatile uint8_t PC_Data = 0;
static volatile uint8_t BT_Data = 0;

void setup(void){
  RCC_PLL_init();
   SysTick_init();

  USART_setting(USART1, GPIOA,9,GPIOA,10, BAUD_38400); // PA9 - RXD , PA10 - TXD
   
  // USB serial init
  UART2_init();
  UART2_baud(BAUD_38400);

  // BT serial init 
  UART1_init();
  UART1_baud(BAUD_38400);
   
   GPIO_init(GPIOA, LED_PIN, OUTPUT);    // calls RCC_GPIOA_enable()
   GPIO_otype(GPIOA, LED_PIN, 0);        //  GPIOA LED_PIN Output Type: Output open drain (1)
   GPIO_ospeed(GPIOA, LED_PIN, EC_FAST);      // GPIOA Speed LED_PIN Medium speed(01)

}


void main(){   
   setup();
    
   while(1){
             
             if(BT_Data==0X4C){  //(L)
         GPIO_write(GPIOA,5, 0);
      }else if(BT_Data==0X48){ //(H)
         GPIO_write(GPIOA,5, 1);
         }
      
    }
}


void USART2_IRQHandler(){                // USART2 RX Interrupt : Recommended
   if(is_USART2_RXNE()){
      PC_Data = USART2_read();      // RX from UART2 (PC)
      USART1_write(&PC_Data,1);      // TX to USART1    (BT)   
      printf("MCU_1 sent : %c \r\n",PC_Data); // TX to USART2(PC)
   }
}

void USART1_IRQHandler(){                // USART2 RX Interrupt : Recommended
   if(is_USART1_RXNE()){
    BT_Data = USART1_read();                              // RX from UART1 (BT)      
      printf("MCU_1 received : %c \r\n",BT_Data); // TX to USART2(PC)
   }
}
