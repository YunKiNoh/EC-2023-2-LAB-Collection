/*
******************************************************************************
* @author   Noh Yun Ki
* @Mod      2023/11/24
* @brief    Embedded Controller:  LAB_RCcar
* 
******************************************************************************
*/

#include "ecSTM32F411.h"

// State Signal
#define GO               0x49  //I
#define BACK           0x4B  //K
#define RIGHT         0x4C  //L
#define LEFT          0x4A  //J
#define STOP           0x53  //S
#define FORWARD        0x46  //F
#define BACKWORD     0x42  //B
#define REST            0x52  //R
#define Control         0x4D  //M
#define AUTO            0x41  //A

// Ultra Sonic
#define TRIG PA_6
#define ECHO PB_6

// Auto Mode display signal
#define auto_stop		0
#define auto_up   1
#define auto_right   2
#define auto_left   3

static volatile int speed = 0; // speed value
static volatile int direc = 1; // Forward/Backward
static volatile int STR = 3; // Steering
static volatile int MODE = 0; // Auto/Manual

static volatile int cnt = 0; // Counting variable for blink LED 
uint32_t ovf_cnt = 0; // Ultra Sonic Overflow Count
uint32_t auto_cnt = 0; // Auto Mode Display periond
static volatile float distance = 0; // Ultra Sonic Distnace
float timeInterval = 0; // Ultra Sonic timeInterval
float time1 = 0; // Rising capture
float time2 = 0; // Falling capture
int display_auto = 0; // Counting variable for Auto Display

uint8_t btData = 0;

int flag = 0;
PinName_t seqCHn[2] = {PB_0, PB_1};

static volatile uint32_t right_IR, left_IR; 

void MCU_init(void);
void Direction_display(uint8_t direction);
void Display_MState(void);
void Display_AState(void);
void Control_mode(void);
void Auto_mode(void);
void USART1_IRQHandler(void);
void TIM4_IRQHandler(void);
void ADC_IRQHandler(void);



int main(void) {
   // Initialiization --------------------------------------------------------
   MCU_init();   
   // Inifinite Loop ----------------------------------------------------------
   while (1){
       //Auto_Display_State(MODE, drirec)
            
       
       //Manual_Set_State(MODE, direc, drc, speed);
        if(btData == Control){
            MODE=1;
            speed = 0;
            STR = 3;
            GPIO_write(GPIOC, 2, 1);
            GPIO_write(GPIOC, 3, 1);
            PWM_duty(PA_0, (float)(1));
            PWM_duty(PA_1, (float)(1));
            GPIO_write(GPIOA, 5, 1);
         }else if(btData == AUTO){
            MODE=2;
            speed = 0;
            STR = 3;
            GPIO_write(GPIOC, 2, 0);
            GPIO_write(GPIOC, 3, 0);
         }else if(btData == REST){
            MODE=0;
            speed = 0;
            STR = 3;
            GPIO_write(GPIOC, 2, 1);
            GPIO_write(GPIOC, 3, 1);
            PWM_duty(PA_0, (float)(1));
            PWM_duty(PA_1, (float)(1));
            GPIO_write(GPIOA, 5, 0);
         }
         
         
         if(MODE ==1){
            Control_mode();
            Display_MState();
            
         }else if(MODE ==2){
            Auto_mode();
            Display_AState();
               
         }else if(MODE ==0){
            GPIO_write(GPIOC, 2, 1);
            GPIO_write(GPIOC, 3, 1);
            PWM_duty(PA_0, (float)(1));
            PWM_duty(PA_1, (float)(1));
         }
   
         
   }
}


void USART1_IRQHandler(){         //USART1 INT 
   if(is_USART_RXNE(USART1)){
     Direction_display(btData);
       
   }
}

void TIM4_IRQHandler(){
   if(is_UIF(TIM4)){                     // Update interrupt
      
      distance = timeInterval * ((float)(340.0 / 2.0 / 10.0));   // [mm] -> [cm]
      
      ovf_cnt++;         // overflow count
      cnt++;
      auto_cnt++;
      
      clear_UIF(TIM4);                           // clear update interrupt flag
   }
   
   if(is_CCIF(TIM4, 1)){                         // TIM4_Ch1 (IC1) Capture Flag. Rising Edge Detect
      time1 = TIM4 -> CCR1;                           // Capture TimeStart
      clear_CCIF(TIM4, 1);                // clear capture/compare interrupt flag 
   }                                              
   else if(is_CCIF(TIM4, 2)){                            // TIM4_Ch2 (IC2) Capture Flag. Falling Edge Detect
      time2 = TIM4 -> CCR2;                           // Capture TimeEnd
      timeInterval = ((time2 - time1) + ovf_cnt * (TIM4->ARR + 1))/100;    // (10us * counter pulse -> [msec] unit) Total time of echo pulse
      ovf_cnt = 0;                        // overflow reset
      clear_CCIF(TIM4,2);                          // clear capture/compare interrupt flag 
   }
}

void ADC_IRQHandler(void){
   if(is_ADC_OVR())
      clear_ADC_OVR();
   
   if(is_ADC_EOC()){      // after finishing sequence
      if (flag==1)
         right_IR = ADC_read();  
      else if (flag==0)
         left_IR = ADC_read();
         
      flag =! flag;      // flag toggle
   }
}



//Print the current direction state based on the pressed key
void Direction_display(uint8_t direction){   
     btData = USART_read(USART1);
   
   if(btData == GO){
      USART_write(USART1, (uint8_t*) "SPEED UP", 8);
   }
    else if(btData == BACK){
      USART_write(USART1, (uint8_t*) "SPEED DOWN", 10);
   }
   else if(btData == RIGHT){
      USART_write(USART1, (uint8_t*)"TURN RIGHT", 10);
   }
   else if(btData == LEFT){
      USART_write(USART1, (uint8_t*) "TURN LEFT", 9);
   }
   else if(btData == STOP){
      USART_write(USART1, (uint8_t*) "STOP", 4);
   }
    else if(btData == FORWARD){
      USART_write(USART1, (uint8_t*) "GO", 2);
   }
    else if(btData == BACKWORD){
      USART_write(USART1, (uint8_t*) "BACK ", 4);
   }
    else if(btData == Control){
          USART_write(USART1, (uint8_t*) "CONTROL MODE", 12);
    }
    else if(btData == AUTO){
          USART_write(USART1, (uint8_t*) "LINE TRACE MODE", 15);
       }
    else if(btData == REST){
          USART_write(USART1, (uint8_t*) "Select a Mode", 13);
       }
   USART_write(USART1, "\r\n", 2);
}



void Control_mode(void){
   
               if(btData == FORWARD){
                     direc=1;
               }else if(btData == BACKWORD){
                     direc=0;
               }else if(btData == GO){
                     speed++;
                     if(speed>3){
                        speed=3;
                     }
               }else if(btData == BACK){
                     speed--;
                     if(speed<0){
                           speed=0;
                     }
               }else if(btData == LEFT){
                     STR--;
                     if(STR<0){
                           STR=0;
                     }
               }else if(btData == RIGHT){
                     STR++;
                     if(STR>6){
                           STR=6;
                     }
               }else if(btData == STOP){
                     direc=1;
                     speed=0;
                     STR=3;
               }
               
               btData =0x35; // clear btData
               GPIO_write(GPIOC, 2, direc);  //Direction out=1 CW Direction out=0 CCW
               GPIO_write(GPIOC, 3, direc);
               
               if(direc == 1){
              if(speed == 0 && STR == 3 || distance < 15){
                    PWM_duty(PA_0, (float)(1));
                    PWM_duty(PA_1, (float)(1));
              }else if(speed==1 && STR==3){
                    PWM_duty(PA_0, (float)(1*0.7));
                    PWM_duty(PA_1, (float)(1*0.7));
              }else if(speed==2 && STR==3){
                    PWM_duty(PA_0, (float)(1*0.4));
                    PWM_duty(PA_1, (float)(1*0.4));
              }else if(speed==3 && STR==3){
                    PWM_duty(PA_0, (float)(1*0.1));
                    PWM_duty(PA_1, (float)(1*0.1));
             }else if(STR==2){                    //LEFT
                    PWM_duty(PA_0, (float)(1*0.6));
                    PWM_duty(PA_1, (float)(1*0.4));
             }else if(STR==1){
                   PWM_duty(PA_0, (float)(1*0.7));
                    PWM_duty(PA_1, (float)(1*0.4));
             }else if(STR==0){
                   PWM_duty(PA_0, (float)(1*0.9));
                   PWM_duty(PA_1, (float)(1*0.4));
             }else if(STR==4){                   //RIGHT
                    PWM_duty(PA_0, (float)(1*0.4));
                   PWM_duty(PA_1, (float)(1*0.6));
             }else if(STR==5){
                    PWM_duty(PA_0, (float)(1*0.4));
                    PWM_duty(PA_1, (float)(1*0.7));
              }else if(STR==6){
                    PWM_duty(PA_0, (float)(1*0.4));
                    PWM_duty(PA_1, (float)(1*0.9));
                    }
        }else if(direc == 0){
              if(speed==0 && STR==3 || distance<15){
                    PWM_duty(PA_0, (float)(0));
                     PWM_duty(PA_1, (float)(0));
              }else if(speed==1 && STR==3){
                    PWM_duty(PA_0, (float)(1*0.3));
                   PWM_duty(PA_1, (float)(1*0.3));
              }else if(speed==2 && STR==3){
                    PWM_duty(PA_0, (float)(1*0.6));
                     PWM_duty(PA_1, (float)(1*0.6));
              }else if(speed==3 && STR==3){
                     PWM_duty(PA_0, (float)(1*0.9));
                    PWM_duty(PA_1, (float)(1*0.9));
              }else if(STR==2){                    //LEFT
                     PWM_duty(PA_0, (float)(1*0.4));
                     PWM_duty(PA_1, (float)(1*0.6));
               }else if(STR==1){
                     PWM_duty(PA_0, (float)(1*0.4));
                     PWM_duty(PA_1, (float)(1*0.7));
              }else if(STR==0){
                      PWM_duty(PA_0, (float)(1*0.4));
                     PWM_duty(PA_1, (float)(1*0.9));
              }else if(STR==4){                   //RIGHT
                      PWM_duty(PA_0, (float)(1*0.4));
                      PWM_duty(PA_1, (float)(1*0.4));
              }else if(STR==5){
                     PWM_duty(PA_0, (float)(1*0.7));
                      PWM_duty(PA_1, (float)(1*0.4));
               }else if(STR==6){
                      PWM_duty(PA_0, (float)(1*0.9));
                      PWM_duty(PA_1, (float)(1*0.4));
               }
         }
}


void Auto_mode(void){
   
   // Stop
        if(distance < 15){
            PWM_duty(PA_0, (float)0);
            PWM_duty(PA_1, (float)0);
                  display_auto = auto_stop;
            }
            else if(distance > 15){
               if(right_IR < 1300 && left_IR < 1300){
            PWM_duty(PA_0,(float)1); // 80%
            PWM_duty(PA_1,(float)1); // 80%
                 display_auto = auto_up;
               }        
                     // Right
        else if(right_IR > 1300 && left_IR < 1300){
            PWM_duty(PA_0,(float)0.2); // 50%
            PWM_duty(PA_1,(float)1);  // 80%      
                  display_auto = auto_right;
               }
                           // Left
        else if(right_IR < 1300 && left_IR > 1300){
            PWM_duty(PA_0,(float)1);  // 80%
            PWM_duty(PA_1,(float)0.2);   // 50%
                  display_auto = auto_left;
               }
            }
}

void Display_MState(void){
   if(auto_cnt > 1){
         USART_write(USART1, (uint8_t*)"MOD: M ", 7);
         if(direc == 1){
               USART_write(USART1, (uint8_t*)"DIR: F ", 7);
         }else if(direc == 0){
               USART_write(USART1, (uint8_t*)"DIR: B ", 7);
         }
         if(STR == 0){
                  USART_write(USART1, (uint8_t*)"STR: -3 ", 8);
                  USART_write(USART1, (uint8_t*)"VEL: V3", 7);
         }else if(STR == 1){
                  USART_write(USART1, (uint8_t*)"STR: -2 ", 8);
                  USART_write(USART1, (uint8_t*)"VEL: V2", 7);
         }else if(STR == 2){
                  USART_write(USART1, (uint8_t*)"STR: -1 ", 8);
                  USART_write(USART1, (uint8_t*)"VEL: V1", 7);
         }else if(STR == 3){
                  USART_write(USART1, (uint8_t*)"STR: 0 ", 7);
         }else if(STR == 4){
                  USART_write(USART1, (uint8_t*)"STR: 1 ", 7);
                  USART_write(USART1, (uint8_t*)"VEL: V1", 7);
         }else if(STR == 5){
                  USART_write(USART1, (uint8_t*)"STR: 2 ", 7);
                  USART_write(USART1, (uint8_t*)"VEL: V2", 7);
         }else if(STR == 6){
                  USART_write(USART1, (uint8_t*)"STR: 3 ", 7);
                  USART_write(USART1, (uint8_t*)"VEL: V3", 7);
         }
         if(speed == 0 && STR == 3){
               USART_write(USART1, (uint8_t*)"VEL: V0", 7);
         }else if(speed == 1 && STR == 3){
               USART_write(USART1, (uint8_t*)"VEL: V1", 7);
         }else if(speed == 2 && STR == 3){
               USART_write(USART1, (uint8_t*)"VEL: V2", 7);
         }else if(speed == 3 && STR == 3){
               USART_write(USART1, (uint8_t*)"VEL: V3", 7);
         }
         USART_write(USART1, "\r\n", 2);
         auto_cnt = 0;
  }
}

void Display_AState(void){ 

         if(cnt > 1){
         if(MODE==2){
            LED_toggle();
            
         }
            cnt=0;
      }  
   if(auto_cnt > 1){
       USART_write(USART1, (uint8_t*) "MODE : Line Tracer   ", 21);
      // STOP
      if(display_auto == auto_stop) USART_write(USART1, (uint8_t*) "STOP", 4);
      // UP
      else if(display_auto == auto_up) USART_write(USART1, (uint8_t*) "GO", 2);
      // RIGHT
      else if(display_auto == auto_right) USART_write(USART1, (uint8_t*)"TURN RIGHT", 10);
      // LEFT
      else if(display_auto == auto_left)USART_write(USART1, (uint8_t*) "TURN LEFT", 9);
      USART_write(USART1, "\r\n", 2);
      auto_cnt = 0;
  }
}


// Initialiization 
void MCU_init(void){
   RCC_PLL_init();
   SysTick_init();

  // BT serial init 
   UART1_init();   // PA9 - TXD , PA10 - RXD
   UART1_baud(BAUD_9600);
   UART2_init();   
   UART2_baud(BAUD_38400);
   
          // ADC Init
   ADC_init(PB_0);
   ADC_init(PB_1);
   
    // ADC channel sequence setting
   ADC_sequence(seqCHn, 2);
   
   
   GPIO_init(GPIOA, LED_PIN, OUTPUT);    // calls RCC_GPIOA_enable()
   GPIO_otype(GPIOA, LED_PIN, 0);        //  GPIOA LED_PIN Output Type: Push-Pull
   GPIO_ospeed(GPIOA, LED_PIN, EC_FAST);      // GPIOA Speed LED_PIN Fast
   
   // PWM of 1 msec:  TIM2_CH1 (PA_0 AFmode)
   PWM_init(PA_0);   
   GPIO_otype(GPIOA, 0, EC_PUSH_PULL);     //PWM PA_0 Push-Pull
   GPIO_pupd(GPIOA, 0, EC_NONE);           //PWM Pin No pull-up, pull-down
   GPIO_ospeed(GPIOA, 0, EC_FAST);         //PWM Pin Fast 
   PWM_period(PA_0, 1);
   
   // PWM of 1 msec:  TIM2_CH2 (PA_1 AFmode)
   PWM_init(PA_1);   
   GPIO_otype(GPIOA, 1, EC_PUSH_PULL);     //PWM Pin Push-Pull
   GPIO_pupd(GPIOA, 1, EC_NONE);           //PWM Pin No pull-up, pull-down
   GPIO_ospeed(GPIOA, 1, EC_FAST);         //PWM Pin Fast 
   PWM_period(PA_1, 1);
   
   //Moter Direction Pin   PC_2
   PWM_init(PC_2);   
   GPIO_init(GPIOC, 2, OUTPUT);
   
   
   //Moter Direction Pin  PC_3
   PWM_init(PC_3);
   GPIO_init(GPIOC, 3, OUTPUT); 
   
   // PWM:  TIM4_CH2 (PA_6 AFmode)
   PWM_init(TRIG);   
   GPIO_otype(GPIOA, 6, EC_PUSH_PULL);   //PWM Pin Push-Pull
   GPIO_pupd(GPIOA, 6, EC_NONE);         //PWM Pin NO Pull-up, Pull-down
   GPIO_ospeed(GPIOA, 6, EC_FAST);       //PWM Pin Fast 
   PWM_period_us(TRIG, 50000);        // 50 msec PWM period  
   PWM_pulsewidth_us(TRIG,10);        // 10 usec PWM pulse width
   
   // input Capture:  TIM4_CH1 (PB_6 AFmode)
   ICAP_init(ECHO);   
   GPIO_pupd(GPIOB, 6, EC_NONE);         //PWM Pin NO Pull-up, Pull-down
   ICAP_counter_us(PB_6, 10);            //Counter Clock : 0.1MHz (10us)
   ICAP_setup(PB_6, 1, IC_RISE);         //TI4 -> IC1 (rising edge)
   ICAP_setup(PB_6, 2, IC_FALL);         //TI4 -> IC2 (falling edge)   

}