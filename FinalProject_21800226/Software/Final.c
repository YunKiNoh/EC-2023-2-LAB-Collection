/*------------------------
/---------------------------------------------------------------/
- Class: 23-2 Embeded Contorller Final Project
- Team Students: EunChan Kim(21801017), YunKi Noh(21800226)
- Project name: Intelligent Fire Suppression Sprinkler
- Advisor: YungKeun Kim Professor
- Date: 23/12/19
/---------------------------------------------------------------/
---------------------------*/

#include "ecSTM32F411.h"

//----------------------------Fire Detection Value----------------------------//
//uint32_t Fire_Detection; // fire sensor value
uint32_t Fire_Signal = 0;  // signal for fire detection  


//----------------------------Servo Motor cnt----------------------------//
int cnt = 0;    // count value for upper servo motor's working
int cnt2 = 0;   // count value for under servo motor's working
int cnt_u = 0;  // count value for manual mode's under direction 
int cnt_r = 0;  // count value for manuala mode's right direction


//----------------------------Ultra Sonic Virable----------------------------//
uint32_t ovf_cnt = 0;
float distance = 0;
float timeInterval = 0;
float time1 = 0;
float time2 = 0;


//----------------------------Define Variable----------------------------//
#define PWM_PIN1 PA_1 // Upper servo
#define PWM_PIN2 PA_5 // Under servo

#define TRIG PA_6     // Ultra Sonic's Trig pin
#define ECHO PB_6     // Ultra Sonic's Echo pin

#define FIRE1 PB_0    // Fire Sensor's Analog pin 
#define FIRE2 PB_1    // Fire Sensor's Digital pin


//----------------------------IR Variable----------------------------//
static volatile uint32_t ADC_Analogue, ADC_Digital; // Analogue value & Digital value of Fire Sensor
int flag = 0;                                                          // flag for reading two IR Sensor's values
PinName_t seqCHn[2] = {PB_0, PB_1};                         // Array for two IR Sensor's value

static volatile uint8_t BT_Data = 0; // Bluetooth data
static volatile char mode;                 // Mode of Fire Sprinkler


//----------------------------Function Declaration----------------------------//
void setup(void);                            // Setting Function
void ADC_IRQHandler(void);                // Fire Sensor Function
void EXTI15_10_IRQHandler(void);       // Button Function
void ServoSetting();                         // Setting for Servo Motor Function
void TIM3_IRQHandler(void);             // Upper Servo Motor Function
void TIM2_IRQHandler(void);             // Under Servo Motor Function


int main(void) { 
   
   // Initialiization --------------------------------------------------------
   setup();

   // Inifinite Loop ----------------------------------------------------------
   while(1){
       
      /*-------------------Ultra Sonic Working-------------------*/
      distance = (float) timeInterval * 0.034 / 2.0;    // [mm] -> [cm]
       
       if(distance>=1 && distance<=6.5){
          
          if(distance>=6.0){
               GPIO_write(GPIOB,5,1);               
            }
          
            else if(distance < 6.0){
               GPIO_write(GPIOB,5,0);
            }
       }
 
       /*--------------------Printing Sector--------------------*/
       /*----------ADC----------*/
       printf("/*-----ADC value*-----/ \r\n");
       printf("Fire Analogue: %d \r\n", ADC_Analogue);
       printf("Fire Digital: %d \r\n", ADC_Digital);
       printf("\r\n");
       
       /*----------Mode----------*/
       printf("/*-----Current Mode-----*/ \r\n");
       if(mode == 'n' || mode == 'N') printf("Current Mode: Auto Mode \r\n");
       else if(mode == 'm' || mode == 'M') printf("Current Mode: Manual Mode \r\n");
       printf("\r\n");
       delay_ms(1000);
    }
}

// Initialiization 
void setup(void)
{   
      RCC_PLL_init(); // System Clock = 84MHz
      SysTick_init(); // SysTick Init
   
      // ADC Init
      ADC_init(PB_0); // Analogue Pin
      ADC_init(PB_1); // Digital Pin
      
      // ADC channel sequence setting
      ADC_sequence(seqCHn, 2);
   
      // BT serial init 
      UART1_init();
      UART1_baud(BAUD_9600);
   
      UART2_init();
      UART2_baud(BAUD_9600);
   
      USART_setting(USART1, GPIOA, 9, GPIOA, 10, BAUD_9600);
   
      // Servo setting
      ServoSetting();
    
      // Ultra Sonic PWM configuration ---------------------------------------------------------------------   
      PWM_init(TRIG);                    // PA_6: Ultrasonic trig pulse
      PWM_period_us(TRIG, 50000);     // PWM of 50ms period. Use period_us()
      PWM_pulsewidth_us(TRIG, 10);    // PWM pulse width of 10us
      // Input Capture conf iguration -----------------------------------------------------------------------   
      ICAP_init(ECHO);       // PB_6 as input caputre
      ICAP_counter_us(ECHO, 10);      // ICAP counter step time as 10us
      ICAP_setup(ECHO, 1, IC_RISE);   // TIM4_CH1 as IC1 , rising edge detect
      ICAP_setup(ECHO, 2, IC_FALL);   // TIM4_CH2 as IC2 , falling edge detect

      // Sending Signal from stm32 to Arduino by using Output pin(stm32) & Input pin(Arduino)
      GPIO_setting(GPIOB,5,OUTPUT,EC_PUSH_PULL,EC_PU,EC_MEDIUM);
      GPIO_write(GPIOB,5,LOW); //water
      GPIO_setting(GPIOB,3,OUTPUT,EC_PUSH_PULL,EC_PU,EC_MEDIUM);
      GPIO_write(GPIOB,3,LOW); // fire

      //Buzzer      
      GPIO_setting(GPIOA,8,OUTPUT,EC_PUSH_PULL,EC_PU,EC_MEDIUM);
      GPIO_write(GPIOA,8,LOW);
}

// ADC Interrupt
void ADC_IRQHandler(void){
   if(is_ADC_OVR())
      clear_ADC_OVR();
   
   if(is_ADC_EOC()){                // after finishing sequence
      if (flag==0)
         ADC_Analogue = ADC_read(); // upper_Analogue
      else if (flag==1)
         ADC_Digital = ADC_read();  // under_Digital
      
      if(ADC_Analogue > 2000){ // Fire not detected
      }
      else if(ADC_Analogue <= 2000){
         if(ADC_Digital < 1000){
               TIM_UI_disable(TIM2);
               TIM_UI_disable(TIM3);

      }
        else if(ADC_Digital >= 1500){ // Fire not detected
               TIM_UI_enable(TIM2);
               TIM_UI_enable(TIM3);
               GPIO_write(GPIOB,3, LOW); // Send LOW signal to Arduino
               GPIO_write(GPIOA,8,LOW);  // Turn off Buzzer
      }
      }
      
    if(ADC_Digital < 1000){ // Fire Detected
         TIM_UI_disable(TIM2);       // Pause Under Servo Motor
         TIM_UI_disable(TIM3);       // Pause Over Servo Motor
         GPIO_write(GPIOB,3, HIGH);  // Send HIGH signal to Arduino
         GPIO_write(GPIOA,8,HIGH);   // Turn on Buzzer
    }
    else { // Fire not Detected
         TIM_UI_enable(TIM2);           // Working Uner Servo Motor
         TIM_UI_enable(TIM3);           // Wokring Over Servo Motor
         GPIO_write(GPIOA,8,LOW);     // Send LOW signal to Arduino 
         GPIO_write(GPIOB,3, LOW);   // Turn off Buzzer
    }
      
      flag =! flag;      // flag toggle
   }
}

void EXTI15_10_IRQHandler(void) {  
   if (is_pending_EXTI(BUTTON_PIN)) {
        // Stop Servo Motor
        PWM_duty(PWM_PIN1, (float)0.025);
        PWM_duty(PWM_PIN2, (float)0.025);
        cnt = 0;
      clear_pending_EXTI(BUTTON_PIN); // cleared by writing '1'
       
   }
}

void ServoSetting(){
   
   // Servo pin setting
    GPIO_init(GPIOA, 1, EC_AF);
    GPIO_init(GPIOA, 5, EC_AF);
   
    // LED/BUTTON pin setting
    GPIO_init(GPIOC, BUTTON_PIN, INPUT);
   
    // Timer setting 
    TIM_init(TIM3, 500);            // TIMER period = 500ms
    TIM3->DIER |= 1;                // Update Interrupt Enabled
    NVIC_EnableIRQ(TIM3_IRQn);      // TIM3's interrupt request enabled   
    NVIC_SetPriority(TIM3_IRQn, 2); // set TIM3's interrupt priority as 2
   
    TIM_init(TIM2, 500);                   // TIMER period = 500ms
    TIM2->DIER |= 1;                        // Update Interrupt Enabled
    NVIC_EnableIRQ(TIM2_IRQn);      // TIM2's interrupt request enabled   
    NVIC_SetPriority(TIM2_IRQn, 3); // set TI2's interrupt priority as 3
    
    // Button Setting
    EXTI_init(GPIOC, 13, FALL, 0);       // set C_port's 13_pin as signal of EXTI
    NVIC_EnableIRQ(EXTI15_10_IRQn);      // enable request interrupt
    NVIC_SetPriority(EXTI15_10_IRQn, 3); // set priority
    
    // PWM setting
    PWM_init(PWM_PIN1);           // set PA1 PWM's output pin
    PWM_period(PWM_PIN1, 20);  // 20 msec PWM period 
   
    PWM_init(PWM_PIN2);           // set PA5 as PWM's output pin
    PWM_period(PWM_PIN2, 20);  // 20 msec PWM period 
}

/*-------------Over Servo Motor-------------*/
void TIM3_IRQHandler(void){
   if((TIM3->SR & TIM_SR_UIF) == 1){ 
      /*----0.5ms = 0dgree, 1.5ms = 90degree, 2.5ms = 180degree----*/
         if(mode == 'N'){
      
      static int direction1 = 1;// Add a direction variable to control the movement
                
      PWM_duty(PWM_PIN1, (float)0.025*(cnt*4/100.0 + 1)); // Working
                
      cnt = cnt + direction1; // coutning
                
      if(cnt >= 100.0) direction1 = -1;  // Rotate to the left when reaching 180 degrees
      else if(cnt <= 0) direction1 = 1;  // Rotate to the right when reaching 0 degrees
         }
      // clear by writing 0
      TIM3->SR &= ~ TIM_SR_UIF;                    
   }
}

/*-------------Under Servo Motor-------------*/
void TIM2_IRQHandler(void){
   if((TIM2->SR & TIM_SR_UIF) == 1){ 
       /*----0.5ms = 0dgree, 1.5ms = 90degree, 2.5ms = 180degree----*/
       if(mode == 'N'){

        static int direction2 = 1; // Add a direction variable to control the movement
      
        PWM_duty(PWM_PIN2, (float)0.05*(cnt2*0.5/30 + 1)); // 0.06
        cnt2 = cnt2 + direction2; // counting
         
        if(cnt2 >= 30) direction2 = -1;  // Rotate to the left when reaching 180 degrees
        else if(cnt2 <= 0) direction2 = 1;  // Rotate to the right when reaching 0 degrees
       }

      // clear by writing 0
      TIM2->SR &= ~ TIM_SR_UIF;    
    }
}



// Ultra Sonic
void TIM4_IRQHandler(void){
   if(is_UIF(TIM4)){                              // Update interrupt
      ovf_cnt++;                                  // overflow count
      clear_UIF(TIM4);                            // clear update interrupt flag
   }
   if(is_CCIF(TIM4, 1)){                          // TIM4_Ch1 (IC1) Capture Flag. Rising Edge Detect
      time1 = ICAP_capture(TIM4, IC_1);                           // Capture TimeStart
      clear_CCIF(TIM4, 1);                        // clear capture/compare interrupt flag 
   }                                               
   else if(is_CCIF(TIM4, 2)){                     // TIM4_Ch2 (IC2) Capture Flag. Falling Edge Detect
      time2 = ICAP_capture(TIM4, IC_2);           // Capture TimeEnd
      timeInterval = 10 * ((ovf_cnt * (TIM4->ARR+1)) + (time2 - time1));    // (10us * counter pulse -> [msec] unit) Total time of echo pulse
      ovf_cnt = 0;                                     // overflow reset
      clear_CCIF(TIM4,2);                         // clear capture/compare interrupt flag 
   }
}

void USART1_IRQHandler(){                // USART2 RX Interrupt : Recommended
   if(is_USART1_RXNE()){
    BT_Data = USART1_read();      // RX from UART1 (BT)
      USART1_write(&BT_Data,1);
      if(BT_Data=='m'||BT_Data=='M'){
         GPIO_write(GPIOA,5, 1);
         mode='M';
            TIM_UI_disable(TIM3);
            TIM_UI_disable(TIM2);
      }
         else if(BT_Data=='n'||BT_Data=='N'){
            mode='N';
            TIM_UI_enable(TIM3);
            TIM_UI_enable(TIM2);
         }

      else if(BT_Data=='S'||BT_Data=='s'){
         cnt_u++;
         //if(cnt_u > 50) cnt_u = 50; // Limit the maximum value to prevent exceeding the servo range
         PWM_duty(PWM_PIN2, (float)0.025*(cnt_u*1/50.0 + 1));
      } 
      else if(BT_Data=='W'||BT_Data=='w'){
         cnt_u--;
         //if(cnt_u < 0) cnt_u = 0; // Limit the minimum value to prevent exceeding the servo range
         PWM_duty(PWM_PIN2, (float)0.025*(cnt_u*1/50.0 + 1));
      }
      else if(BT_Data=='D'||BT_Data=='d'){
             cnt_r++;
         //if(cnt_r > 50) cnt_r = 50; // Limit the maximum value to prevent exceeding the servo range
         PWM_duty(PWM_PIN1, (float)0.025*(cnt_r*1/50.0 + 1));
      }
      else if(BT_Data=='A'||BT_Data=='a'){
         cnt_r--;
         //if(cnt_r < 0) cnt_r = 0; // Limit the minimum value to prevent exceeding the servo range
         PWM_duty(PWM_PIN1, (float)0.025*(cnt_r*1/50.0 + 1));
      }
   }
}