# 23-2 Embeded Controller Files Collection

본 레포지토리는 2023년도 2학기에 수강한 Embeded Controller(EC)의 학업 결과를 정리하기 위하여 작성되었습니다. 본 강의는 MCU(Micro Controller Unit)에 대하여 배우며, Arduino 보드로 MCU에 대한 기초를 익히고 곧바로 stm 보드로 실험 과제를 수행해 갑니다. 총 9개의 실험과 1개의 파이널 팀 프로젝트를 진행하였으며, 팀 프로젝트에서는 지난 과제들을 바탕으로 직접 프로젝트 주제를 선정하였습니다. 각각의 LAB과 프로젝트 대한 정보(보고서, 소스코드)는 상단 폴더에 정리하였으며, 본 메인문에서는 본 실험들에 쓰인 라이브러리 코드들에 대한 핵심 정보를 정리 및 기제하였습니다.

## LAB Environment
- **IDE/Compiler**: Keil uVision 5
- **System**: Window 10/11
- **MCU Board**: STM32F411RE, Nucleo-64

## LAB Information
- **LAB1**: trigger signal을 통해 mini fan의 DC motor 구동
- **LAB2**: trigger signal을 통해 led 점등 및 소등
- **LAB3**: 7-segment decoder를 통해 0부터 9까지의 숫자 표현
- **LAB4**: timer를 통해 trigger signal을 생성하여 rc servo motor 구동
- **LAB5**: systick 타이머를 통해 단위시간마다 숫자를 0부터 9까지 변환
- **LAB6**: pwm 신호를 통해 초음파 센서로부터 물체와의 거리 값 계산
- **LAB7**: pwm 신호를 통해 stepper motor 구동
- **LAB8**: USART 통신
- **LAB9**: DC morot, IR sensor, UltarSonic sensor를 통하여 자동차가 선 따라서 자율주행하도록 구현
- **Final Team Project**: 초동 화재 진압 시스템 구축
- **Lib**: 라이브러리 폴더로써, 총 20개의 개별 라이브러리 코드 파일(ecSTM32F411.h 이외의 20개 파일)과 1개의 선언 파일(ecSTM32F411.h)이 있음

# EC_Lib_Documentation

## ecGPIO.c

### void GPIO_mode()

**Function:** Set pin's mode

~~~
void GPIO_mode(GPIO_TypeDef *Port, int pin, int mode);
~~~

**Parameters:**

- `GPIO_TypeDef *Port`: Port information
- `int pin`: Pin information
- `int mode`: mode of pin
  - Input: 00 / Output: 01 / AlterFunc: 10 / Analog: 11

**Example:** LED

~~~ 
GPIO_mode(GPIOA, 5, OUTPUT);
~~~

### void GPIO_ospeed()

**Function:** Set response speed of pin

~~~ 
void GPIO_ospeed(GPIO_TypeDef *Port, int pin, int speed);
~~~

**Parameters:**

- `GPIO_TypeDef *Port`: Port information
- `int pin`: Pin information
- `int speed`: Set response speed
  - Low: 00 / Medium: 01 / Fast: 10 / High: 11

**Example:** Button

~~~
GPIO_ospeed(GPIOC, 13, High);
~~~

### void GPIO_otype();

**Function:** Set type of output

~~~
void GPIO_otype(GPIO_TypeDef *Port, int pin, int type);
~~~

**Parameters:** 

- `GPIO_TypeDef *Port`: Port information
- `int pin`: Pin information
- `int type`: type of output
  - push-pull: 0 / open drain: 1

**Example:** LED

~~~
GPIO_otype(GPIOA, 5, push-pull);
~~~

### void GPIO_pupd();

**Function:** Set pull-up/pull-down configuration of a pin

~~~
void GPIO_pupd(GPIO_TypeDef *Port, int pin, int pupd)
~~~

**Parameters:**

- `GPIO_TypeDef *Port`: Port information
- `int pin`: Pin information
- `int type`: Pull-up/pull-down configuration
  - Pull-up: 0 / Pull-down: 1 / No pull-up No pull-down: 2

**Example:** LED

~~~
GPIO_pupd(GPIOB, 3, pull-up);
~~~

### void GPIO_read();

**Function:** Read the value of a GPIO pin

~~~
int GPIO_read(GPIO_TypeDef *Port, int pin);
~~~

**Parameters:**

- **GPIO_TypeDef *Port**: Port information
- `int pin`: Pin information
- `int type`: Pull-up/pull-down configuration
  - Pull-up: 0 / Pull-down: 1 / No pull-up No pull-down: 2

**Return:**

- `int`: The value of the specified pin (0 or 1)

**Example:** LED

~~~
int buttonState = GPIO_read(GPIOC, 2);
~~~

### void GPIO_write();

**Function:** Write a value to a GPIO pin

~~~
void GPIO_write(GPIO_TypeDef *Port, int pin, int Output);
~~~

**Parameters:**

- `GPIO_TypeDef *Port`: Port information
- `int pin`: Pin information
- `int Output`: Output value to be written (0 or 1)

**Example:** Turn on an LED

```
GPIO_write(GPIOD, 5, 1);
```

### void GPIO_setting();

**Function:** Configure various settings for a GPIO pin

~~~
void GPIO_setting(GPIO_TypeDef *Port, int pin, int mode, int otype, int pupd, int ospeed);
~~~

**Parameters:**

- `GPIO_TypeDef *Port`: Port information
- `int pin`: Pin information
- `int mode`: Mode of the pin
  - Input: 00 / Output: 01 / AlterFunc: 10 / Analog: 11
- `int otype`: Output type of the pin
  - Push-pull: 0 / Open drain: 1
- `int pupd`: Pull-up/pull-down configuration
  - Pull-up: 0 / Pull-down: 1 / No pull-up No pull-down: 2
- `int ospeed`: Output speed configuration
  - Low: 00 / Medium: 01 / High: 10 / Very high: 11

**Example:**

```
GPIO_setting(GPIOA, 5, output, push-pull, pull-up, high-speed)
```

### void LED_toggle();

**Function:** Toggle the state of an LED(PA5)

~~~
void LED_toggle(void);
~~~

**Example:**

```
LED_toggle();
```

### void sevensegment_init()

**Function:** Initialize GPIO pins setting for software decoder

~~~
void sevensegment_init(void);
~~~

**More Detail:** In the report 'LAB GPIO Digital InOut 7-segment' for 'software algorithm & circuit'

### void sevensegment_decoder()

**Function:** 7-segment software decoder  

~~~
void sevensegment_decoder(uint8_t  num);
~~~

**Parameters:**

- `uint8_t  num`: number to display on the 7-segment

**Example:** display '3' using software decoder

~~~
sevensegment_decoder(3);
~~~

**More Detail:** In the report 'LAB GPIO Digital InOut 7-segment' for 'software algorithm & circuit'

### void sevensegment_display_init()

**Function:** Initialize GPIO pins setting for hardware decoder

~~~
void sevensegment_display_init(void);
~~~

**Example:**

~~~
sevensegment_display_init();
~~~

**More Detail:** In the report 'LAB GPIO Digital InOut 7-segment' for 'software algorithm & circuit'

### void sevensegment_display()

**Function:** 7-segment hardware decoder

~~~
void sevensegment_display(uint8_t  num);
~~~

**Parameters:**

- `uint8_t  num`: number to display on the 7-segment

**Example:** display '7' hardware decoder

~~~
sevensegment_display(7);
~~~

**More Detail:** In the report 'LAB GPIO Digital InOut 7-segment' for 'software algorithm & circuit'

## ecRCC.c

### void RCC_HSI_init()

**Function:** Initial setting for HSI Clock

~~~
void RCC_HSI_init();
~~~

**Example:** 16MhHz

~~~
RCC_HSI_init();
~~~

### void RCC_PLL_init()

**Function:** Initial setting for HSI Clock

~~~
void RCC_PLL_init();
~~~

**Example:** 84MhHz

~~~
RCC_PLL_init();
~~~

## ecEXTI.c

### void EXTI_init()

**Function:** Initial setting for EXTI(External Interrupt)

~~~
void EXTI_init(GPIO_TypeDef *Port, int Pin, int trig_type,int priority);
~~~

**Parameters:** 

- `GPIO_TypeDef *Port`: Port
- `int Pin`: Pin
- `int trig_type`: Trigger type
  -  Falling trigger: 0 / Rising trigger: 1 / Both: 2
- `int priority`: IRQHandler's working priority

**Example:** Sensing Button when Button is pressed not taken off

~~~
EXTI_init(GPIOC, 13, FALL, 0);
~~~

### void EXTI_enable()

**Function:** Enable External Interrupt for the specified pin

```
void EXTI_enable(uint32_t pin);
```

**Parameters:**

- `uint32_t pin`: Pin to enable external interrupt

**Example:** Enable External Interrupt for Pin 13

```
EXTI_enable(13);
```

### void EXTI_disable()

**Function:** Disable External Interrupt for the specified pin

```
void EXTI_disable(uint32_t pin);
```

**Parameters:**

- `uint32_t pin`: Pin to disable external interrupt

**Example:** Disable External Interrupt for Pin 13

```
EXTI_disable(13);
```

### uint32_t is_pending_EXTI()

**Function:** Check if External Interrupt for the specified pin is pending(pending means this External Interrupt is ready to work)

```
cCopy code
uint32_t is_pending_EXTI(uint32_t pin);
```

**Parameters:**

- `uint32_t pin`: Pin to check for pending external interrupt

**Example:** Check if External Interrupt for Pin 13 is pending

```
if (is_pending_EXTI(BUTTON_PIN)) {
    toggle_LED();
}
```

### void clear_pending_EXTI()

**Function:** Clear pending status for External Interrupt on the specified pin

```
void clear_pending_EXTI(uint32_t pin);
```

**Parameters:**

- `uint32_t pin`: Pin for which to clear the pending status

**Example:** Clear pending status for External Interrupt on Pin 13

```
clear_pending_EXTI(BUTTON_PIN);
```

## ecSystick.c

### void SysTick_init()

**Function:** Set basic interrupt period of system

```
void SysTick_init(void);
```

**Example:**

```
RCC_PLL_init(); 
SysTick_init();
```

**More detail:**

- `#define MCU_CLK_PLL 84000000` // 84MHz
  - When select PLL clock, basic interrupt period becomes 1ms
- `#define MCU_CLK_HSI 16000000` // 16MHz
  - When select HSI clock, basic interrupt period becomes 0.5ms

### void SysTick_Handler()

**Function:** Work Systick_counter()

~~~
void SysTick_Handler(void){
	SysTick_counter();	
}
~~~

**Example:**

~~~
SysTick_Handler();
~~~

### SysTick_counter()

**Function:** Counting time per interrupt period

~~~
void SysTick_counter(){
	msTicks++;
}	
~~~

**Example:**

~~~
SysTick_counter();
~~~

### void delay_ms()

**Function:** Delay Execution for a Specified Duration

```
void delay_ms(uint32_t msec);
```

**Parameters:**

- `uint32_t msec`: Duration of the delay in milliseconds

**Example:** Delay execution for 500 milliseconds:

```
RCC_PLL_init(); 
SysTick_init();
delay_ms(500);
```

### void SysTick_reset()

**Function:** Reset the SysTick Timer Value

```
void SysTick_reset(void)
{
	// Reset the SysTick Current Value Register
	SysTick->VAL = 0;
}
```

**Example:**

```
SysTick_reset();
```

### void SysTick_val()

**Function:** Get the Current Value of SysTick Timer

```
uint32_t SysTick_val(void) {
	return SysTick->VAL;
}
```

**Example:**

```
uint32_t currentValue = SysTick_val();
```

**Mote detail:** PDF "Programming-manual", 249 pg.[Link]

## ecTIMc

### void TIM_init()

**Function:** Initialize Timer for a Specified Duration

```
void TIM_init(TIM_TypeDef* TIMx, uint32_t msec);
```

**Parameters:**

- `TIM_TypeDef* TIMx`: Timer instance (e.g., TIM1, TIM2, etc.)
- `uint32_t msec`: Interrupt period of the timer in milliseconds

**Example:**

```
TIM_init(TIM1, 1000); 
```

**More detail:** including

`Tim_enable(TIMx);` 

`TIM_period_ms(TIMx, msec);`

`TIMx->CR1 &= (1<<4);`

`TIMx->CR1 |= TIM_CR1_CEN;`

### void TIM_period_us()

**Function:** Set Timer Period for Microseconds

```
void TIM_period_us(TIM_TypeDef *TIMx, uint32_t usec);
```

**Parameters:**

- `TIM_TypeDef *TIMx`: Timer instance (e.g., TIM2, TIM5, etc.)
- `uint32_t usec`: Desired timer period in microseconds (1 to 1000 microseconds)

**Example:**

```
TIM_period_us(TIM2, 500); // Set TIM2 period to 500 microseconds
```

### void TIM_period_ms()

**Function:** Set Timer Period for Milliseconds

```
void TIM_period_ms(TIM_TypeDef* TIMx, uint32_t msec);
```

**Parameters:**

- `TIM_TypeDef* TIMx`: Timer instance (e.g., TIM2, TIM5, etc.)
- `uint32_t msec`: Desired timer period in milliseconds (1 to 6000 milliseconds)

### void TIM_UI_init()

**Function:** Initialize Timer for Update Event Interrupt

```
void TIM_UI_init(TIM_TypeDef* TIMx, uint32_t msec);
```

**Parameters:**

- `TIM_TypeDef* TIMx`: Timer instance (e.g., TIM1, TIM2, etc.)
- `uint32_t msec`: Desired timer period in milliseconds for initialization

**Example:** Initialize TIM2 for update event interrupt with a period of 1000 milliseconds

```
TIM_UI_init(TIM2, 1000); 
```

​	**More detail:** including

​	`TIM_init(TIMx,msec);`

​	`TIM_UI_enable(TIMx);`

​	`NVIC_EnableIRQ(IRQn_reg); `: Enable IRQ

​	`NVIC_SetPriority(IRQn_reg,2);` : Set IRQ Priority

### void TIM_UI_enable()

**Function:** Enable Timer Update Interrupt

~~~
void TIM_UI_enable(TIM_TypeDef* TIMx);
~~~

**Parameters:**

- `TIM_TypeDef* TIMx`: Timer instance (e.g., TIM1, TIM2, etc.)

**Example:** Enable update interrupt for TIM2

```
TIM_UI_enable(TIM2); 
```

### void TIM_UI_disable()

**Function:** Disable Timer Update Interrupt

```
cCopy code
void TIM_UI_disable(TIM_TypeDef* TIMx);
```

**Parameters:**

- `TIM_TypeDef* TIMx`: Timer instance (e.g., TIM1, TIM2, etc.)

**Example:** Disable update interrupt for TIM2

```
TIM_UI_disable(TIM2); 

```

### uint32_t is_UIF()

**Function:** Check Timer Update Interrupt Flag

```
uint32_t is_UIF(TIM_TypeDef *TIMx);
```

**Parameters:**

- `TIM_TypeDef *TIMx`: Timer instance (e.g., TIM1, TIM2, etc.)

**Return:**

- `uint32_t`: Returns `1` if the UIF is set, indicating that a timer update event has occurred. Returns `0` otherwise.

**Example:** Ultra Sonic Distance Value Calculation

```
void TIM4_IRQHandler(void){
   if(is_UIF(TIM4)){                     			   // Update interrupt
      ovf_cnt++;                                       // overflow count
      clear_UIF(TIM4);                         		   // clear update interrupt flag
   }
   if(is_CCIF(TIM4, 1)){                         	  // TIM4_Ch1 (IC1) Capture Flag. Rising Edge Detect
		  time1 = ICAP_capture(TIM4,IC_1);            // Capture TimeStart
		  clear_CCIF(TIM4, 1);                	      // clear capture/compare interrupt flag 
   }                                              
   else if(is_CCIF(TIM4, 2)){                  	      // TIM4_Ch2 (IC2) Capture Flag. Falling Edge Detect
		  time2 = ICAP_capture(TIM4,IC_2);            // Capture TimeEnd
      timeInterval = 10*((time2 - time1)+(TIM3->ARR+1)*ovf_cnt);                
      distance = (float) timeInterval/58.0;    
      ovf_cnt = 0;                       
      clear_CCIF(TIM4,2);                          	  // clear capture/compare interrupt flag 
   }
}
```

### void clear_UIF()

**Function:** Clear Timer Update Interrupt Flag

```
void clear_UIF(TIM_TypeDef *TIMx);
```

**Parameters:**

- `TIM_TypeDef *TIMx`: Timer instance (e.g., TIM1, TIM2, etc.)

**Example:** Clear timer update interrupt flag for TIM2

```
clear_UIF(TIM2);
```

### void Tim_enable()

**Function:** Enable Timer Clock

```
void Tim_enable(TIM_TypeDef* TIMx);
```

**Parameters:**

- `TIM_TypeDef* TIMx`: Timer instance (e.g., TIM1, TIM2, etc.)

**Example: ** Enable clock for TIM2

```
Tim_enable(TIM2);
```

### void ICAP_init()

**Function:** Initialize Input Capture (ICAP)

```
void ICAP_init(PinName_t pinName);
```

**Parameters:**

- `PinName_t pinName`: Pin name for Input Capture(e.g., PA1, PA5, PC13, etc.)

**Example:** Initialize Input Capture using pin PA1

```
ICAP_init(PA1); 
```

### void ICAP_setup()

**Function:** Set Input Capture (ICAP) Edge Type

```
void ICAP_setup(PinName_t pinName, int ICn, int edge_type);
```

**Parameters:**

- `PinName_t pinName`: Pin name for Input Capture
- `int ICn`: Input Capture channel number (1 to 4)
- `int edge_type`: Edge type 
  - Rising: 00 / Falling: 01 / Both: 11

**Example:** Set PA1 as falling edge for IC1

```
ICAP_setup(PA1, 1, IC_FALL); 
```

**More detail:** ICAP Edge Type is the timing to detect signal

### void ICAP_counter_us()

**Function:** Set Input Capture (ICAP) Counter Step Value

```
void ICAP_counter_us(PinName_t pinName, int usec);
```

**Parameters:**

- `PinName_t pinName`: Pin name for Input Capture
- `int usec`: Microseconds for the counter step value

**Example:**  Set counter step value for PA1 to 50 microseconds

```
ICAP_counter_us(PA1, 50);
```

### uint32_t is_CCIF()

**Function:** Check Capture/Compare Interrupt Flag

```
uint32_t is_CCIF(TIM_TypeDef *TIMx, uint32_t ccNum);
```

**Parameters:**

- `TIM_TypeDef *TIMx`: Timer instance (e.g., TIM1, TIM2, etc.)
- `uint32_t ccNum`: Capture/Compare channel number (1 to 4)

**Example:** Ultra Sonic Distance Value Calculation

```
void TIM4_IRQHandler(void){
   if(is_UIF(TIM4)){                     			   // Update interrupt
      ovf_cnt++;                                       // overflow count
      clear_UIF(TIM4);                         		   // clear update interrupt flag
   }
   if(is_CCIF(TIM4, 1)){                         	  // TIM4_Ch1 (IC1) Capture Flag. Rising Edge Detect
		  time1 = ICAP_capture(TIM4,IC_1);            // Capture TimeStart
		  clear_CCIF(TIM4, 1);                	      // clear capture/compare interrupt flag 
   }                                              
   else if(is_CCIF(TIM4, 2)){                  	      // TIM4_Ch2 (IC2) Capture Flag. Falling Edge Detect
		  time2 = ICAP_capture(TIM4,IC_2);            // Capture TimeEnd
      timeInterval = 10*((time2 - time1)+(TIM3->ARR+1)*ovf_cnt);                
      distance = (float) timeInterval/58.0;    
      ovf_cnt = 0;                       
      clear_CCIF(TIM4,2);                          	  // clear capture/compare interrupt flag 
   }
}
```

### void clear_CCIF()

**Function:** Clear Capture/Compare Interrupt Flag

```
void clear_CCIF(TIM_TypeDef *TIMx, uint32_t ccNum);
```

**Parameters:**

- `TIM_TypeDef *TIMx`: Timer instance (e.g., TIM1, TIM2, etc.)
- `uint32_t ccNum`: Capture/Compare channel number (1 to 4)

**Example:** Ultra Sonic Distance Value Calculation

```
void TIM4_IRQHandler(void){
   if(is_UIF(TIM4)){                     			   // Update interrupt
      ovf_cnt++;                                       // overflow count
      clear_UIF(TIM4);                         		   // clear update interrupt flag
   }
   if(is_CCIF(TIM4, 1)){                         	  // TIM4_Ch1 (IC1) Capture Flag. Rising Edge Detect
		  time1 = ICAP_capture(TIM4,IC_1);            // Capture TimeStart
		  clear_CCIF(TIM4, 1);                	      // clear capture/compare interrupt flag 
   }                                              
   else if(is_CCIF(TIM4, 2)){                  	      // TIM4_Ch2 (IC2) Capture Flag. Falling Edge Detect
		  time2 = ICAP_capture(TIM4,IC_2);            // Capture TimeEnd
      timeInterval = 10*((time2 - time1)+(TIM3->ARR+1)*ovf_cnt);                
      distance = (float) timeInterval/58.0;    
      ovf_cnt = 0;                       
      clear_CCIF(TIM4,2);                          	  // clear capture/compare interrupt flag 
   }
}
```

###  uint32_t ICAP_capture()

**Function:** Get Input Capture (ICAP) Capture Value

```
uint32_t ICAP_capture(TIM_TypeDef* TIMx, uint32_t ICn);
```

**Parameters:**

- `TIM_TypeDef *TIMx`: Timer instance (e.g., TIM1, TIM2, etc.)
- `uint32_t ICn`: Input Capture channel number (1 to 4)

**Returns:**

- `uint32_t`: Capture value for the specified Input Capture channel (`ICn`).

**Example:** Ultra Sonic Distance Value Calculation

```
void TIM4_IRQHandler(void){
   if(is_UIF(TIM4)){                     			   // Update interrupt
      ovf_cnt++;                                       // overflow count
      clear_UIF(TIM4);                         		   // clear update interrupt flag
   }
   if(is_CCIF(TIM4, 1)){                         	  // TIM4_Ch1 (IC1) Capture Flag. Rising Edge Detect
		  time1 = ICAP_capture(TIM4,IC_1);            // Capture TimeStart
		  clear_CCIF(TIM4, 1);                	      // clear capture/compare interrupt flag 
   }                                              
   else if(is_CCIF(TIM4, 2)){                  	      // TIM4_Ch2 (IC2) Capture Flag. Falling Edge Detect
		  time2 = ICAP_capture(TIM4,IC_2);            // Capture TimeEnd
      timeInterval = 10*((time2 - time1)+(TIM3->ARR+1)*ovf_cnt);                
      distance = (float) timeInterval/58.0;    
      ovf_cnt = 0;                       
      clear_CCIF(TIM4,2);                          	  // clear capture/compare interrupt flag 
   }
}
```

###  void ICAP_pinmap()

**Function:** Input Capture (ICAP) Pin Mapping

```
void ICAP_pinmap(PinName_t pinName, TIM_TypeDef **TIMx, int *chN);
```

**Parameters:**

- `PinName_t pinName`: Pin name or number to be mapped (enum type or integer).
- `TIM_TypeDef **TIMx`: Pointer to the timer instance (e.g., TIM1, TIM2, etc.) to be populated based on pin mapping.
- `int *chN`: Pointer to the integer to be populated with the Input Capture channel number based on pin mapping.

**Example:** Mapping pinName, TIMx and Channel  

```
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
```

## ecPWM.c

### void PWM_init()

**Function:** PWM Initialization

```
void PWM_init(PinName_t pinName);
```

**Parameters:**

- `PinName_t pinName`: Pin name to initialize PWM (enum type or integer, e.g., PA_0, PC_3, etc.).

**Example: **Initialize PWM on pin PA_0

```
PWM_init(PA_0); 
```

**More detail:** including

`GPIO_setting`

`TIM_init `

etc.

### void PWM_period_ms()

**Function:** PWM Period Setup in microseconds

```
void PWM_period_ms(PinName_t pinName, uint32_t msec);
```

**Parameters:**

- `PinName_t pinName`: Pin name or number associated with the PWM channel (enum type or integer, e.g., PA_0, PC_3, etc.).
- `uint32_t msec`: Desired PWM period in milliseconds.

**Example:**  Set PWM period to 100 milliseconds on pin PA_0

```
PWM_period_ms(PA_0, 100); 
```

**More detail:** 'PWM_period' means 'interrupt period of PWM'

### void PWM_period()

**Function:** PWM Period Setup in milliseconds

```
void PWM_period(PinName_t pinName, uint32_t msec);
```

**Parameters:**

- `PinName_t pinName`: Pin name or number associated with the PWM channel (enum type or integer).
- `uint32_t msec`: Desired PWM period in milliseconds.

**Example:**  Set PWM period to 100 milliseconds on pin PA_0

~~~
PWM_period(PA_0, 100); 
~~~

### void PWM_period_us()

**Function:** PWM Period Setup in Microseconds

```
void PWM_period_us(PinName_t pinName, uint32_t usec);
```

**Parameters:**

- `PinName_t pinName`: Pin name or number associated with the PWM channel (enum type or integer).
- `uint32_t usec`: Desired PWM period in microseconds.

**Example: **Set PWM period to 500 microseconds on pin PA_0

```
PWM_period_us(PA_0, 500); 
```

### void PWM_pulsewidth()

**Function:** PWM Pulse Width Setup in Milliseconds

```
void PWM_pulsewidth(PinName_t pinName, uint32_t pulse_width_ms);
```

**Parameters:**

- `PinName_t pinName`: Pin name or number associated with the PWM channel (enum type or integer).
- `uint32_t pulse_width_ms`: Desired high pulse width in milliseconds.

**Example:**  Set PWM high pulse width to 5 milliseconds on pin PA_0

```
PWM_pulsewidth(PA_0, 5); 
```

**More detail:** 'PWM_pulsewidth' means 'period of pulse which is consisting of each interrupt'

### void PWM_pulsewidth_ms()

**Function:** PWM Pulse Width Setup in Milliseconds

```
void PWM_pulsewidth_ms(PinName_t pinName, uint32_t pulse_width_ms);
```

**Parameters:**

- `PinName_t pinName`: Pin name or number associated with the PWM channel (enum type or integer).
- `uint32_t pulse_width_ms`: Desired high pulse width in milliseconds.

**Example:**  Set PWM high pulse width to 5 milliseconds on pin PA_0

```
PWM_pulsewidth_ms(PA_0, 5); 
```

### void PWM_pulsewidth_us()

**Function:** PWM Pulse Width Setup in Microseconds

```
void PWM_pulsewidth_us(PinName_t pinName, uint32_t pulse_width_us);
```

**Parameters:**

- `PinName_t pinName`: Pin name or number associated with the PWM channel (enum type or integer).
- `uint32_t pulse_width_ms`: Desired high pulse width in milliseconds.

**Example:**  Set PWM high pulse width to 5 Microseconds on pin PA_0

```
PWM_pulsewidth_us(PA_0, 5); 
```

### void PWM_duty()

**Function:** Set Duty Cycle for PWM

```
void PWM_duty(PinName_t pinName, float duty);
```

**Parameters:**

- `PinName_t pinName`: Pin name or number associated with the PWM channel (enum type or integer).
- `float duty`: Duty cycle value ranging from 0 to 1.

**Example:** Set the duty cycle to 0.75 for pin PA_1.

```
PWM_duty(PA_1, 0.75);
```

**More detail:** PWM Duty cycle shows different impacts on speed of DC Motor according to the types of DC motor driver connection (e.g. 0.4 duty --> 60% speed or reversely).

### void PWM_pinmap()

**Function:** Map Pin to PWM Timer and Channel

```
void PWM_pinmap(PinName_t pinName, TIM_TypeDef **TIMx, int *chN);
```

**Parameters:**

- `PinName_t pinName`: Pin name or number associated with the PWM channel (enum type or integer).
- `TIM_TypeDef **TIMx`: Pointer to a pointer to the TIM_TypeDef structure representing the PWM timer.
- `int *chN`: Pointer to an integer representing the PWM channel number.

**Example:** void PWM_duty();

~~~
void PWM_duty(PinName_t pinName, float duty){ 
   
// 0. Match TIMx from  Port and Pin    
   GPIO_TypeDef *port;
   unsigned int pin;   
   ecPinmap(pinName, &port, &pin);   
   TIM_TypeDef *TIMx;
   int chN;      
   PWM_pinmap(pinName, &TIMx, &chN);

   
// value = CCR
   float value = (TIMx->ARR + 1)*duty - 1;                          // (ARR+1)*dutyRatio + 1               
  
   switch(chN){
      case 1: TIMx->CCR1 = value; break;
      case 2: TIMx->CCR2 = value; break;
      case 3: TIMx->CCR3 = value; break;
      case 4: TIMx->CCR4 = value; break;
      default: break;
   }
}
~~~

## ecStepper

### #define

~~~
//State number 
#define S0 0
#define S1 1
#define S2 2
#define S3 3
//for half
#define S4 4
#define S5 5
#define S6 6
#define S7 7
~~~

### FSM_full_structure

**Function:** To save output value and next state value

~~~
typedef struct {
   uint8_t out;
     uint32_t next[2];
} State_full_t;
~~~

### FSM_full

**Function:** To work stepper motor using Moore FSM 

~~~
State_full_t FSM_full[4] = {  
    {0b1010,{S3,S1}}, 
    {0b0110,{S0,S2}},
    {0b0101,{S1,S3}},
    {0b1001,{S2,S0}}
};
~~~

### FSM_half_structure

**Function:** Make memory to save output value and next state value

~~~
typedef struct {
   uint8_t out;
     uint32_t next[2];
} State_half_t;
~~~

### FSM_half

**Function:** To work stepper motor using Moore FSM 

~~~
State_half_t FSM_half[8] = { 
   {0b1000,{S7,S1}},
   {0b1010,{S0,S2}},
   {0b0010,{S1,S3}},
   {0b0110,{S2,S4}},
   {0b0100,{S3,S5}},
   {0b0101,{S4,S6}},
   {0b0001,{S5,S7}},
   {0b1001,{S6,S0}},
};
~~~

**More detail:** In Report, "LAB_Stepper_report_21800226_NohYunKi"

### void Stepper_init()

**Function:** Stepper Motor Initialization

```
void Stepper_init(GPIO_TypeDef* port1, int pin1, GPIO_TypeDef* port2, int pin2, GPIO_TypeDef* port3, int pin3, GPIO_TypeDef* port4, int pin4);
```

**Parameters:**

- `GPIO_TypeDef* port1`: GPIO port for stepper motor pin 1.
- `int pin1`: Pin number for stepper motor pin 1.
- `GPIO_TypeDef* port2`: GPIO port for stepper motor pin 2.
- `int pin2`: Pin number for stepper motor pin 2.
- `GPIO_TypeDef* port3`: GPIO port for stepper motor pin 3.
- `int pin3`: Pin number for stepper motor pin 3.
- `GPIO_TypeDef* port4`: GPIO port for stepper motor pin 4.
- `int pin4`: Pin number for stepper motor pin 4.

**Example:** Initialize stepper motor pins on ports PA_1, PA_2, PA_3, and PA_4

```
Stepper_init(GPIOA, 1, GPIOA, 2, GPIOA, 3, GPIOA, 4);
```

### void Stepper_pinOut()

**Function:** Set Stepper Motor Pins According to State and Mode.

```
void Stepper_pinOut(uint32_t state, uint32_t mode);
```

**Parameters:**

- `uint32_t state`: Current state of the stepper motor.
- `uint32_t mode`: Stepper motor driving mode (FULL or HALF).

**Example:** Set stepper motor pins in FULL mode for state 2

```
Stepper_pinOut(2, FULL);
```

### void Stepper_setSpeed()

**Function:** Set Stepper Motor Speed or Convert rpm to ms_delay

```
void Stepper_setSpeed(long whatSpeed);
```

**Parameters:**

- `long whatSpeed`: Desired speed of the stepper motor in revolutions per minute (rpm).

**Example:** Set the stepper motor speed to 300 rpm

```
Stepper_setSpeed(300);
```

### void Stepper_step()

**Function:** Perform Stepper Motor Steps

```
void Stepper_step(uint32_t steps, uint32_t direction, uint32_t mode);
```

**Parameters:**

- `uint32_t steps`: Number of steps to move the stepper motor.
- `uint32_t direction`: Direction of movement.
  - CW: 0 / CCW: 1
- `uint32_t mode`: Stepper motor driving mode.
  - FULL: 1 / HALF: 0

**Example:**

Move the stepper motor forward by 200 steps in FULL mode:

```
Stepper_step(200, 0, FULL);
```

### void Stepper_stop()

**Function:** Stop Stepper Motor

```
void Stepper_stop(void);
```

**Example:** Stop the stepper motor

```
Stepper_stop();
```

## ecUART

### void USART_write()

**Function:**

```
void USART_write(USART_TypeDef * USARTx, uint8_t *buffer, uint32_t nBytes);
```

**Parameters:**

- `USART_TypeDef * USARTx`: USART peripheral to which data will be written (e.g., USART1, USART2, etc.).
- `uint8_t *buffer`: Pointer to the buffer containing the data to be transmitted.
- `uint32_t nBytes`: Number of bytes to be transmitted.

**Example:**

Write data to USART1:

```
uint8_t data[] = "Hello, USART!";
USART_write(USART1, data, sizeof(data)-1);
```

### uint32_t is_USART_RXNE()

**Function:** Check USART RXNE Status

```
uint32_t is_USART_RXNE(USART_TypeDef * USARTx);
```

**Parameters:**

- `USART_TypeDef * USARTx`: USART peripheral for which to check the RXNE status.

**Return:**

- `uint32_t`: Non-zero if RXNE (Read Data Register Not Empty) is set; otherwise, returns zero.

**More detail:**

- RX: read / TX: send

**Example:** MCU receives date from PC 

~~~
void USART2_IRQHandler(){                // USART2 RX Interrupt : Recommended
   if(is_USART2_RXNE()){
      PC_Data = USART2_read();      // RX from UART2 (PC)
      USART1_write(&PC_Data,1);      // TX to USART1    (BT)   
      printf("MCU_1 sent : %c \r\n",PC_Data); // TX to USART2(PC)
   }
}
~~~

### uint8_t USART_read()

**Function:** This function is designed to read a byte from the specified USART peripheral. 

```
uint8_t USART_read(USART_TypeDef * USARTx);
```

**Parameters:**

- `USART_TypeDef * USARTx`: USART peripheral for which to check the RXNE status.

**Return:**

- data which are from USART communication.

### void USART_setting()

**Function:** Initial setting for USART communication

~~~
void USART_setting(USART_TypeDef* USARTx, GPIO_TypeDef* GPIO_TX, int pinTX, GPIO_TypeDef* GPIO_RX, int pinRX, uint32_t baud);
~~~

**Parameters:**

* `USART_TypeDef* USARTx`: USART peripheral for which to use
* `GPIO_TypeDef* GPIO_TX`: Port for TX_USART
* `int pinTX`: Pin for TX_USART
* `GPIO_TypeDef* GPIO_RX`: Port for RX_USART
* `int pinRX`: Pin for RX_USART
* `uint32_t baud`:  baud rate for communication (e.g., 9600, 115200, etc.).

**Example:** 

~~~
USART_setting(USART1, GPIOA,9,GPIOA,10, BAUD_38400);
~~~

### void UART_baud()

**Function:** Set baud rate for USART communication

~~~
void UART_baud(USART_TypeDef* USARTx, uint32_t baud);
~~~

**Parameters:**

- `USART_TypeDef* USARTx`: USART peripheral for which to use

- `uint32_t baud`:  baud rate for communication (e.g., 9600, 115200, etc.).

**Example:** 

~~~
UART_baud(USART1, 9600);
~~~

## ecADC

### void ADC_init()

**Function:**  Initialize Analog-to-Digital Converter (ADC) for a specific pin.

~~~
void ADC_init(PinName_t pinName);
~~~

**Parameters:** 

- `PinName_t pinName`: Pin name or number associated with the ADC channel.

**Example:** Initialize PB_0 to use ADC 

~~~
ADC_init(PB_0);
~~~

### void ADC_trigger()

**Function:**  Set ADC trigger sensing type

~~~
void ADC_trigger(TIM_TypeDef* TIMx, int msec, int edge);
~~~

**Parameters:** 

- `TIM_TypeDef* TIMx`: Timer peripheral (TIM2 or TIM3) to be used for triggering the ADC.
- `int msec`: Time period in milliseconds for the timer.
- `int edge`: Trigger edge configuration .
  - RISE: 1 / FALL: 2 / BOTH: 2

**Example:** Set TIM3 as trigger method with 50ms period and Rising  mode.

~~~
ADC_trigger(TIM3, 50, RISE);
~~~
