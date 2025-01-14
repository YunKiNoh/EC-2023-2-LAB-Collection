/*Mealy: 입력을 하면 반응(출력)이 먼저 나오고 상태(next state)가 갱신된다
  Moore: 입력을 하면 상태(next state)가 먼저 갱신되고 반응(출력)이 뒤따라 온다.

  그렇기에  Mealy는 현 상태에서의 입력에 따라 바로 따라오는 출력을 찾아야 하고,
           Moore는 다음 상태를 먼저 갱신하고 이 상태에서의 출력값을 찾아야 한다. 
           
  그래서 Moore에서는 
  
    void stateOutput(){
      pwmOut = FSM[state].out[PWM];
      ledOut = FSM[state].out[LED];  
      
   이 코드에서 state에 해당하는 값이 next state이고 이에 따른 결과를 PWM, LED라는 
   지정된 위치에서 뽑아내는 것이다.
}
*/

// State definition
#define S0  0
#define S1  1
#define S2  2

// Pin setting
const int ledPin = 13;
const int pwmPin = 11;
const int btnPin = 3;
const int trigPin = 10;
const int echoPin = 7;

// Initialize
unsigned char state = S0;
unsigned char input[2] = {0, 0}; // input[0] = button, input[1] = distance
unsigned char pwmOut = 0;
unsigned char ledOut = LOW;
unsigned long duration;

float distance;
int thresh = 5;

float time = 0;

// State table definition
typedef struct {
  uint32_t out[2][2];      // output = FSM[state].out[PWM or LED]
  uint32_t next[2][2];  // nextstate = FSM[state].next[input X][input Y]
} State_t;

// state[three rows]: 3 types[S0, S1, S2], 
// next state[first column]: 4 types per each state, 
// output of pwm[second column]: 4 reuslts per each type occured from led and distance inputs
State_t FSM[3] = {
  { {{0  , 0 }, {0, 255/2}}, {{S0 , S0}, {S1 , S1}} },
  { {{0  , 255/2 }, {0, 255}}, {{S1 , S1}, {S2 , S2}} },
  { {{0  , 255 }, {0, 0}}, {{S2 , S2}, {S0 , S0}} },
};

void setup() {
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);

  // Initialize pwm pin as an output:
  pinMode(pwmPin, OUTPUT);

  // initialize the pushbutton pin as an interrupt input:
  pinMode(btnPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(btnPin), pressed, FALLING);

  // Initialize the trigger pin as an output
  pinMode(trigPin, OUTPUT);

  // Initialize the echo pin as an input
  pinMode(echoPin, INPUT);
  
  Serial.begin(9600);
}

void loop() {
   // Generate pwm singal on the trigger pin.
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(10);

  // Distance is calculated using how much time it takes.
  duration = pulseIn(echoPin, HIGH);
  distance = (float)duration / 58.0;
  
  // Output State
  stateOutput();

  analogWrite(pwmPin, pwmOut);
  digitalWrite(ledPin, ledOut);
    
  // Calculate next state, then update State
  nextState();
  
  // Print 
  Serial.print("Input = ");
  Serial.print(input[1]);

  Serial.print("    State = ");
  Serial.print(state);
  
  Serial.print("    distance = ");
  Serial.print(distance);
  Serial.println(" [cm]");

  Serial.print("Output : ");
  Serial.print("LED = ");
  if (state == S0){
    Serial.print("OFF ");
    }
  else 
    Serial.print("Blink ");
    
  Serial.print("    PWM = ");
  Serial.println(pwmOut);

  Serial.println(" ");
  
  
  delay(1000);
}

void pressed(){
  input[0] = 1;
  // print out Output before updating present state with next state
  stateOutput();
  nextState();
  // reset the value of input[0](button pressing)
  input[0] = 0;
}

void nextState(){
  state = FSM[state].next[input[0]][input[1]];
}

void stateOutput(){
  // distance value input
  if (distance < thresh)
    input[1] = 1;
  else
    input[1] = 0;

  // Button pressing input
  if (state == S0){
    ledOut = 0;  
  }
    // blink
  else{
    if (millis() - time >= 1000){
      if(ledOut == HIGH)
        ledOut = LOW;
      else 
        ledOut = HIGH;

      time = millis();
    }
  }

  // Assign pwmOut value and ledOut value
  pwmOut = FSM[state].out[input[0]][input[1]];
}
