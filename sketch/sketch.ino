#define INTERRUPT_PIN 2
#define RED_PIN 3
#define GREEN_PIN 6
#define BLUE_PIN 9

volatile uint8_t PWM_VALUE;
uint8_t lastPwmValue;

void setup() {

  Serial.begin(57600);
  pinMode(INTERRUPT_PIN, INPUT);
  digitalWrite(INTERRUPT_PIN, LOW);
  delay(1000);
  
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), PwmValueChange, RISING);
  
  pinMode(RED_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);

  PWM_VALUE = 250;
  lastPwmValue = PWM_VALUE;
  
  analogWrite(RED_PIN, PWM_VALUE);
  analogWrite(BLUE_PIN, PWM_VALUE);
  analogWrite(GREEN_PIN, PWM_VALUE);

}



void PwmValueChange(){
  if(PWM_VALUE >= 50)
  {
    PWM_VALUE -= 50;
  }
  else
  {
    PWM_VALUE = 250;
  }
}

void loop() {
   if(lastPwmValue != PWM_VALUE)
   {
    Serial.print("PWM value changed!");
    Serial.println(PWM_VALUE);
    lastPwmValue = PWM_VALUE;
   }
   
   analogWrite(RED_PIN, PWM_VALUE);
   analogWrite(BLUE_PIN, PWM_VALUE);
   analogWrite(GREEN_PIN, PWM_VALUE);
  
   // busy-wait to replace by a power mode switch
   delay(200);
}
