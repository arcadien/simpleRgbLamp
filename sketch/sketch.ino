#include <LowPower.h>

#define INTERRUPT_PIN 2

#define RED_PIN 9
#define GREEN_PIN 6
#define BLUE_PIN 3

#define POWER_OFF 254

volatile uint8_t lastRgbSetup;

uint8_t rgb_setup[4][3];


void setup() {

  rgb_setup [0][0] = 10;
  rgb_setup [0][1] = 254;
  rgb_setup [0][2] = 254;
  
  rgb_setup [1][0] = 10;
  rgb_setup [1][1] = 254;
  rgb_setup [1][2] = 254;

  rgb_setup [2][0] = 10;
  rgb_setup [2][1] = 254;
  rgb_setup [2][2] = 254;

  rgb_setup [3][0] = 10;
  rgb_setup [3][1] = 254;
  rgb_setup [3][2] = 254;

  Serial.begin(57600);
  pinMode(INTERRUPT_PIN, INPUT);
  digitalWrite(INTERRUPT_PIN, LOW);
  delay(1000);
    
  pinMode(RED_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  
  lastRgbSetup = 0;
  
  analogWrite(RED_PIN, rgb_setup[lastRgbSetup][0]);
  analogWrite(GREEN_PIN, rgb_setup[lastRgbSetup][1]);
  analogWrite(BLUE_PIN, rgb_setup[lastRgbSetup][2]);

  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), PwmValueChange, RISING);

}



void PwmValueChange(){
  if(lastRgbSetup < 3)
  {
    lastRgbSetup++;
  }
  else
  {
    lastRgbSetup = POWER_OFF;
  }
}

void loop() {


  Serial.print("Current config: ");
  Serial.println(lastRgbSetup);
  delay(200);
  
  if(lastRgbSetup == POWER_OFF)
  {
    Serial.println("Deep sleep!");
    delay(100);
    analogWrite(RED_PIN, 0);
    analogWrite(GREEN_PIN,0);
    analogWrite(BLUE_PIN,0);
  
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
    lastRgbSetup = 0;
  }
   
  analogWrite(RED_PIN, rgb_setup[lastRgbSetup][0]);
  analogWrite(GREEN_PIN, rgb_setup[lastRgbSetup][1]);
  analogWrite(BLUE_PIN, rgb_setup[lastRgbSetup][2]);
  
}
