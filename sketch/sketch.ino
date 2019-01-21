#include <LowPower.h>

#define INTERRUPT_PIN 2

#define RED_PIN 9
#define BLUE_PIN 6
#define GREEN_PIN 3

#define POWER_OFF 254

volatile uint8_t lastRgbSetup;

const uint8_t PROGMEM rgb_setup[4][3] = {
  {255, 0, 0}   /* RED   */,
  {0, 255, 0}   /* GREEN */,
  {0, 0, 255}   /* BLUE  */,
  {230, 90, 30} /* PINK  */
};


void ApplyColor()
{
  analogWrite(RED_PIN, (rgb_setup[lastRgbSetup][0] / 6));
  analogWrite(GREEN_PIN, (rgb_setup[lastRgbSetup][1] / 6));
  analogWrite(BLUE_PIN, (rgb_setup[lastRgbSetup][2] / 6));
}
void setup() {


  Serial.begin(57600);
  pinMode(INTERRUPT_PIN, INPUT);
  digitalWrite(INTERRUPT_PIN, LOW);
  delay(1000);

  pinMode(RED_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);

  lastRgbSetup = 0;

  ApplyColor();

  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), PwmValueChange, RISING);

}



void PwmValueChange() {
  if (lastRgbSetup < 3)
  {
    lastRgbSetup++;
  }
  else
  {
    lastRgbSetup = POWER_OFF;
  }
}

void loop() {
  if (lastRgbSetup == POWER_OFF)
  {
    Serial.println("Deep sleep!");
    delay(100);
    analogWrite(RED_PIN, 0);
    analogWrite(GREEN_PIN, 0);
    analogWrite(BLUE_PIN, 0);

    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
    lastRgbSetup = 0;
  }
  else
  {
    ApplyColor();
  }
  LowPower.idle(SLEEP_FOREVER, ADC_OFF, TIMER2_ON, TIMER1_ON, TIMER0_ON, SPI_OFF, USART0_OFF, TWI_OFF);
}
