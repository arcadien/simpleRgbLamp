#include <Arduino.h>
#include <LowPower.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <EEPROM.h>
#include <avr/wdt.h> 

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

int lastUsedColorAddressInEeprom = 0;

static const uint16_t SECONDS_BEFORE_AUTO_DEEP_SLEEP = 3600 * 6;
static const uint16_t _WDT_COUNT_BEFORE_DEEP_SLEEP = (SECONDS_BEFORE_AUTO_DEEP_SLEEP / 8);

uint16_t wdt_it_counter;

struct Pins
{
  static const uint8_t BUTTON = 2; // PD2
  static const uint8_t GREEN = 3;
  static const uint8_t BLUE = 6;
  static const uint8_t RED = 9;
};

/*!
   a RGB color

   Use Reduce() for actual value storing

*/
struct Color
{
  // RGB intensities will be divided by this number.
  // It allows flexibility on global lamp light intensity
  static const int INTENSITY_REDUCTION_FACTOR = 10;

  Color(uint8_t red, uint8_t green, uint8_t blue) : RED(Reduce(red)), GREEN(Reduce(green)), BLUE(Reduce(blue)) {}
  uint8_t RED;
  uint8_t GREEN;
  uint8_t BLUE;

  static uint8_t Reduce(uint8_t in)
  {
    return (in / INTENSITY_REDUCTION_FACTOR);
  }
};

static const Color rgb_setup[7] = {
    Color(0, 0, 0) /* blackout */,
    Color(255, 0, 0) /* RED   */,
    Color(0, 255, 0) /* GREEN */,
    Color(0, 0, 255) /* BLUE  */,
    Color(230, 90, 30) /* PINK  */,
    Color(255, 88, 0) /* ORANGE */,
    Color(0, 0, 0) /* blackout */,
};

struct InterruptBackup
{
  uint8_t _TIMSK0;
  uint8_t _TIMSK1;
  uint8_t _TIMSK2;

  void Save()
  {
    _TIMSK0 = TIMSK0;
    _TIMSK1 = TIMSK1;
    _TIMSK2 = TIMSK2;
    TIMSK0 = 0;
    TIMSK1 = 0;
    TIMSK2 = 0;
  }

  void Restore()
  {
    TIMSK0 = _TIMSK0;
    TIMSK1 = _TIMSK1;
    TIMSK2 = _TIMSK2;
  }
};
InterruptBackup interruptBackup;

uint8_t rgbOrder = 1;
static const uint8_t SHUTDOWN = 6;

void ApplyColor()
{

  if (rgbOrder > SHUTDOWN)
    rgbOrder = 0;
  analogWrite(Pins::RED, rgb_setup[rgbOrder].RED);
  analogWrite(Pins::GREEN, rgb_setup[rgbOrder].GREEN);
  analogWrite(Pins::BLUE, rgb_setup[rgbOrder].BLUE);
}

// just ²ke up
volatile bool buttonPressed;
volatile bool movementDetected;

void it_OnButtonPressed()
{
  buttonPressed = true;
}

ISR(PCINT2_vect)
{
  movementDetected = true;
}

ISR(PCINT0_vect)
{
  movementDetected = true;
}

void setup()
{

  Serial.begin(115200);
  // All pins as input
  DDRB = 0;
  DDRC = 0;
  DDRD = 0;

  pinMode(Pins::RED, OUTPUT);
  pinMode(Pins::BLUE, OUTPUT);
  pinMode(Pins::GREEN, OUTPUT);

  // PB2 (10) state change
  sbi(PCIFR, PCIF0);
  sbi(PCMSK0, PCINT2 | PCINT3 | PCINT4);

  // INT0 triggers on LOW level, we can
  // activate the internal pull-up
  pinMode(Pins::BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), it_OnButtonPressed, LOW);

  // EICRA = 0;
  rgbOrder = EEPROM.read(lastUsedColorAddressInEeprom);
  buttonPressed = false;
  wdt_it_counter = 0;
  ApplyColor();
  sei();
}

void Sleep8s()
{
  Serial.println("Sleep8s");
  delay(100);
  sbi(EIMSK, INT0);
  sbi(PCMSK0, PCINT2);
  interruptBackup.Save();
  LowPower.idle(SLEEP_8S, ADC_OFF, TIMER2_ON, TIMER1_ON, TIMER0_ON, SPI_OFF, USART0_OFF, TWI_OFF);
  interruptBackup.Restore();
}

void DeepSleep()
{
  Serial.println("DeepSleep");
  delay(100);
  rgbOrder = 0;
  ApplyColor();
  wdt_disable();
  sbi(EIMSK, INT0);
  sbi(PCMSK0, PCINT2);
  interruptBackup.Save();
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  interruptBackup.Restore();
  ApplyColor();
}

void OnButtonPressed()
{
  rgbOrder++;
  ApplyColor();
  if (rgbOrder >= SHUTDOWN)
  {
    delay(200);
    DeepSleep();
  }
  else
  {
    // save last non-dark color order
    if (rgbOrder > 0 && rgbOrder < SHUTDOWN)
    {
      uint8_t _SREG = SREG;
      cli();
      EEPROM.put(lastUsedColorAddressInEeprom, rgbOrder);
      SREG = _SREG;
      delay(200);
    }
    Sleep8s();
  }
}

void InactivityDeepSleep()
{
  Serial.println("InactivityDeepSleep");
  delay(100);
  rgbOrder = 0;
  ApplyColor();
  sbi(EIMSK, INT0);
  sbi(PCMSK0, PCINT2);
  interruptBackup.Save();
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  interruptBackup.Restore();
  rgbOrder = EEPROM.read(lastUsedColorAddressInEeprom);

  // do not interpret the button press when just waking
  // up from inactivity deep sleep
  buttonPressed = false;

  ApplyColor();
}

void loop()
{

  cbi(EIMSK, INT0);
  cbi(PCMSK0, PCINT2);

  if (movementDetected == true)
  {
    Serial.println("Movement detected");
    movementDetected = false;
    OnButtonPressed();
  }
  else if (buttonPressed == true)
  {
    Serial.println("Button pressed");
    buttonPressed = false;
    OnButtonPressed();
  }
  else
  {
    wdt_it_counter++;
    if (wdt_it_counter >= _WDT_COUNT_BEFORE_DEEP_SLEEP)
    {
      wdt_it_counter = 0;
      InactivityDeepSleep();
    }
    else
    {
      Sleep8s();
    }
  }
}
