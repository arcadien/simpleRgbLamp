#include <Arduino.h>
#include <LowPower.h>
#include <avr/power.h>
#include<avr/interrupt.h>
#include <EEPROM.h>

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

int lastUsedColorAddressInEeprom = 0;


struct Pins {
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
  Color(0,   0, 0)   /* blackout */,
  Color(255, 0, 0)   /* RED   */,
  Color(0, 255, 0)   /* GREEN */,
  Color(0, 0, 255) /* BLUE  */,
  Color(230, 90, 30)  /* PINK  */  ,
  Color(255, 88, 0)   /* ORANGE */,
  Color(0, 0, 0)   /* blackout */,
};

struct InterruptBackup {
  uint8_t _TIMSK0;
  uint8_t _TIMSK1;
  uint8_t _TIMSK2;

  void Save()
  {
    _TIMSK0 = TIMSK0;
    _TIMSK1 = TIMSK1;
    _TIMSK2 = TIMSK2;
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

  if (rgbOrder > SHUTDOWN) rgbOrder = 0;
  analogWrite(Pins::RED,    rgb_setup[rgbOrder].RED);
  analogWrite(Pins::GREEN,  rgb_setup[rgbOrder].GREEN);
  analogWrite(Pins::BLUE,   rgb_setup[rgbOrder].BLUE);

}

// just wake up
ISR(INT0_vect) {}

void PrepareForSleep()
{

  EEPROM.put(lastUsedColorAddressInEeprom, rgbOrder);

  // PWM will be running in sleep mode,
  // we disable their interrupts here to
  // avoid immediate wakeup
  interruptBackup.Save();

  TIMSK0 = 0;
  TIMSK1 = 0;
  TIMSK2 = 0;

  // enable INT0 interrupt
  sbi(EIMSK, INT0);
}

void WakeUp() {
  // disable INT0
  cbi(EIMSK, INT0);
  interruptBackup.Restore();
  rgbOrder = EEPROM.read(lastUsedColorAddressInEeprom);
}



void setup() {

  // All pins as input
  DDRB = 0;
  DDRC = 0;
  DDRD = 0;

  pinMode(Pins::RED, OUTPUT);
  pinMode(Pins::BLUE, OUTPUT);
  pinMode(Pins::GREEN, OUTPUT);

  pinMode(Pins::BUTTON, INPUT);

  // trigger on LOW level, we can
  // activate the internal pull-up
  digitalWrite(Pins::BUTTON, HIGH);

  // setup INT0 to fire on LOW level
  cbi(EICRA, ISC00);
  cbi(EICRA, ISC01);

  rgbOrder = EEPROM.read(lastUsedColorAddressInEeprom);

  sei();
}

void loop() {

  // disable ext. interrupt
  cbi(EIMSK, INT0);

  ApplyColor();
  delay(250);

  if (rgbOrder >= SHUTDOWN)
  {
    rgbOrder = 0;
    PrepareForSleep();
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
    WakeUp();
  }
  PrepareForSleep();
  LowPower.idle(SLEEP_FOREVER, ADC_OFF, TIMER2_ON, TIMER1_ON, TIMER0_ON, SPI_OFF, USART0_OFF, TWI_OFF);
  WakeUp();
  rgbOrder++;
}
