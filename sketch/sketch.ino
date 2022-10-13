#include <Arduino.h>
#include <LowPower.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define ACTIVATE_MOVEMENT_INTERRUPT() sbi(PCMSK1, PCINT12)
#define SUSPEND_MOVEMENT_INTERRUPT() cbi(PCMSK1, PCINT12)
#define IS_MOVEMENT_DETECTED() HIGH == (digitalRead(A4))

static const uint16_t SECONDS_BEFORE_AUTO_DEEP_SLEEP = 3600 * 6;
static const uint16_t WDT_COUNT_BEFORE_DEEP_SLEEP = (SECONDS_BEFORE_AUTO_DEEP_SLEEP / 8);

uint16_t wdt_it_counter;

uint8_t rgbOrder = 0;
static const uint8_t SHUTDOWN = 5;

#define DEBUG
#if defined(DEBUG)
#define DEBUG(message)   \
  Serial.print(message); \
  delay(20);
#define DEBUG_LN(message)  \
  Serial.println(message); \
  delay(20);
#else
#define DEBUG(message)
#define DEBUG_LN(message)
#endif
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

static const Color rgb_setup[6] = {
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

void ApplyColor()
{
  if (rgbOrder < 6)
  {
    analogWrite(Pins::RED, rgb_setup[rgbOrder].RED);
    analogWrite(Pins::GREEN, rgb_setup[rgbOrder].GREEN);
    analogWrite(Pins::BLUE, rgb_setup[rgbOrder].BLUE);
  }
  else
  {
    DEBUG("ApplyColor(): rgbOrder has inconsistent value: ");
    DEBUG_LN(rgbOrder);
  }
}

volatile bool buttonPressed;
volatile bool buttonSwitched;
volatile bool movementSensorStateHasChanged;

void Sleep8s()
{
  DEBUG_LN("Sleep8s");
  interruptBackup.Save();
  LowPower.idle(SLEEP_8S, ADC_OFF, TIMER2_ON, TIMER1_ON, TIMER0_ON, SPI_OFF, USART0_OFF, TWI_OFF);
  interruptBackup.Restore();
}

void DeepSleep()
{
  DEBUG_LN("DeepSleep");
  wdt_disable();
  interruptBackup.Save();
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  interruptBackup.Restore();
}

ISR(PCINT1_vect)
{
  SUSPEND_MOVEMENT_INTERRUPT();
  movementSensorStateHasChanged = true;
}

void it_OnButtonChanged()
{
  buttonSwitched = true;
}

void setup()
{
  cli();

#if defined(DEBUG)
  Serial.begin(115200);
#endif
  // All pins as input
  DDRB = 0;
  DDRC = 0;
  DDRD = 0;

  // no ADC
  cbi(ADCSRA, ADEN);

  pinMode(Pins::RED, OUTPUT);
  pinMode(Pins::BLUE, OUTPUT);
  pinMode(Pins::GREEN, OUTPUT);

  // INT0 triggers on LOW level, we can
  // activate the internal pull-up
  pinMode(Pins::BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), it_OnButtonChanged, LOW);

  // PC0 (A4) state change
  pinMode(A4, INPUT);
  sbi(PCICR, PCIE1);
  ACTIVATE_MOVEMENT_INTERRUPT();

  rgbOrder = 0;
  ApplyColor();

  wdt_it_counter = 0;
  buttonPressed = false;
  movementSensorStateHasChanged = false;
  sei();
}

void incrementRgbOrder()
{
  rgbOrder++;
  if (rgbOrder > 5)
  {
    rgbOrder = 0;
  }
}

void loop()
{
  if (movementSensorStateHasChanged == true)
  {
    movementSensorStateHasChanged = false;
    ACTIVATE_MOVEMENT_INTERRUPT();
    if (IS_MOVEMENT_DETECTED())
    {
      DEBUG_LN("Movement detected");
    }
    else
    {
      DEBUG_LN("End of movement detected");
      wdt_it_counter = WDT_COUNT_BEFORE_DEEP_SLEEP;
    }
  }

  if (buttonSwitched)
  {
    buttonSwitched = false;
    DEBUG_LN("Button switched to ");
    if (buttonPressed)
    {
      buttonPressed = false;
      DEBUG_LN("released");
      attachInterrupt(digitalPinToInterrupt(2), it_OnButtonChanged, LOW);
    }
    else
    {
      buttonPressed = true;
      DEBUG_LN("pressed");
      incrementRgbOrder();
      ApplyColor();
      if (rgbOrder == SHUTDOWN)
      {
        rgbOrder = 0;
        wdt_it_counter = WDT_COUNT_BEFORE_DEEP_SLEEP;
        attachInterrupt(digitalPinToInterrupt(2), it_OnButtonChanged, LOW);
      }
      else
      {
        attachInterrupt(digitalPinToInterrupt(2), it_OnButtonChanged, HIGH);
      }
      delay(200); // debounce
    }
  }

  wdt_it_counter++;
  if (wdt_it_counter >= WDT_COUNT_BEFORE_DEEP_SLEEP)
  {
    wdt_it_counter = 0;

    // movement does not wake up when deep sleeping
    SUSPEND_MOVEMENT_INTERRUPT();

    DeepSleep();

    detachInterrupt(digitalPinToInterrupt(2));
    ACTIVATE_MOVEMENT_INTERRUPT();

    // it_OnButtonChanged woke up MCU.
    // we do not want to interpret this
    // as a color change but just wakeup.
    buttonSwitched = false;
    buttonPressed = false;

    ApplyColor();
    delay(500);

    // wait for press again, forget release
    attachInterrupt(digitalPinToInterrupt(2), it_OnButtonChanged, LOW);
  }

  Sleep8s();
}
