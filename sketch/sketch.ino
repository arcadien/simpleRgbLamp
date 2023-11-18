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

#define AMBIANT_LIGHT_TRESHOLD 35

#define IS_MOVEMENT_DETECTED() HIGH == (digitalRead(A4))


bool movementDetected = false;
uint8_t rgbOrder = 0;
static const uint8_t SHUTDOWN = 5;

#if defined(DEBUG)
#define DEBUG(message) \
  Serial.print(message); \
  delay(20);
#define DEBUG_LN(message) \
  Serial.println(message); \
  delay(20);
#else
#define DEBUG(message)
#define DEBUG_LN(message)
#endif
struct Pins {
  static const uint8_t BUTTON = 2;  // PD2
  static const uint8_t BATTERY = A1;
  static const uint8_t LUMINOSITY = A0;
  static const uint8_t GREEN = 3;
  static const uint8_t BLUE = 6;
  static const uint8_t RED = 9;
};

/*!
   a RGB color

   Use Reduce() for actual value storing

*/
struct Color {
  // RGB intensities will be divided by this number.
  // It allows flexibility on global lamp light intensity
  static const int INTENSITY_REDUCTION_FACTOR = 10;

  Color(uint8_t red, uint8_t green, uint8_t blue)
    : RED(Reduce(red)), GREEN(Reduce(green)), BLUE(Reduce(blue)) {}
  uint8_t RED;
  uint8_t GREEN;
  uint8_t BLUE;

  static uint8_t Reduce(uint8_t in) {
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

struct InterruptBackup {
  uint8_t _TIMSK0;
  uint8_t _TIMSK1;
  uint8_t _TIMSK2;

  void Save() {
    _TIMSK0 = TIMSK0;
    _TIMSK1 = TIMSK1;
    _TIMSK2 = TIMSK2;
    TIMSK0 = 0;
    TIMSK1 = 0;
    TIMSK2 = 0;
  }

  void Restore() {
    TIMSK0 = _TIMSK0;
    TIMSK1 = _TIMSK1;
    TIMSK2 = _TIMSK2;
  }
};
InterruptBackup interruptBackup;

void ApplyColor() {
  if (rgbOrder < 6) {
    analogWrite(Pins::RED, rgb_setup[rgbOrder].RED);
    analogWrite(Pins::GREEN, rgb_setup[rgbOrder].GREEN);
    analogWrite(Pins::BLUE, rgb_setup[rgbOrder].BLUE);
  } else {
    DEBUG("ApplyColor(): rgbOrder has inconsistent value: ");
    DEBUG_LN(rgbOrder);
  }
}

volatile bool buttonPressed;
volatile bool buttonSwitched;
volatile bool movementSensorStateHasChanged;

void Sleep(period_t duration) {
  DEBUG_LN("Sleep");
  interruptBackup.Save();
  LowPower.idle(duration, ADC_OFF, TIMER2_ON, TIMER1_ON, TIMER0_ON, SPI_OFF, USART0_OFF, TWI_OFF);
  interruptBackup.Restore();
}

void DeepSleep() {
  DEBUG_LN("DeepSleep");

  analogWrite(Pins::RED, 0);
  analogWrite(Pins::GREEN, 0);
  analogWrite(Pins::BLUE, 0);

  interruptBackup.Save();

  // movement and button can wakeup
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
}

ISR(PCINT1_vect) {
  movementSensorStateHasChanged = true;
}

void it_OnButtonChanged() {
  buttonSwitched = true;
}

void setup() {
  cli();

#if defined(DEBUG)
  Serial.begin(115200);
#endif
  // All pins as input
  DDRB = 0;
  DDRC = 0;
  DDRD = 0;

  pinMode(Pins::BATTERY, INPUT);
  pinMode(Pins::LUMINOSITY, INPUT);

  analogRead(Pins::BATTERY);
  analogRead(Pins::LUMINOSITY);

  pinMode(Pins::RED, OUTPUT);
  pinMode(Pins::BLUE, OUTPUT);
  pinMode(Pins::GREEN, OUTPUT);

  // INT0 triggers on LOW level, we can
  // activate the internal pull-up
  pinMode(Pins::BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Pins::BUTTON), it_OnButtonChanged, CHANGE);

  // PC0 (A4) state change
  pinMode(A4, INPUT);
  sbi(PCICR, PCIE1);
  sbi(PCMSK1, PCINT12);
  movementSensorStateHasChanged = false;

  rgbOrder = 0;
  ApplyColor();
  sei();
}

void incrementRgbOrder() {
  rgbOrder++;
  if (rgbOrder > 5) {
    rgbOrder = 0;
  }
}

struct Monitoring {
  uint16_t luminosity;
  uint16_t batteryVoltage;
};

static void notifyTooMuchLight() {

  analogWrite(Pins::RED, 0);
  analogWrite(Pins::GREEN, 0);
  analogWrite(Pins::BLUE, 100);
  _delay_ms(200);

  analogWrite(Pins::RED, 0);
  analogWrite(Pins::GREEN, 0);
  analogWrite(Pins::BLUE, 0);
  _delay_ms(200);

  analogWrite(Pins::RED, 0);
  analogWrite(Pins::GREEN, 0);
  analogWrite(Pins::BLUE, 100);
  _delay_ms(200);

  analogWrite(Pins::RED, 0);
  analogWrite(Pins::GREEN, 0);
  analogWrite(Pins::BLUE, 0);
}

static struct Monitoring performMonitoring() {
  Monitoring monitoring;
  monitoring.luminosity = analogRead(Pins::LUMINOSITY);
  monitoring.luminosity += analogRead(Pins::LUMINOSITY);
  monitoring.luminosity /= 2;
  uint16_t batlevel = analogRead(Pins::BATTERY);
  batlevel += analogRead(Pins::BATTERY);
  batlevel += analogRead(Pins::BATTERY);
  batlevel += analogRead(Pins::BATTERY);
  batlevel /= 4;
  monitoring.batteryVoltage = batlevel * (5000 / 1024);
  return monitoring;
}

void loop() {

  Monitoring monitoring = performMonitoring();

#if defined(DEBUG)
  DEBUG("LOOP: ");
  DEBUG(monitoring.batteryVoltage);
  DEBUG(" mV;");
  DEBUG(monitoring.luminosity);
  DEBUG(" ADC;");
  DEBUG_LN(movementDetected ? "mvt" : "nomvt");
  delay(10);
#endif

  if (buttonSwitched) {
    buttonSwitched = false;
    bool buttonIsReleased = (digitalRead(Pins::BUTTON) == HIGH);
    if (buttonIsReleased) {
      detachInterrupt(digitalPinToInterrupt(Pins::BUTTON));
      DEBUG_LN("Button clicked, color bump");
      incrementRgbOrder();
      ApplyColor();
      _delay_ms(500);
      attachInterrupt(digitalPinToInterrupt(Pins::BUTTON), it_OnButtonChanged, CHANGE);
      if (rgbOrder == SHUTDOWN) {
        rgbOrder = 0;

#if defined(DEBUG)
        DEBUG_LN("Manual shutdown")
        // give time to Serial
        delay(10);
#endif
        DeepSleep();
      }
    }
  }

  if (monitoring.luminosity < AMBIANT_LIGHT_TRESHOLD) {

    if (movementSensorStateHasChanged == true) {
      DEBUG_LN("Movement event!");
      movementSensorStateHasChanged = false;

      if (IS_MOVEMENT_DETECTED()) {
        DEBUG_LN("Movement is detected");
        movementDetected = true;

      } else {
        DEBUG_LN("No more movement");
        movementDetected = false;
      }
    }
    ApplyColor();

#if defined(DEBUG)
    // give time to Serial
    delay(10);
#endif

    if (movementDetected) {
      Sleep(SLEEP_8S);
    } else {
      DeepSleep();
    }

  } else {
    DEBUG_LN("Too much light!");
#if defined(DEBUG)
    // give time to Serial
    delay(10);
#endif
    if (!buttonSwitched) {
      notifyTooMuchLight();
    }
    DeepSleep();
  }
}
