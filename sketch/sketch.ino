#include <Arduino.h>
#include <LowPower.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <x10rf.h>

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define AMBIANT_LIGHT_TRESHOLD 25
#define LOW_POWER_MV 2500

#define RADIO_PIN 10
#define X10_IDENTIFIER 0xCC
#define X10_REPEAT_COUNT 3

#define IS_MOVEMENT_DETECTED() HIGH == (digitalRead(A4))

x10rf x10Meter(RADIO_PIN, X10_REPEAT_COUNT);

bool movementDetected = false;
uint8_t rgbOrder = 0;
static const uint8_t SHUTDOWN = 5;

//#define SERIAL_OUT

#if defined(SERIAL_OUT)

#define SERIAL_OUTPUT(message)                                                 \
  Serial.print(message);                                                       \
  delay(20);
#define SERIAL_OUTPUT_LN(message)                                              \
  Serial.println(message);                                                     \
  delay(20);
#else
#define SERIAL_OUTPUT(message)
#define SERIAL_OUTPUT_LN(message)
#endif

struct Pins {
  static const uint8_t BUTTON = 2; // PD2
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
    Color(255, 0, 0) /* RED   */,   Color(0, 255, 0) /* GREEN */,
    Color(0, 0, 255) /* BLUE  */,   Color(230, 90, 30) /* PINK  */,
    Color(255, 88, 0) /* ORANGE */, Color(0, 0, 0) /* blackout */,
};

void ApplyColor() {
  if (rgbOrder < 6) {
    analogWrite(Pins::RED, rgb_setup[rgbOrder].RED);
    analogWrite(Pins::GREEN, rgb_setup[rgbOrder].GREEN);
    analogWrite(Pins::BLUE, rgb_setup[rgbOrder].BLUE);
  } else {
    SERIAL_OUTPUT("ApplyColor(): rgbOrder has inconsistent value: ");
    SERIAL_OUTPUT_LN(rgbOrder);
  }
}

volatile bool buttonPressed;
volatile bool buttonSwitched;
volatile bool movementSensorStateHasChanged;

void Sleep(period_t duration) {
  SERIAL_OUTPUT_LN("Sleep");
  LowPower.idle(duration, ADC_OFF, TIMER2_ON, TIMER1_ON, TIMER0_ON, SPI_OFF,
                USART0_OFF, TWI_OFF);
}

void DeepSleep() {
  SERIAL_OUTPUT_LN("DeepSleep");

  analogWrite(Pins::RED, 0);
  analogWrite(Pins::GREEN, 0);
  analogWrite(Pins::BLUE, 0);

  // movement and button can wakeup
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
}

ISR(PCINT1_vect) { movementSensorStateHasChanged = true; }

void it_OnButtonChanged() { buttonSwitched = true; }

void setup() {
  cli();

#if defined(SERIAL_OUTPUT)
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
  attachInterrupt(digitalPinToInterrupt(Pins::BUTTON), it_OnButtonChanged,
                  CHANGE);

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
  _delay_ms(100);

  analogWrite(Pins::RED, 0);
  analogWrite(Pins::GREEN, 0);
  analogWrite(Pins::BLUE, 0);
  _delay_ms(100);

  analogWrite(Pins::RED, 0);
  analogWrite(Pins::GREEN, 0);
  analogWrite(Pins::BLUE, 100);
  _delay_ms(100);

  analogWrite(Pins::RED, 0);
  analogWrite(Pins::GREEN, 0);
  analogWrite(Pins::BLUE, 0);
}

static void notifyLowPower() {

  analogWrite(Pins::RED, 100);
  analogWrite(Pins::GREEN, 0);
  analogWrite(Pins::BLUE, 0);
  _delay_ms(50);

  analogWrite(Pins::RED, 0);
  analogWrite(Pins::GREEN, 0);
  analogWrite(Pins::BLUE, 0);
  _delay_ms(50);

  analogWrite(Pins::RED, 100);
  analogWrite(Pins::GREEN, 0);
  analogWrite(Pins::BLUE, 0);
  _delay_ms(50);

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
  monitoring.batteryVoltage = batlevel * (5000.0 / 1024.0);

#if defined(SERIAL_OUT)
  SERIAL_OUTPUT("Monitoring: ");
  SERIAL_OUTPUT(monitoring.batteryVoltage);
  SERIAL_OUTPUT(" mV;");
  SERIAL_OUTPUT(monitoring.luminosity);
  SERIAL_OUTPUT(" ADC;");
  SERIAL_OUTPUT_LN(movementDetected ? "mvt" : "nomvt");
  delay(10);
#endif

  return monitoring;
}

static uint16_t dec16ToHex(uint16_t input) {
  uint16_t result = 0;
  static const uint8_t FACTOR_COUNT = 3;
  static const uint16_t factors[FACTOR_COUNT] = {10, 100, 1000};

  for (uint8_t i = FACTOR_COUNT; i > 0; i--) {
    uint16_t factor = factors[i - 1];
    uint8_t howManyPow10 = (input / factor);
    if (howManyPow10 > 0) {
      result += (howManyPow10 & 0x0f) << (i * 4);
      input -= (howManyPow10 * factor);
    }
  }
  if (input > 0) {
    result += (input & 0x0f);
  }
  return result;
}

bool sendPowerAtNextCycle = false;
void loop() {

  Monitoring monitoring = performMonitoring();

  if (sendPowerAtNextCycle) {
    x10Meter.RFXmeter(X10_IDENTIFIER, 0x00,
                      dec16ToHex(monitoring.batteryVoltage));
    sendPowerAtNextCycle = false;
  }

  bool doDeepSleep = false;
  bool doSleep = false;

  if (monitoring.batteryVoltage > LOW_POWER_MV) {

    if (buttonSwitched) {
      buttonSwitched = false;
      bool buttonIsReleased = (digitalRead(Pins::BUTTON) == HIGH);
      if (buttonIsReleased) {
        detachInterrupt(digitalPinToInterrupt(Pins::BUTTON));
        SERIAL_OUTPUT_LN("Button clicked, color bump");
        incrementRgbOrder();
        ApplyColor();
        _delay_ms(500);
        attachInterrupt(digitalPinToInterrupt(Pins::BUTTON), it_OnButtonChanged,
                        CHANGE);
        if (rgbOrder == SHUTDOWN) {
          rgbOrder = 0;
          SERIAL_OUTPUT_LN("Manual shutdown")
          doDeepSleep = true;
        }
      }
    } else if (monitoring.luminosity < AMBIANT_LIGHT_TRESHOLD) {

      if (movementSensorStateHasChanged == true) {
        SERIAL_OUTPUT_LN("Movement event!");
        movementSensorStateHasChanged = false;

        if (IS_MOVEMENT_DETECTED()) {
          SERIAL_OUTPUT_LN("Movement is detected");
          movementDetected = true;

        } else {
          SERIAL_OUTPUT_LN("No more movement");
          movementDetected = false;
        }
      }
      ApplyColor();

      if (movementDetected) {
        doSleep = true;
      } else {
        // end of movement detection
        doDeepSleep = true;
      }

    } else {
      SERIAL_OUTPUT_LN("Too much light!");
      if (!buttonSwitched) {
        notifyTooMuchLight();
      }
      doDeepSleep = true;
    }
  } else {

    // button may have been clicked
    buttonSwitched = false;

    // /!\ low power
    notifyLowPower();
    doDeepSleep = true;
  }

  delay(10);
  if (doDeepSleep) {
    sendPowerAtNextCycle = true;
    DeepSleep();
  } else if (doSleep) {
    // forever, but with active PWM
    // will be sendPowerAtNextCycle at least by end of movement event
    Sleep(SLEEP_FOREVER);
  }
}
