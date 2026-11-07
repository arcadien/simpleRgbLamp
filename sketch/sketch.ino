#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <x10rf.h>

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define AMBIANT_LIGHT_TRESHOLD 25
// 8 MHz internal clock needs >= 2.7 V; 3.0 V also protects the 18650
#define LOW_POWER_MV 3000

#define RADIO_PIN PIN_PA2
#define X10_IDENTIFIER 0xCC
#define X10_REPEAT_COUNT 3

#define ADC_BANDGAP 0x21 // MUX 100001: 1.1 V bandgap, VCC reference

#define IS_MOVEMENT_DETECTED() HIGH == (digitalRead(Pins::PIR))

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
  static const uint8_t BUTTON = PIN_PB2;     // PCINT10
  static const uint8_t PIR = PIN_PA3;        // PCINT3
  static const uint8_t LUMINOSITY = A1;      // ADC channel 1 = PA1
  static const uint8_t LDR_POWER = PIN_PA0;  // powers the LDR divider
  static const uint8_t STEPUP_OFF = PIN_PB0; // BC327 PNP base (via 220R), LOW = LEDs powered
  static const uint8_t RED = PIN_PA7;        // OC0B
  static const uint8_t GREEN = PIN_PA6;      // OC1A
  static const uint8_t BLUE = PIN_PA5;       // OC1B
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

// The step-up only runs while at least one LED is lit
static void setRgb(uint8_t red, uint8_t green, uint8_t blue) {
  digitalWrite(Pins::STEPUP_OFF, (red | green | blue) ? LOW : HIGH);
  analogWrite(Pins::RED, red);
  analogWrite(Pins::GREEN, green);
  analogWrite(Pins::BLUE, blue);
}

void ApplyColor() {
  if (rgbOrder < 6) {
    setRgb(rgb_setup[rgbOrder].RED, rgb_setup[rgbOrder].GREEN,
           rgb_setup[rgbOrder].BLUE);
  } else {
    SERIAL_OUTPUT("ApplyColor(): rgbOrder has inconsistent value: ");
    SERIAL_OUTPUT_LN(rgbOrder);
  }
}

volatile bool buttonPressed;
volatile bool buttonSwitched;
volatile bool movementSensorStateHasChanged;

// Idle keeps the PWM timers running; any interrupt (incl. millis) wakes up
void IdleSleep() {
  SERIAL_OUTPUT_LN("Sleep");
  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_mode();
}

void DeepSleep() {
  SERIAL_OUTPUT_LN("DeepSleep");

  setRgb(0, 0, 0);

  // movement and button (pin change interrupts) can wakeup
  uint8_t adcsra = ADCSRA;
  ADCSRA = 0; // ADC would draw current in power-down
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  cli();
  sleep_enable();
#if defined(sleep_bod_disable)
  sleep_bod_disable();
#endif
  sei();
  sleep_cpu();
  sleep_disable();
  ADCSRA = adcsra;
}

ISR(PCINT0_vect) { movementSensorStateHasChanged = true; }

// Pin change (not INT0 edge): only it can wake the ATtiny from power-down
ISR(PCINT1_vect) { buttonSwitched = true; }

void setup() {
  cli();

#if defined(SERIAL_OUT)
  // shortcut: ATTinyCore soft serial TX is PA1 (the LDR), debug only
  Serial.begin(115200);
#endif
  // All pins as input
  DDRA = 0;
  DDRB = 0;
  // HWR0: pull-ups on the unconnected pins so they don't float in sleep
  PORTA = _BV(PA4); // ISP SCK only
  PORTB = _BV(PB1); // not connected

  digitalWrite(Pins::STEPUP_OFF, HIGH); // step-up off before driving the pin
  pinMode(Pins::STEPUP_OFF, OUTPUT);
  pinMode(Pins::LDR_POWER, OUTPUT); // LOW: LDR divider unpowered
  pinMode(RADIO_PIN, OUTPUT);

  pinMode(Pins::RED, OUTPUT);
  pinMode(Pins::BLUE, OUTPUT);
  pinMode(Pins::GREEN, OUTPUT);

  pinMode(Pins::BUTTON, INPUT_PULLUP);
  sbi(GIMSK, PCIE1);
  sbi(PCMSK1, PCINT10);

  pinMode(Pins::PIR, INPUT);
  sbi(GIMSK, PCIE0);
  sbi(PCMSK0, PCINT3);
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

  setRgb(0, 0, 100);
  _delay_ms(100);

  setRgb(0, 0, 0);
  _delay_ms(100);

  setRgb(0, 0, 100);
  _delay_ms(100);

  setRgb(0, 0, 0);
}

static void notifyLowPower() {

  setRgb(100, 0, 0);
  _delay_ms(50);

  setRgb(0, 0, 0);
  _delay_ms(50);

  setRgb(100, 0, 0);
  _delay_ms(50);

  setRgb(0, 0, 0);
}

static struct Monitoring performMonitoring() {
  Monitoring monitoring;
  digitalWrite(Pins::LDR_POWER, HIGH);
  monitoring.luminosity = analogRead(Pins::LUMINOSITY);
  monitoring.luminosity += analogRead(Pins::LUMINOSITY);
  monitoring.luminosity /= 2;
  digitalWrite(Pins::LDR_POWER, LOW);

  // VCC is the battery: measure the 1.1 V bandgap against VCC
  analogRead(ADC_BANDGAP);
  _delay_ms(2); // bandgap settling after the mux change
  uint16_t bandgap = analogRead(ADC_BANDGAP);
  bandgap += analogRead(ADC_BANDGAP);
  bandgap += analogRead(ADC_BANDGAP);
  bandgap += analogRead(ADC_BANDGAP);
  bandgap /= 4;
  // shortcut: nominal 1.1 V (+/-10 %), calibrate per chip if mV accuracy matters
  monitoring.batteryVoltage = (1100UL * 1024UL) / bandgap;

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
        cbi(PCMSK1, PCINT10);
        SERIAL_OUTPUT_LN("Button clicked, color bump");
        incrementRgbOrder();
        ApplyColor();
        _delay_ms(500);
        sbi(PCMSK1, PCINT10);
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
    IdleSleep();
  }
}
