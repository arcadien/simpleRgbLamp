# Power saving multicolor light

A battery-powered RGB lamp able to detect movement and ambiant light to adapt its behavior.

_The behaviour is simple : Once it starts, the first color is red. A click on the button (on release) cycles the colors: red, green, blue, pink, orange. After orange, a click turns the lamp off (deep sleep).
When in deep sleep, lamp can be woken up by button press or movement detection. If ambiant luminosity is too high, the lamp emits a quick blue blink and shuts down. If the battery is too low, it blinks red and shuts down.
If the ambiant luminosity is low, the lamp remains on the selected color as long as movement is detected. On each wake-up from deep sleep, the battery voltage is sent over 433Mhz._

The repository contains:

- A PlatformIO firmware for the ATtiny84A (`sketch/`)
- The [Kicad](http://kicad.org/) schema (`kicad/`)
- A [VeroRoute](https://sourceforge.net/projects/veroroute/) implantation layout (_outdated: predates the current schematic_)

# Firmware

Each wake-up, the main loop measures battery and light, then picks one state by priority:

| Priority | State         | Condition                                  | Action                                                   |
| -------- | ------------- | ------------------------------------------ | -------------------------------------------------------- |
| 1        | `LowBattery`  | battery ≤ `LOW_POWER_MV`                   | blink red, deep sleep                                    |
| 2        | `ColorSelect` | button released                            | show next color; after orange, blackout and deep sleep   |
| 3        | `TooBright`   | light ≥ `AMBIANT_LIGHT_TRESHOLD`           | blink blue, deep sleep                                   |
| 4        | `NightMotion` | dark and PIR high                          | show color, idle sleep (PWM keeps running)               |
| 5        | `NightIdle`   | dark and PIR low                           | deep sleep                                               |

While in `NightMotion` the LDR is not read: the lamp's own light would trip `TooBright` and switch it off by itself.
The PIR level is read directly; its interrupt only wakes the MCU.

Build and flash (from `sketch/`, USBasp by default, see the ISP warning below):

```
pio run                 # build
pio run -t fuses        # once: internal 8 MHz, BOD 2.7 V
pio run -t upload
```

Uncomment `#define SERIAL_OUT` in `sketch.ino` to log state transitions.

# Software

## High level requirements

| Identifier             | Description                                                                                               |
| ---------------------- | --------------------------------------------------------------------------------------------------------- |
| [HLR1](../../issues/2) | ~PSML button allow to change the color in a set of 5 colors (RED, GREEN, BLUE, ORANGE, PINK)~             |
| [HLR2](../../issues/3) | ~PSML detects ambient light and does not power up if luminosity is more than AMBIANT_LIGHT_TRESHOLD (ADC counts)~ |
| [HLR3](../../issues/4) | ~PSML wakes up when it detects movement around, and go back to deep sleeps when movement detection ends~  |
| [HLR4](../../issues/5) | ~Duplicate of HLR1~                                                                                       |
| [HLR5](../../issues/6) | PSML transmit its remaning battery level wirelessly, each 15 minutes (_partial: sent on each wake-up from deep sleep, no periodic wake-up yet_) |
| [HLR6](../../issues/7) | PSML transmit light intensity wirelessly, each 15 minutes (_not implemented_)                             |
| HLR7                   | Long press (2 seconds) on push button toggles permanent light mode (_not implemented, no issue yet_)      |

## Architecture

## Components and data

NOTE: _data_ is a value which is exchanged between components

```
 HW -------------------                    --> BatteryVoltageSensor ------> D_tension_in_mv
 HW -------------------                    --> LuminositySensor ----------> D_luminosity
 HW -------------------                    --> PushButton     ------------> D_ButtonPressed
 HW -------------------                    --> PushButton ----------------> D_ButtonLongPressed
 HW -------------------                    --> TimeCounter ---------------> D_EmitData
 D_tension_in_mv,D_luminosity, D_EmitData  --> X10MessageFactory  --------> D_X10Message
 D_X10Message,D_SendBatteryVoltage         --> X10Encoder ----------------> HW
```

# Hardware

## Architecture

```
                     |--------|
 Battery ----------->|        |
                     |        |
 Presence Sensor --->|        |
                     |        |--> RGB LEDs
 Luminosity Sensor ->|  PSML  |
                     |        |--> 433Mhz transmitter
 Push button  ------>|        |
                     |--------|

```

- Battery is a 18650, charged through a TP4056 module (micro-USB, with protection). The MCU and sensors run directly on the battery (`VBAT`, 3.0–4.2 V)
- A MT3608 step-up module provides 5 V for the LEDs only; a BC327 PNP (base via 220 Ω from `PB0`, 10k base-emitter) cuts its input when the LEDs are off
- Light sensor is a simple photoresistor GL5528 (10–20 kΩ at 10 lux, ≥1 MΩ dark), powered from `PA0` only while measuring, with a 10k pull-down to GND
- Presence sensor is an AM312 mini PIR (2.7–12 V)
- 433Mhz transmitter is a FS1000A module
- LEDs are switched low-side by BC337 NPN transistors, 1k base resistors (≤ ~45 mA per color at 3.0 V; common anode LED, total ≤ 0.1 A)
- Push button is available from outside enclosing device
- PSML itself is an [ATtiny84A](https://www.microchip.com/en-us/product/ATtiny84A) on its internal 8 MHz oscillator, programmed through a 2×3 ISP header
  - ⚠️ The ISP header VCC is the battery rail: never let the programmer power the target (no 5 V jumper on a USBasp). Program with the battery in place and a programmer set to 3.3 V logic, or with the battery removed and 3.3 V supplied by the programmer
- A 100 µF capacitor on `VBAT` absorbs the inrush when the step-up switches on

| Identifier | Description                                                                                                             |
| ---------- | ----------------------------------------------------------------------------------------------------------------------- |
| HWR0       | ~Unless specified, all pins shall be set as 'input/pull up' for lower power consumption~                                  |
| HWR1       | photoresistor shall be wired on `PA1` (ADC1), through a voltage divider using a 10k resistor, powered by `PA0`            |
| HWR2       | Presence sensor signal shall be wired on `PA3` (PCINT3)                                                                  |
| HWR3       | Battery voltage shall be measured without a pin, by reading the internal 1.1 V bandgap against Vcc (= VBAT)               |
| HWR4       | The DC/DC step-up shall only power the LEDs, and be switched off (`PB0` HIGH) when the LEDs are off                       |
| HWR5       | 433Mhz emmitter shall be wired on `PA2`                                                                                  |
| HWR6       | The three leds outputs are RED `PA7` (OC0B), Green `PA6` (OC1A), Blue `PA5` (OC1B)                                       |
| HWR7       | ~Leds shall be driver by transistor/MOSFET rather that pins to avoid too high power draw~                                 |
| HWR8       | Push button shall be wired on `PB2` (PCINT10, wakes from power-down)                                                     |

# Non functional requirements

| Identifier | Description                                                               |
| ---------- | ------------------------------------------------------------------------- |
| NFR1       | ~PSML runs on battery and can be charged using micro-USB socket~            |
| NFR2       | ~PSML does not start if battery voltage is too low to avoid deep discharge~ |

# Constants

| Name                   | Value | Description                                         |
| ---------------------- | ----- | ----------------------------------------------------|
| AMBIANT_LIGHT_TRESHOLD | 25 (ADC counts, ratiometric to Vcc) | PSML only wakes up under that light ADC measurement |
| LOW_POWER_MV           | 3000mV  | PSML blinks and does not start normally under this battery voltage|

