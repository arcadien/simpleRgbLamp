# Power saving multicolor light

A battery-powered RGB lamp able to detect movement and ambiant light to adapt its behavior.

_The behaviour is simple : Once it starts, the first color is red. A push on the button will cycle the color order. The colors are: red, green, blue, pink, orange. After orange, a button press triggers deep sleep.
When in deep sleep, lamp can be wake up by button press or movement detection. If ambiant luminosity is too high, the lamp emits a quick blue blink and shut down. 
If the ambiant luminosity is low, the lamp remain on selected color until no more movement is detected long enough._

The repository contains:

- An Arduino sketch to drive the lamp
- The [Kicad](http://kicad.org/) schema
- A [VeroRoute](https://sourceforge.net/projects/veroroute/) implantation layout (_outdated: predates the current schematic_)

# Software

## High level requirements

| Identifier             | Description                                                                                               |
| ---------------------- | --------------------------------------------------------------------------------------------------------- |
| [HLR1](../../issues/1) | ~PSML button allow to change the color in a set of 5 colors (RED, GREEN, BLUE, ORANGE, PINK)~             |
| [HLR2](../../issues/2) | ~PSML detects ambient light and does not power up if luminosity is more than AMBIANT_LIGHT_TRESHOLD mv~   |
| [HLR3](../../issues/3) | ~PSML wakes up when it detects movement around, and go back to deep sleeps when movement detection ends~  |
| [HLR4](../../issues/4) | ~PSML button allow to change the color in a set of 5 colors (RED, GREEN, BLUE, ORANGE, PINK)~             |
| [HLR5](../../issues/5) | PSML transmit its remaning battery level wirelessly, each 15 minutes                                      |
| [HLR6](../../issues/6) | PSML transmit light intensity wirelessly, each 15 minutes                                                 |
| [HLR7](../../issues/7) | Long press (2 seconds) on push button toggles permanent light mode                                        |

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
- A MT3608 step-up module provides 5 V for the LEDs only; an AO3401A P-MOSFET cuts its input when the LEDs are off
- Light sensor is a simple photoresistor GL5528 (10–20 kΩ at 10 lux, ≥1 MΩ dark), powered from `PA0` only while measuring, with a 10k pull-down to GND
- Presence sensor is an AM312 mini PIR (2.7–12 V)
- 433Mhz transmitter is a FS1000A module
- LEDs are switched by IRLZ44N logic-level MOSFETs (common anode strip)
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

