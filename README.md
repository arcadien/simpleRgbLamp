# Power saving multicolor light

A battery-powered RGB lamp able to detect movement and ambiant light to adapt its behavior.

_The behaviour is simple : Once it starts, the first color is red. A push on the button will cycle the color order. The colors are: red, green, blue, pink, orange. After orange, a button press triggers deep sleep.
When in deep sleep, lamp can be wake up by button press or movement detection. If ambiant luminosity is too high, the lamp emits a quick blue blink and shut down. 
If the ambiant luminosity is low, the lamp remain on selected color until no more movement is detected long enough._

The repository contains:

- An Arduino sketch to drive the lamp
- The [Kicad](http://kicad.org/) schema
- A [VeroRoute](https://sourceforge.net/projects/veroroute/) implantation layout

# Software

## High level requirements

| Identifier             | Description                                                                                               |
| ---------------------- | --------------------------------------------------------------------------------------------------------- |
| [HLR1](../../issues/1) | PSML button allow to change the color in a set of 5 colors (RED, GREEN, BLUE, ORANGE, PINK)               |
| [HLR2](../../issues/2) | PSML detects ambient light and does not power up if luminosity is more than TBD lux                       |
| [HLR3](../../issues/3) | PSML wakes up when it detects movement around, and go back to deep sleeps after 5 minutes of non-movement |
| [HLR4](../../issues/4) | PSML button allow to change the color in a set of 5 colors (RED, GREEN, BLUE, ORANGE, PINK)               |
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
                     |        |--> RGB LED
 Light Sensor ------>|  PSML  |
                     |        |--> 433Mhz transmitter
 Push button  ------>|        |
                     |--------|

```

- Battery is a 18650
- Light sensor is a simple photoresistor (LDR)
- Presence sensor is a [HC SR501](https://www.makerguides.com/hc-sr501-arduino-tutorial/)
- Push button is available from outside enclosing device
- Rogers itself will be implemented on a [Arduino Pro Mini](https://docs.arduino.cc/retired/boards/arduino-pro-mini)

| Identifier | Description                                                                                                             |
| ---------- | ----------------------------------------------------------------------------------------------------------------------- |
| HWR0       | Unless specified, all pins shall be set as 'input/pull up' for lower power consumption                                  |
| HWR1       | photoresistor shall be wired on `PC0 (A0)`, configured as analog input, throught a voltage divider using a 10k resistor |
| HWR2       | Presence sensor signal shall be wired on `PC4` (Arduino Pro mini A4)                                                    |
| HWR3       | Battery (VBatt) shall be wired on Presence sensor signal is wired on `PC1` (Arduino Pro mini A1)                        |
| HWR4       | Battery shall be wired on a DC/DC converter so that Vcc is always 5v                                                    |
| HWR5       | 433Mhz emmitter shall be wired on `PB0 (8)` pin                                                                         |
| HWR6       | The three leds outputs are RED `PB1` (Arduino Pro mini 9), Green `PD1` (Arduino Pro mini 3), Blue ` PD6`  (Arduino Pro mini 6)                                            |
| HWR7       | Leds shall be driver by transistor/MOSFET rather that pins to avoid too high power draw                                 |
| HWR8       | Push button shall be wired on `PD2` (Arduino Pro mini 2)                                                                |

# Non functional requirements

| Identifier | Description                                                               |
| ---------- | ------------------------------------------------------------------------- |
| NFR1       | PSML runs on battery and can be charged using micro-USB socket            |
| NFR2       | PSML does not start if battery voltage is too low to avoid deep discharge |

# Constants

| Name                   | Value | Description                                     |
| ---------------------- | ----- | ----------------------------------------------- |
| MAX_LIGHT_FOR_POWER_ON | TBD   | PSML only wakes up under that light measurement |
