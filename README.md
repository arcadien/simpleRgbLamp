# Power saving multicolor light

A multi-mode RGB lamp, which:

- is able to display various colors, selectable by round robin selection using a single button
- has a movement detection mode, where the light stays on for 5 minutes when movement is detected
- has a forced night mode, where the light is on for 6 hours
- does not power on if the ambient light is to high
- provide wireless information about its environment (light and battery) for domotic usage
- runs on battery

# Software

## High level requirements

| Identifier             | Description                                                                                                                  |
| ---------------------- | ---------------------------------------------------------------------------------------------------------------------------- |
| [HLR1](../../issues/2) | PSML button allow to change the color in a set of 5 colors (RED, GREEN, BLUE, ORANGE, PINK)                                  |
| [HLR2](../../issues/3) | PSML measure ambient light and does not power up if luminosity is more than MAX_LIGHT_FOR_POWER_ON                           |
| [HLR3](../../issues/4) | PSML wakes up when it detects movement around, and go back to deep sleeps after MOVEMENT_COOLDOWN_IN_MINUTES of non-movement |
| [HLR4](../../issues/5) | PSML button allow to change the color in a set of 5 colors (RED, GREEN, BLUE, ORANGE, PINK)                                  |
| [HLR5](../../issues/6) | PSML transmit its remaning battery level wirelessly, each EMIT_COOLDOWN_IN_MINUTES                                           |
| [HLR6](../../issues/7) | PSML transmit light intensity wirelessly, each EMIT_COOLDOWN_IN_MINUTES                                                      |
| [HLR7](../../issues/8) | Long press (2 seconds) on push button toggles permanent light mode                                                           |

## Architecture

### Component diagram

```puml {align="center"}
!theme superhero
skinparam component {
    backgroundColor<<hardSoft>> Green
    backgroundColor<<soft>> Grey
    backgroundColor<<softHard>> Orange
}

component "Arduino pro mini" {
  port "PC0 (A0)"
  port "PC1 (A1)"
  port "PC4 (18)"
  port "PB0 (8)"
  port "PB1 (9)"
  port "PD2 (2)"
  port "PD3 (3)"
  port "PD6 (6)"
}



package "PSML" {

 [AnalogReader] <<hardSoft>>
 [PushButton] <<hardSoft>>
 [LedsDriver] <<softHard>>
 [X10Encoder] <<softHard>>
 [PresenceSensor] <<hardSoft>>

 "PC0 (A0)" -down-> [AnalogReader]
 "PC1 (A1)" -down-> [AnalogReader]
 "PD2 (2)"  -down-> [PushButton]
 "PC4 (18)" -down-> [PresenceSensor]

[X10MessageFactory] <<soft>>
[StateMachine] <<soft>>

 AnalogReader -> X10MessageFactory : D_tension_in_mv
 AnalogReader -> X10MessageFactory : D_luminosity
 PushButton -> StateMachine : D_ButtonPressed
 PushButton -> StateMachine : D_ButtonLongPressed
 StateMachine -> LedsDriver : D_Color
 StateMachine -> X10Encoder : D_EmitData
 PresenceSensor -> StateMachine : D_presenceDetectionState

 [X10MessageFactory] -> [X10Encoder] : D_RadioMessage
 [LedsDriver] -down-> "PB1 (9)"
 [LedsDriver] -down-> "PD6 (6)"
 [LedsDriver] -down-> "PD3 (3)"
 [X10Encoder] -down-> "PB0 (8)"
}
```

### sequence diagram

```puml {align="center"}
start
while ()
    if (D_ButtonPressed?) then (yes)
        :nextColor();
    elseif(D_ButtonLongPressed?) then (yes)
        :switchMode();
    endif
    if(D_luminosity < MAX_LIGHT_FOR_POWER_ON) then (yes)
        if(D_presenceDetectionState == DETECTED) then (yes)
            : lightOn();
        else (no)
            : lightOff();
        endif
    else (no)
        : lightOff();
    endif

    if(D_EmitData?) then (yes)
        :readBatteryLevel();
        :readLightLevel();
        :generateMessage();
        :emitData();
    else (no)
    endif

    while ()
        :sleep();
    endwhile
endwhile
end
```

## Low level requirements

| Identifier              | Component | Description                                                                             |
| ----------------------- | --------- | --------------------------------------------------------------------------------------- |
| [LLR1](../../issues/9)  |           | PSML sense battery voltage using ADC (PC1) channel in millivolts                        |
| [LLR2](../../issues/10)  |           | PSML sense ambient light using ADC (PC1) channel in raw ADC value, compared against VCC |
| [LLR3](../../issues/11)  |           | PSML is awaken when `PB2` pin is driven low                                             |
| [LLR4](../../issues/12)  |           | PSML emits battery voltage in millivolt using X10Meter protocol                         |
| [LLR5](../../issues/13) |           | PSML emits total water consumption in liters using X10Meter protocol                    |
| [LLR6](../../issues/14) |           | PSML detects metal disc movement through the CNY70 wired on `PA0`                       |

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
| HWR2       | Presence sensor signal shall be wired on `PC4`                                                                          |
| HWR3       | Battery (VBatt) shall be wired on Presence sensor signal is wired on `PC1 (A1)`, configured as analog input             |
| HWR4       | Battery shall be wired on a DC/DC converter so that Vcc is always 5v                                                    |
| HWR5       | 433Mhz emmitter shall be wired on `PB0 (8)` pin                                                                         |
| HWR6       | The three leds outputs are RED `PB1 (9)`, Green `PD3 (3)` , Blue ` PD6 (6)`                                             |
| HWR7       | Leds shall be driver by transistor rather that pins to avoid too high power draw                                        |
| HWR8       | Push button shall be wired on `PD2 (2)`                                                                                 |

# Non functional requirements

| Identifier | Description                                                               |
| ---------- | ------------------------------------------------------------------------- |
| NFR1       | PSML runs on battery and can be charged using micro-USB socket            |
| NFR2       | PSML does not start if battery voltage is too low to avoid deep discharge |

# Constants

| Name                              | Value | Description                                                                    |
| --------------------------------- | ----- | ------------------------------------------------------------------------------ |
| MAX_LIGHT_FOR_POWER_ON            | TBD   | PSML only wakes up under that light measurement                                |
| MAX_SLEEP_DURATION_IN_S           | 8     | How many seconds can sleep the hardware at once                                |
| MOVEMENT_COOLDOWN_IN_MINUTES      | 5     | How many minutes after a movement detection to power off                       |
| NIGHT_MODE_IN_MINUTES             | 360   | How many minutes to stay on when in 'night mode'                               |
| EMIT_COOLDOWN_IN_MINUTES          | 900   | Duration between each wireless message emission                                |
| MOVEMENT_COOLDOWN_IN_SLEEP_PERIOD | 37    | How many MAX_SLEEP_DURATION_IN_S periods to sleep MOVEMENT_COOLDOWN_IN_MINUTES |
| NIGHT_MODE_IN_SLEEP_PERIOD        | 2700  | How many MAX_SLEEP_DURATION_IN_S periods to sleep 6 hours                      |
| EMIT_COOLDOWN_IN_SLEEP_PERIOD     | 111   | How many MAX_SLEEP_DURATION_IN_S periods to sleep 15 minutes                   |
