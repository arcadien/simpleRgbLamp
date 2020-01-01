# Simple Rgb Lamp
A RGB lamp with a single control button. Created to reuse a spare RGB led strip.
The behaviour is simple : Once it starts, the first color is red. A push on the button will cycle the color order. The colors are: red, green, blue, pink, orange and deep sleep. 
When in deep sleep, the power consumption is minimized to some µA (the Pro mini is powered directly on the VCC pin, so there is no DC regulator consumption).
When user push the button again, the firmware wakes up and color cycle stars with red again.

The repository contains:
* An Arduino sketch to drive the lamp (the actual target is a Atmel ATmega328P on an Arduino pro mini)
* The [Kicad](http://kicad-pcb.org/) schema 
* A [VeroRoute](https://sourceforge.net/projects/veroroute/) implantation layout
