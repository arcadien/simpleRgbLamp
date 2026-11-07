# simpleRgbLamp

Battery-powered RGB lamp on an Arduino Pro Mini (5 V / 16 MHz). See README.md.

- `sketch/` — PlatformIO firmware. **The firmware is the source of truth for pin assignments**; schematic and README follow it.
- `kicad/` — KiCad schematic (project symbols in `kicad/LedLamp.kicad_sym`).
- `veroroute/` — stripboard layout (VeroRoute), edited by hand.

## Hardware conventions

### Wire colors
Power goes in pairs (thick wire, 26 AWG for TARS-size builds):
| Pair (+ / −)  | Use |
|---------------|-----|
| red / black   | battery and 5V: B+/B−, VBAT, switched battery rail, VUSB |
| white / blue  | regulated or stepped voltages: 3V3, buck/boost outputs (never yellow) |
| grey          | speaker |

Signals (thin wire, 30 AWG), by role:
| Color  | Use |
|--------|-----|
| yellow | clocks: SCK, BCLK, SCL, SD_CLK |
| green  | data: MOSI, DIN, SDA, SD_D0 |
| purple | select / frame: CS, LRC/WS, SD_CMD |
| red    | status: DC, interrupts, PIR out |
| orange | reset, detect (RST, USB_DET) |
| brown  | enables / analog: BL_EN, VBAT/2, *_SD, *_EN |

The kit has 10 colors: red, black, white, blue, yellow, green, purple, orange, brown, grey. **Blue is reserved for power** (regulated GND) and never carries a signal.

When several buses share colors, tell them apart with 1 / 2 / 3 heat-shrink rings at each end (TARS: 1 = I2S, 2 = I2C, 3 = microSD).

### Wire ends
Label every wire end as **soldered**, **Dupont** or **JST-XH** (with F/M contact), on both ends, and give each wire's length, in every doc and diagram.

### Resistors
Current-limiting and pull resistors sit **at the board end**, never at the LED or the sensor.

### Images
Strip EXIF/GPS from every image before committing:
`python3 ~/.claude/skills/hardware-change/scripts/strip_exif.py <paths>`
(`--check` to verify).

### Commits
No pre-push hook in this repo. Use the conventional prefixes already in the history: `feat:`, `fix:`, `docs:`. Make granular commits, one per area, and push after each.
