---
layout: default
---

# LCD Module
The LCD module allows the command station to display general status information. The information displayed on the screen is split into two types:
1. Static information (Command Station IP address, version information)
2. Dynamic information (free heap, active loco count, track current utilization, LocoNet status, LCC status)

With a two line display the first line will be used for static information and the second line for the dynamic data rotating between data roughly five seconds.
With a four line display the first and second lines are used for the static information, third for track current utilization and fourth for the remaining dynamic data.

## Configuration

| PARAM | Description |
| ----- | ----------- |
| INFO_SCREEN_LCD_I2C_ADDRESS | This is the I2C address of the LCD screen, often it is 0x27. |
| INFO_SCREEN_LCD_LINES | This is the number of lines the LCD screen can display. Currently only 2 or 4 lines are supported. |
| INFO_SCREEN_LCD_COLUMNS | This is the number of columns the LCD screen can display. Currently only 16 or 20 column displays are supported. |

[Return to Building ESP32 Command Station](./building-esp32cs.html)<br/>
[Return to ESP32 Command Station](./index.html)