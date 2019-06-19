---
layout: default
---

# S88 Module
The S88 module allows reading of multiple S88 sensor buses. Each bus will have a unique data pin but shares the clock, reset and load pins.

## Configuration

| PARAM | Description |
| ----- | ----------- |
| S88_CLOCK_PIN | This is the ESP32 pin that is connected to the S88 Bus Clock line. |
| S88_RESET_PIN | This is the ESP32 pin that is connected to the S88 Bus Reset line. |
| S88_LOAD_PIN | This is the ESP32 pin that is connected to the S88 Bus Load line. |

All buses will share the above three lines but have a unique data line connected to individual pins on the ESP32. The buses will need to be configured through the web interface after startup.

[Return to Building ESP32 Command Station](./building-esp32cs.html)<br/>
[Return to ESP32 Command Station](./index.html)