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

## Connecting the CS to the S88 bus

Each S88 bus requires four pins from the command station, the three listed above and a fourth for the data pin. Connect each pin as described below:

| CS PIN | S88 PIN | S88-n PIN (RJ45) |
| ------ | ------- | ---------------- |
| DATA | 1 | 2 (green) |
| GND | 2 | 3 (white/orange) |
| GND | n/a | 5 (white/blue) |
| S88_CLOCK_PIN | 3 | 4 (blue) |
| S88_LOAD_PIN | 4 | 6 (orange) |
| S88_RESET_PIN | 5 | 7 (white/brown) |
| 5V/12V | 6 | 1 (white/green) |

Note: The DATA pin above is defined per bus. Similarly the 5V/12V pin should *NOT* be connected to the ESP32 module but instead to a dedicated power supply for the S88 bus. The ESP32 5V pin is connected to the voltage regulator on the ESP32 and will not be able to supply sufficient power for both the ESP32 and other devices.

## Additional details about S88/S88-n

Additional details about S88/S88-n can be found [here](https://www.opendcc.de/s88/s88_n/s88-n_e.html).

[Return to Building ESP32 Command Station](./building-esp32cs.html)<br/>
[Return to ESP32 Command Station](./index.html)
