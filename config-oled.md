---
layout: default
---

# OLED Configuration
If the ESP32 board has a built in OLED screen it may automatically be usable, this depends on the board being configured correctly in the [arduino-esp32](https://github.com/espressif/arduino-esp32) release. Currently only ome of the Heltec (see note below), TTGO or D-Duino-32 boards provide the automatic configuration. If the OLED does not work out-of-box with these boards or if your board does not define OLED_SDA and OLED_SCL, you can configure it via the parameters below.

| PARAM | Description |
| ----- | ----------- |
| OLED_CHIPSET | This configures how the Command Station will talk to the OLED screen, currently only SH1106 and SSD1306 chips are supported. |
| INFO_SCREEN_OLED_I2C_ADDRESS | This is the I2C address of the OLED screen, often it is 0x3C or 0x27. If the OLED screen is not detected an I2C scan is performed on startup and addresses that respond are printed to the console. |
| INFO_SCREEN_OLED_VERTICAL_FLIP | If you find that your OLED screen is displaying text upside down, you may need to enable or disable this by setting it to true or false. |
| INFO_SCREEN_OLED_LINES | This is the number of lines that the OLED screen can display, the font used is 13 pixels tall and is configured for a 128x64 OLED display. |
| INFO_SCREEN_SDA_PIN | If your ESP32 does not use a standard SDA pin (defined in the pins_arduino.h from arduino-esp32) you can define it here. |
| INFO_SCREEN_SCL_PIN | If your ESP32 does not use a standard SCL pin (defined in the pins_arduino.h from arduino-esp32) you can define it here. |
| INFO_SCREEN_RESET_PIN | If your OLED screen requires a reset pin to be used you can enable it by defining this parameter. Only a few OLED displays require this option and they are often attached to the ESP32 board directly in which case this option should automatically be enabled with the correct board type being selected. |

##### Heltec WiFi Kit 32 / Heltec WiFi Lora 32 Configuration
These boards have on-board OLED screens, if the automatic configuration does not work for these boards try the following settings:

| PARAM | Description |
| ----- | ----------- |
| OLED_CHIPSET | SSD1306 |
| INFO_SCREEN_OLED_I2C_ADDRESS | 0x3C |
| INFO_SCREEN_SDA_PIN | 4 |
| INFO_SCREEN_SCL_PIN | 15 |
| INFO_SCREEN_RESET_PIN | 16 |

All other parameters can be left as their defaults.

[Return to Building ESP32 Command Station](./building-esp32cs.html)