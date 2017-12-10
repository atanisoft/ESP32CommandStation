# DCCppESP32
DCC++ Base Station for the ESP32

## What is DCC++ESP32?
DCC++ESP32 is an open-source hardware and software system for the operation of DCC-equipped model railroads.

The DCC++ESP32 Base Station consists of an ESP32 micro controller connected to at least one Motor Shield that can be connected directly to the tracks of a model railroad.

## Whats in this Repository
This repository, DCCppESP32, contains a complete DCC++ESP32 Base Station code designed for compiling and uploading into an ESP32. All files are in the folder named src.

To utilize this code, simply download a zip file of this repository and open the included project file in PlatformIO IDE. No code modifications should be required *EXCEPT* for

The latest production release of the code is 1.0.0:
* Supports almost any variant of the ESP32.
* Built-in configuration for both the original Arduino Motor Shield, Pololu MC33926 Motor Shield or BTS7960B based Motor Shield.
* Built-in web interface for controlling locomotives or configuring the DCC++ESP32 Base Station.

Detailed diagrams showing connections to the supported Motor Shields, OLED or LCD screens can be found in the Wiki.

## Building DCC++ESP32
Building the DCC++ESP32 code is easiest using PlatformIO IDE which can be installed from http://platformio.org/platformio-ide. When installing PlatformIO IDE be sure to add Python to the default path (if prompted). Using Atom or VSCode does not make that much of a difference in compilation/usage of the included PlatformIO project file.

The only configuration that needs to be done prior to compilation/upload is in Config.h:
* WIFI_SSID - This must be configured prior to compilation/upload. This should be the name of the WiFi network to join.
* WIFI_PASSWORD - This must be configured prior to compilation/upload. This should be the password of the WiFi network to join.

The following parameters in Config.h are optional but are useful to enable if you have the required hardware:
* INFO_SCREEN_OLED - This can be enabled to use an I2C connected OLED display using either the SH1106 or SSD1306 chipset. If this is enabled the following parameters must also be defined: OLED_CHIPSET, INFO_SCREEN_OLED_I2C_ADDRESS, INFO_SCREEN_OLED_VERTICAL_FLIP.
* INFO_SCREEN_LCD - This can be enabled to use an I2C connected LCD display. IF this option is enabled INFO_SCREEN_OLED can not be used. If this is enabled the following parameters must also be defined: INFO_SCREEN_LCD_I2C_ADDRESS, INFO_SCREEN_LCD_LINES, INFO_SCREEN_LCD_COLUMNS.

-December 10, 2017
