# DCCppESP32
DCC++ Base Station for the ESP32

## What is DCC++ESP32?
DCC++ESP32 is an open-source hardware and software system for the operation of DCC-equipped model railroads.

The DCC++ESP32 Base Station consists of an ESP32 micro controller connected to at least one Motor Shield that can be connected directly to the tracks of a model railroad.

## Whats in this Repository
This repository, DCCppESP32, contains a complete DCC++ESP32 Base Station code designed for compiling and uploading into an ESP32. All source files are in the folder named src and header files are under include (including index_html.h which is generated during the build in PlatformIO IDE).

To utilize this code, simply download a zip file of this repository and open the included project file in PlatformIO IDE, and the dependent libraries listed under the "lib" folder (see the readme.txt file for details). No code modifications should be required *EXCEPT* for

The latest production release of the code is 1.2.0:
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

## Supported ESP32 Boards
The DCC++ESP32 Base Station has been tested on a variety of ESP32 boards available on the internet, the current preferred format is either the Arduino Uno R3 formated boards (easy to use with the Arduino Motor Shield) or the TTGO T1 with the integrated SD Card. However, almost any variant of the ESP32 will work as long as there are enough pins available for the Motor Driver connections.

## Supported Motor Drivers/Boards
The DCC++ESP32 Base Station currently supports three types of Motor Driver/Boards.
* Arduino Motor Shield Rev3 (https://store.arduino.cc/usa/arduino-motor-shield-rev3). There are various clones of this board avialable throughout the internet, many of these clones will work but many will not.
* Pololu MC33926 Motor Driver (https://www.pololu.com/product/1213 or https://www.pololu.com/product/2503). There are a few variants on this board available from Pololu and they all should function identically. It is not necessary to have the Arduino shield format and all the majority of the testing has been carried out using the carrier format.
* BTS 7960B. This is a *VERY* high current H-Bridge based circuit, in the DCC++ESP32 Base Station code it is software limited to either 5A or 10A.

-October 31, 2018
