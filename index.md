---
layout: default
---

# What is the ESP32 Command Station?
ESP32 Command Station is an open-source hardware and software Command Station for the operation of DCC-equipped model railroads.

The ESP32 Command Station consists of an ESP32 micro controller connected to at least one Motor Shield that can be connected directly to the tracks of a model railroad.

## ESP32 Command Station Features

* Supports almost any variant of the ESP32.
* Built-in configuration for both the original Arduino Motor Shield, Pololu MC33926 Motor Shield or BTS7960B based Motor Shield.
* Built-in web interface for controlling locomotives or configuring the ESP32 Command Station.
* Built-in support for LCC either via WiFi or a hardware CAN transceiver, please see the section below on LCC if you intend on using this feature.
* Custom PCB with all required hardware "on-board" for LCC, DCC signal generation, RailCom detection and expansion ports (I2C and UART).

## Latest public release
The latest production release of the code is v1.2.3.

## Latest development updates
The development branch is where all "in-progress" work will be made available, there is no guarantee of stability of the project when using this version of the code.

## Supported ESP32 Boards
The ESP32 Command Station has been tested on a variety of ESP32 boards available on the internet, the current preferred format is either the Arduino Uno R3 formated boards (easy to use with the Arduino Motor Shield) or the TTGO T1 with the integrated SD Card. However, almost any variant of the ESP32 will work as long as there are enough pins available for the Motor Driver connections.

## Building ESP32 Command Station
Details on building the ESP32 Command Station can be found [here](./building-esp32cs.html)

-May 31, 2019