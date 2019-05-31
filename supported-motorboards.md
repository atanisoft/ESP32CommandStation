---
layout: default
---

# Supported Motor Boards
The ESP32 Command Station currently supports three types of Motor Boards.
* [Arduino Motor Shield Rev3](https://store.arduino.cc/usa/arduino-motor-shield-rev3). There are various clones of this board avialable throughout the internet, many of these clones will work but many will not.
* [Pololu MC33926 Motor Driver](https://www.pololu.com/product/2503) or [Pololu MC33926 Motor Driver Carrier](https://www.pololu.com/product/1213). There are a few variants on this board available from Pololu and they all should function identically. It is not necessary to have the Arduino shield format and all the majority of the testing has been carried out using the carrier format.
* BTS 7960B. This is a *VERY* high current H-Bridge based circuit, in the ESP32 Command Station code it is software limited to either 5A or 10A.
