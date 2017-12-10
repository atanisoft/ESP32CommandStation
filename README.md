# DCCppESP32
DCC++ Base Station for the ESP32

## What is DCC++ESP32?
DCC++ESP32 is an open-source hardware and software system for the operation of
DCC-equipped model railroads.

The DCC++ESP32 Base Station consists of an ESP32 micro controller connected to
at least one Motor Shield that can be connected directly to the tracks of a
model railroad.

## Whats in this Repository
-------------------------
This repository, DCCppESP32, contains a complete DCC++ESP32 Base Station code
designed for compiling and uploading into an ESP32. All files are in the folder
named src.

To utilize this code, simply download a zip file of this repository and open the
included project file in PlatformIO IDE. No code modifications should be
required *EXCEPT* for

The latest production release of the code is 1.0.0:
* Supports almost any variant of the ESP32.
* Built-in configuration for both the original Arduino Motor Shield, Pololu
MC33926 Motor Shield or BTS7960B based Motor Shield.
* Built-in web interface for controlling locomotives or configuring the
DCC++ESP32 Base Station.

Detailed diagrams showing connections to the supported Motor Shields can be
found in the Wiki.

-December 10, 2017
