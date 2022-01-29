# What is ESP32 Command Station?

ESP32 Command Station is an open-source Command Station for the operation of
OpenLCB (LCC) and DCC-equipped model railroads.

The ESP32 Command Station consists of an ESP32 micro controller connected to at
least one h-bridge that can be connected directly to the tracks of a model
railroad.

## Features

* Supports most variants of the ESP32 or ESP32-S3.
* Support for multiple h-bridge types (Arduino Motor Shield, Pololu MC33926 Motor Shield, BTS7960B, LMD18200, DRV8873, DRV8800/DRV8801).
* Built-in web interface for controlling locomotives or configuring the ESP32 Command Station.
* Built-in support for OpenLCB (LCC) either via WiFi or an external CAN transceiver.

## Supported Hardware

ESP32 Command Station supports many features that utilize built-in and external
hardware. Please see [Supported Hardware](supported_hardware.md) for details.

## Releases

* [![GitHub release](https://img.shields.io/github/release/atanisoft/ESP32CommandStation.svg?label=Stable&style=plastic)](https://github.com/atanisoft/ESP32CommandStation/releases) [![GitHub release date](https://img.shields.io/github/release-date/atanisoft/ESP32CommandStation.svg?style=plastic)](https://github.com/atanisoft/ESP32CommandStation/releases)
* [![GitHub release](https://img.shields.io/github/release-pre/atanisoft/ESP32CommandStation.svg?label=Unstable&style=plastic)](https://github.com/atanisoft/ESP32CommandStation/releases) [![GitHub release date](https://img.shields.io/github/release-date-pre/atanisoft/ESP32CommandStation.svg?style=plastic)](https://github.com/atanisoft/ESP32CommandStation/releases)

## Building from source code

In most cases a pre-compiled binary will work but there are cases where
compiling from source code is necessary. If you have not previously configured
an ESP-IDF build environment please follow [this](build_env.md) guide.

Otherwise proceed to [Configuring build options](configuring.md).

## Pre-compiled binaries

[link](precompiled.md).