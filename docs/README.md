# What is ESP32 Command Station?

ESP32 Command Station is an open-source NMRA compliant DCC Command Station for
the operation of DCC-equipped model railroads with integrated support for using
RailCom and OpenLCB (LCC).

Please refer to [Supported Hardware](supported_hardware.md) for what hardware
can be used with this software.

## Features

* Supports most variants of the ESP32 or ESP32-S3.
* Support for multiple h-bridge types (Arduino Motor Shield, Pololu MC33926 Motor Shield, BTS7960B, LMD18200, DRV8873, DRV8800/DRV8801).
* Built-in web interface for controlling locomotives or configuring the ESP32 Command Station.
* Built-in support for OpenLCB (LCC) either via WiFi or an external CAN transceiver.
* Built-in support for generating RailCom cut-out and receiving RailCom feedback.

## Development Status

| Release Type | Label | Date |
| ------------ | ----- | ---- |
| Stable | [![GitHub release](https://img.shields.io/github/release/atanisoft/ESP32CommandStation.svg?label=Stable&style=plastic)](https://github.com/atanisoft/ESP32CommandStation/releases) | [![GitHub release date](https://img.shields.io/github/release-date/atanisoft/ESP32CommandStation.svg?style=plastic)](https://github.com/atanisoft/ESP32CommandStation/releases) |
| Preview | [![GitHub release](https://img.shields.io/github/release-pre/atanisoft/ESP32CommandStation.svg?label=Preview&style=plastic)](https://github.com/atanisoft/ESP32CommandStation/releases) | [![GitHub release date](https://img.shields.io/github/release-date-pre/atanisoft/ESP32CommandStation.svg?style=plastic)](https://github.com/atanisoft/ESP32CommandStation/releases) |

## Using this software

This software is provided in source code form and will require compilation
using the [Espressif IDF](https://github.com/espressif/esp-idf) Framework.
[Pre-compiled](precompiled.md) binaries are available for common configurations
as part of releases starting with v2.0.0.

If a customized version of the firmware is required, it will be necessary to
setup the [Espressif IDF](https://github.com/espressif/esp-idf) build
environment by following [these](build_env.md) instructions. After the build
env has been configured proceed to [Configuring](configuring.md).

### After flashing, what's next?

After this software has been flashed to the ESP32 you should be able to connect
the track connections and supply power to the ESP32 (and h-bridge). The
software will automatically start when the ESP32 is powered.

Proceed to [Getting Started](getting_started.md) for instructions on how to use
this software.