---
layout: default
---

# Building the ESP32 Command Station code
Building the ESP32 Command Station code is only possible by using PlatformIO IDE which can be installed from http://platformio.org/platformio-ide. When installing PlatformIO IDE be sure to add Python to the default path (if prompted). Using [Atom](https://atom.io/) or [Visual Studio Code](https://code.visualstudio.com/) does not make that much of a difference in compilation/usage of the included PlatformIO project file. Currently all development and testing is conducted with [Visual Studio Code](https://code.visualstudio.com/).

The ESP32 Command Station consists of multiple modules, most of which are optional. The list below covers the various modules:

| Module | Description |
| ------ | ----------- |
| [WiFi](./config-wifi.html) (Required) | This is a required module as it makes the ESP32 Command Station accessible to JMRI, throttles or WiFi enabled devices via the web interface. |
| [MotorBoard](./config-motorboard.html) (Required) | This is a required module as it is used to generate the DCC signal supplied to the track. |
| [OLED](./config-oled.html) or [LCD](./config-lcd.html) | These are optional modules, only one can be included at a time. This module provides support for displaying runtime statistics and state information (IP Address, track power, track power utilization, etc) |
| [LocoNet](./config-loconet.html) | This is an optional module that provides capability for the Command Station to interface with LocoNet devices. |
| [Nextion](./config-nextion.html) | This is an optional module that provides a touch screen interface with a throttle and a few extra features for accessories. |
| [S88](./config-s88.html) | This is an optional module that allows the Command Station to poll one (or more) S88 busses as sensor inputs. |
| [LCC](./config-lcc.html) | This is an optional module that allows the Command Station to interface with other LCC devices either through a physical CAN bus or through WiFi connections to a LCC HUB device. |

[Return to ESP32 Command Station](./index.html)