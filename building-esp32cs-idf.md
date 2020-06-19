---
layout: default
---

# Overview
The ESP32 Command Station code is built on top of the ESP-IDF framework.

## Setting up ESP-IDF build environment
ESP-IDF is a required component and unfortunately there are only a few options for setting up the build environment. For all options it is required to have Python3 and the Git client installed and available in the path.

### VSCode "Espressif IDF" extension
This is a very easy way to install all components and configure them for building. There are a few bugs in the plugin still but overall it will work.

### Command line building on Windows
The easiest way to install the dependencies is to use [this](https://dl.espressif.com/dl/esp-idf-tools-setup-2.3.exe) installer to download/configure the ESP-IDF tools. When running this installer be sure to select "release/v4.0" as the version to install.

### Command line building on Linux / Mac OS
It is recommended to follow steps 1-4 from [this](https://docs.espressif.com/projects/esp-idf/en/release-v4.0/get-started/index.html#installation-step-by-step) page.

## Configuring ESP32 Command Station
If you are using the command line build tools you should refer to running:
~~~~
  idf.py menuconfig
~~~~
from the ESP32 Command Station directory.

If you are using the VSCode extension click on the gear ( :gear: ) icon in the bottom status line to use the graphical menuconfig tool (warning, this tool may be unstable!)

### Configuration

| Component | Description |
| --------- | ----------- |
| [LCC Configuration](#layout-command-control-configuration) | Layout Command Control, an NMRA standard, that allows compatible devices to interact on a dedicated bus seperate from the DCC bus. |
| [WiFi Configuration](#wifi-configuration) | Configures the WiFi settings for the ESP32 Command Station. |
| [DCC Signal Configuration](#dcc-signal-configuration) | Configures the DCC Signal Generation. |
| [GPIO](#gpio-configuration) | Configures optional GPIO support (including S88). |
| [JMRI DCC++ Interface](#jmri-dcc-interface) | Configures optional support for DCC++ interface to be used by JMRI.<br/>NOTE: The JMRI interface is a legacy interface and may be removed in the future. |
| [Status Display](#status-display-configuration) | Configures an optional OLED/LCD display. |
| [Status LED](#status-led-configuration) | Configures addressable LEDs for optional status indicators. |

#### Layout Command Control Configuration
Layout Command Control, an NMRA standard, that allows compatible devices to interact on a dedicated bus seperate from the DCC bus. Common usages of this would be for track side signals and block detection. The ESP32 Command Station can connect with LCC devices either via WiFi or physical CAN bus connections. Details about LCC can be found here.

| PARAM | Default | Description |
| ----- | ------- | ----------- |
| Node ID | 05.01.01.01.3F.00 | This is the unique 64bit node ID for the ESP32 Command Station.<br/>Note: It is recommended to use an ID from your unique ID range which can be reserved [here](https://registry.openlcb.org/requestuniqueidrange). |
| CAN interface | false | This option will enable the usage of a hardware CAN connection.<br/>Note: this will require an external CAN transceiver such as SN6565HVD23x or MCP2551. |
| CAN RX Pin | not provided | This is the pin connected to the CAN transceiver RX pin. |
| CAN TX Pin | not provided | This is the pin connected to the CAN transceiver TX pin. |
| Print all packets | false | When this is enabled all LCC packets will be printed to the console.<br/>Note: this is not recommended for use on the layout. |
| Add newline to GridConnect packets | false | When this is enabled all LCC packets will be transmitted with a newline appended to them.<br/>Note: This is useful for debugging but is not necessary for normal layout operations. |
| Perform LCC Factory reset on startup | false | When this is enabled the LCC configuration will be cleared on startup.<br/>Note: this is not recommended for use on the layout. |
| MemoryConfig space limit | 10 | This controls the maximum number of MemoryConfig spaces that will be available to the LCC Stack.<br/>Note: This normally will not require modification. |
| StateFlows to execute between select() calls | 60 | This controls how many StateFlows will be executed between checks for pending data.<br/>Note: This normally will not require modification. |
| Number of 'local' LCC nodes | 30 | This setting controls how many local nodes will be supported by the LCC stack.<br/>Note: This normally will not require modification. |
| Automatic fsync() interval (seconds) | 10 | When an SD card is used for persistence it is necessary to flush the configuration data to the SD card periodically. This controls the flush interval.<br/>Note: This normally will not require modification. |
| GridConnect packet delay (microseconds) | 500 (1500 with select() enabled) | This controls how long the LCC stack will buffer outgoing GridConnect packets before flushing them to the socket.<br/>Note: This normally will not require modification. |
| GridConnect packet count limit | 2 | This controls how many outbound GridConnect packets to allow before rejecting additional packets.<br/>Note: This normally will not require modification. |
| Use select() for TCP/IP connections | false | Enabling this option enables the LCC stack to monitor TCP/IP sockets for readiness before reading or writing data.<br/>Note: This may increase performance of the LCC stack but may cause some instability. |

[Return to Configuration](#configuration)

#### WiFi Configuration
The ESP32 Command Station supports connecting to an AP (Access Point) or advertising itself as an AP, or even both simultaneously.

| PARAM | Default | Description |
| ----- | ------- | ----------- |
| Hostname prefix | esp32cs_ | This is used to generate a unique hostname for the ESP32 Command Station.<br/>Note: the LCC Node ID will be appended to this value. |
| WiFi mode | Connect to SSID | This controls the operating mode of the WiFi stack in the ESP32 Command Station.<br/>1. Connect to SSID - Connect to an existing AP<br/>2. Create SoftAP - Advertises the ESP32 as an AP.<br/>3. Connect to SSID and create SoftAP - Combines the above two options. |
| SoftAP SSID | esp32cs | Name of the AP to advertise when "Create SoftAP" is enabled. |
| SoftAP Password | esp32cs | Password for connecting to the AP to advertise when "Create SoftAP" is enabled. |
| SSID | not provided | This is the SSID to connect to when operating in "Connect to SSID" or "Connect to SSID and create SoftAP" mode. |
| Password | not provided | This is the password for the SSID when operating in "Connect to SSID" or "Connect to SSID and create SoftAP" mode. |
| WiFi IP (type) | DHCP | Both DHCP and Static IP are supported by the ESP32 Command Station. |
| IP address | 10.0.0.155 | Static IP to assign to the ESP32 Command Station when operating in "Static IP" mode. |
| Gateway IP address | 10.0.0.1 | Gateway IP to use when operating in "Static IP" mode. |
| Subnet mask | 255.255.255.0 | Subnet mask to use when operating in "Static IP" mode. |
| Primary DNS address | 8.8.8.8 | DNS server to use when operating in "Static IP" mode. |

[Return to Configuration](#configuration)

#### DCC Signal Configuration
The DCC Signal Generator used by the ESP32 Command Station leverages the built-in RMT peripheral to generate the square wave DCC signal with variable pulse timing. The ESP32 has eight RMT channels available and the DCC Signal Generator will use two of them with up to three memory blocks each (eight available). It is not recommended to use the RMT peripheral outside of the DCC Signal Generator or the Status LED.

The configuration for the DCC Signal Generator is split into a couple sections:
* [OPS Track Configuration](#ops-track-configuration)
* [PROG Track Configuration](#prog-track-configuration)
* [Advanced Configuration](#advanced-configuration)

##### OPS Track Configuration

| PARAM | Default | Description |
| ----- | ------- | ----------- |
| Name | OPS | This is the displayed name for the OPS track output. |
| Energize track upon startup | false | Enabling this option will turn on the track output for the OPS track upon startup. |
| H-Bridge type | L298 | This determines which h-bridge profile is used by the DCC Signal Generator for the OPS track output.<br/>[Supported H-Bridges](#supported-h-bridges) |
| H-Bridge enable/pwm pin | 25 | This pin is used to enable/disable the h-bridge. |
| H-Bridge signal/direction pin | 19 | This pin is used to switch the direction output of the h-bridge. |
| H-Bridge current sense pin | ADC1:0 / 36 | This pin is used for current sense / short detection.<br/>[ADC channels and pin mappings](#adc-channel-mappings) |
| LMD18200 Thermal Alert pin | not provided | This pin is only available for the LMD18200 h-bridge and should be connected to the thermal alert pin of the LMD18200. |
| Enable RailCom detector | false | Enabling this option will allow receiving of RailCom data from the layout. |
| RailCom detector enable pin | not provided | This pin is used to enable the RailCom receiver circuitry. |
| H-Bridge brake pin | not provided | This is used to put the h-bridge into "coast" mode and is only available for the LMD18200 h-bridge. |
| RailCom UART | UART1 | This controls which hardware UART will be used to receive RailCom data. |
| RailCom UART RX pin | not provided | This pin should be connected to the RailCom data output pin on the receiver circuit. |
| Display all RailCom packets as they are received | false | Enabling this option will cause all received RailCom data to be printed to the console.<br/>Note: this is not recommended for use on the layout and should only be used for debugging. |
| DCC packet preamble bits | 11 (RailCom disabled)<br/>16 (RailCom enabled) | This controls how many preamble bits will be transmitted as part of each DCC packet. |
| Number of packets to queue for the OPS track | 5 | This is how many DCC packets will be available to transmit in the queue. |

[Return to DCC Signal Configuration](#dcc-signal-configuration)<br/>[Return to Configuration](#configuration)

##### PROG Track Configuration

| PARAM | Default | Description |
| ----- | ------- | ----------- |
| Name | PROG | This is the displayed name for the PROG track output. |
| H-Bridge type | L298 | This determines which h-bridge profile is used by the DCC Signal Generator for the PROG track output.<br/>Note: [Supported H-Bridges](#supported-h-bridges) |
| H-Bridge enable/pwm pin | 23 | This pin is used to enable/disable the h-bridge. |
| H-Bridge signal/direction pin | 18 | This pin is used to switch the direction output of the h-bridge. |
| H-Bridge current sense pin | ADC1:3 / 39 | This pin is used for current sense / short detection.<br/>[ADC channels and pin mappings](#adc-pins) |
| LMD18200 Thermal Alert pin | not provided | This pin is only available for the LMD18200 h-bridge and should be connected to the thermal alert pin of the LMD18200. |
| DCC packet preamble bits | 22 | This controls how many preamble bits will be transmitted as part of each DCC packet. |
| Number of packets to queue for the PROG track | 5 | This is how many DCC packets will be available to transmit in the queue. |

[Return to DCC Signal Configuration](#dcc-signal-configuration)<br/>[Return to Configuration](#configuration)

##### Advanced Configuration
These configuration options typically do not require modifications but are made available for debugging or special situations.

| PARAM | Default | Description |
| ----- | ------- | ----------- |
| ADC attenuation | 11 dB | This controls the current sense supported voltage and sensitivity. |
| Maximum number of DCC packets to queue | 5 | This controls how many packets to support in the global packet queue.<br/>Note: This normally will not require modification. |
| Number of eStop packets to send before powering off track | 200 | This controls how many eStop packets will be broadcast before powering off the OPS track output.<br/>Note: This normally will not require modification. |
| DCC RMT logging | Minimal | This controls the verbosity of the logging by the OPS and PROG track signal code. |
| Usage report interval (seconds) | 30 seconds | This controls how often the track current usage will be reported to the console. |
| Number of consecutive over-current reads before shutdown | 3 | This controls how quickly an over-current condition will shutdown the h-bridge. |
| Generate HIGH,LOW signal | true | This controls the direction of the square wave generated by the RMT. |
| Use REF_TICK clock source (1Mhz) | false | This option switches from the APB_CLK (80Mhz) to the REF_CLK (1Mhz). This is primarily useful for slowing down the DCC signal to monitor the bits via LEDs. |
| RMT clock divider | 1 (REF_CLK)<br/>80 (APB_CLK) | This is the divider for the clock signal used to time the bit length of the DCC packets. |
| RMT ticks for DCC zero | 96 | This controls the length of each half of the square wave in number of RMT ticks. |
| RMT ticks for DCC one | 58 | This controls the length of each half of the square wave in number of RMT ticks. |

[Return to DCC Signal Configuration](#dcc-signal-configuration)<br/>[Return to Configuration](#configuration)

#### Supported H-bridges
The ESP32 Command Station supports multiple h-bridge types as shown below:
| Name | Max Current (Amps) | Current Limit (Amps) |
| ---- | ------------------ | -------------------- |
| [L298](#l298) | 2 Amp | 1.75 Amp |
| [LDM18200](#lmd18200) | 3 Amp | 2.75 Amp |
| [Pololu MC33926](#pololu-mc33926) | 2.5 Amp | 2.25 Amp |
| [BTS 7960B](#bts7960b) | 43 Amp | 5 Amp or 10 Amp |

###### L298
The L298 h-bridge is one of the most common available on the internet since it is used by the Arduino Motor Shield rev3 and it's clones. However, be cautious about some clones as some do not expose current sense and/or use the L293 instead of the L298.

The L298 h-bridge is one of the easiest to connect to the ESP32 Command Station since it is available as an Arduino Shield format that can be plugged in directly to an ESP32 Uno form factor board. A couple connections will need to be adjusted to use this h-bridge:
1. Disconnect the brake pins, this are typically traces on the underside of the shield but may also be exposed as jumper pins on the top side. This is necessary to ensure the outputs are not unintentionally disabled.
2. Disconnect the VIN pin, this is typically a trace on the underside of the shield but may also be exposed as a jumper on the top side. This must be disconnected to prevent the track voltage from being connected to the ESP32.
3. Add a wire jumper from A0 to A4 and from A1 to A5. The pins under A0 and A1 are not suitable for current sense on most boards.

[Return to Supported H-bridges](#supported-h-bridges)<br/>[Return to DCC Signal Configuration](#dcc-signal-configuration)<br/>[Return to Configuration](#configuration)

###### LMD18200
The LMD18200 h-bridge operates similar to the L298 but is only for a single track output and supports a higher amperage. This h-bridge offers a few advantages over other h-bridges:
1. Inline current sense - the L298 uses low side current sense which is less accurate to high side or inline current sense.
2. Thermal alert pin - this can be used to indicate that the h-bridge is too hot and should be shutdown by the ESP32 Command Station.
3. Brake pin - this is used by the RailCom detector code to disconnect the h-bridge from the track outputs when receiving RailCom data.
4. Powered by Track Voltage - Most h-bridges use a 5VDC power supply in addition to the track power supply.

[Return to Supported H-bridges](#supported-h-bridges)<br/>[Return to DCC Signal Configuration](#dcc-signal-configuration)<br/>[Return to Configuration](#configuration)

###### Pololu MC33926
The Pololu MC33926 is available in two varieties:
1. [Pololu MC33926 Motor Driver](https://www.pololu.com/product/2503) - This is similar to the L298 Arduino Motor Shield.
2. [Pololu MC33926 Motor Driver Carrier](https://www.pololu.com/product/1213) - This is similar to the [BTS 7960B](#bts7960b) in that it requires additional wiring.

**WARNING**: The MC33926 h-bridge has a built in circuit breaker that will be triggered after around 16 microseconds of "short" and there is no way to disable this functionality. Therefore, this h-bridge is not recommended as it can interfere with auto-reversers.

The [Pololu MC33926 Motor Driver](https://www.pololu.com/product/2503) uses mostly the same pins as the [L298](#l298) but will require the following wire jumpers:
1. M1PWM to D2.
2. M1FB to A4.
3. M2FB to A5.

WARNING: Be sure to remove the VIN/VOUT jumper otherwise the ESP32 may be damaged by the track power supply.

The [Pololu MC33926 Motor Driver Carrier](https://www.pololu.com/product/1213) requires many connections to the ESP32 as well as a [DCC Signal Splitting](#dcc-signal-splitting) circuit.
1. M1 PWM to OPS "H-Bridge enable/pwm pin"
2. M2 PWM to PROG "H-Bridge enable/pwm pin"
3. D1 needs to be the inverted logic value of M1 PWM.
4. D2 needs to be the inverted logic value of M2 PWM.
5. M1 FB to OPS "H-Bridge current sense pin"
6. M2 FB to PROG "H-Bridge current sense pin"
7. M1 IN1 to OPS "H-Bridge signal/direction pin"
8. M2 IN1 to PROG "H-Bridge signal/direction pin"
9. M1 IN2 needs to be the inverted logic value of M1 IN1.
10. M2 IN2 needs to be the inverted logic value of M2 IN1.

For the inverted logic value inputs the [DCC Signal Splitting](#dcc-signal-splitting) circuit can be used.

[Return to Supported H-bridges](#supported-h-bridges)<br/>[Return to DCC Signal Configuration](#dcc-signal-configuration)<br/>[Return to Configuration](#configuration)

###### BTS7960
The BTS7960B motor driver, also known at IBT_2, is a high amperage half h-bridge based motor driver. It is best suited as a standalone booster for the OPS DCC signal but can be used directly connected to the ESP32 Command Station. It is not known if this motor driver is suitable for use on the PROG track.

| ESP32 pin | BTS7960B pin |
| --------- | ---------- |
| 5V | VCC |
| GND | GND |
| OPS "H-Bridge enable/pwm pin" | R_EN and L_EN |
| OPS "H-Bridge current sense pin" | R_IS and L_IS |
| OPS "H-Bridge signal/direction pin" | The [DCC Signal Splitting](#dcc-signal-splitting) will be necessary for the L_PWM and R_PWM inputs. |

[Return to Supported H-bridges](#supported-h-bridges)<br/>[Return to DCC Signal Configuration](#dcc-signal-configuration)<br/>[Return to Configuration](#configuration)

##### DCC Signal Splitting
Some h-bridges require two inputs instead of one for the signal/direction input. The following circuit can be used for that purpose:
![DCC Signal Split](dcc-signal-split.png)

[Return to Supported H-bridges](#supported-h-bridges)<br/>[Return to DCC Signal Configuration](#dcc-signal-configuration)<br/>[Return to Configuration](#configuration)

#### GPIO Configuration

| PARAM | Default | Description |
| ----- | ------- | ----------- |
| Enable GPIO pins to be used as outputs | true | This option will allow usage of free GPIO pins as a controllable output pin.<br/>Note: there are only a handful of pins available on most ESP32 modules so this will be very limited.<br/>Note 2: For the ESP32 custom PCB this option is disabled. |
| Enable GPIO pins to be used as sensors | true | This option will allow usage of free GPIO pins as a monitored input pins.<br/>Note: there are only a handful of pins available on most ESP32 modules so this will be very limited.<br/>Note 2: For the ESP32 custom PCB this option is disabled. |
| Enable S88 Sensor functionality | false | Enabling this option will allow creation of an S88 bus supporting up to 512 sensors.<br/>Note: For the ESP32 custom PCB this option is disabled. |
| Number of milliseconds until a remote sensor will automatically clear | 60000 | This controls how long a remote sensor will remain active if it does not report state information again.<br/>Note: this requires "Enable GPIO pins to be used as sensors" to be enabled. |
| First ID to assign remote sensors | 100 | This is the first ID to assign to any remote sensors.<br/>Note: this requires "Enable GPIO pins to be used as sensors" to be enabled. |
| S88 Clock pin | 17 | This is the S88 clock pin which will be pulsed to read in individual sensor data from the S88 bus.<br/>Note: this requires "Enable S88 Sensor functionality" to be enabled. |
| S88 Reset pin | 16 | This is the S88 reset pin which will be pulsed when preparing to read the S88 sensors. This can be disabled by setting the pin to -1.<br/>Note: this requires "Enable S88 Sensor functionality" to be enabled. |
| S88 Load pin | 27 | This is the S88 load pin which will be pulsed when preparing to read the S88 sensors.<br/>Note: this requires "Enable S88 Sensor functionality" to be enabled. |
| First S88 sensor ID | 512 | This will be used as the first S88 sensor ID.<br/>Note: this requires "Enable S88 Sensor functionality" to be enabled. |
| S88 sensors per bus | 512 | This controls the maximum number of S88 sensors per bus.<br/>Note: this requires "Enable S88 Sensor functionality" to be enabled. |
| Allow usage of restricted GPIO pins | false | By default a number of pins are restricted from use as GPIO inputs/outputs due to usage elsewhere in the ESP32 Command Station or due to ESP32 restrictions on the pins. Enabling this option will disable runtime checks for the ESP32 restricted pins.<br/>Note: it is generally not recommended to enable this option. | 

[Return to Configuration](#configuration)

#### JMRI DCC++ Interface

| PARAM | Default | Description |
| ----- | ------- | ----------- |
| Enable JMRI listener | true | This option enables or disables the JMRI DCC++ TCP/IP listener. |
| JMRI Listener port | 2560 | TCP/IP port that will be used for incoming connections using the DCC++ text based protocol. |
| mDNS service name | _esp32cs._tcp | mDNS service name to advertise for the listener. |

[Return to Configuration](#configuration)

#### Status Display Configuration

| PARAM | Default | Description |
| ----- | ------- | ----------- |
| Type of display | None | This option enables or disables the usage of an OLED or LCD to display runtime status information. |
| I2C SCL pin | 22 | SCL pin to use for I2C communication.<br/>Note: some boards have an LED on this pin and in that case it is recommended to use another pin instead. |
| I2C SDA pin | 21 | SDA pin to use for I2C communication. |
| LCC Stat Logging | Minimal | ADVANCED: This controls how verbose the logging of the LCC statistics collector will be at runtime.<br/>This generally will not require modification and is not recommeneded for use on the layout. |
| [LCD Configuration](#lcd-configuration) | | Click [here](#lcd-configuration) for LCD configuration. |
| [OLED Configuration](#oled-configuration) | | Click [here](#oled-configuration) for OLED configuration. |

[Return to Configuration](#configuration)

##### LCD Configuration
| PARAM | Default | Description |
| ----- | ------- | ----------- |
| Type of LCD display | 20 columns, 4 lines | This controls the dimension of the LCD display.<br/>Note: this is only available if "Type of display" is set to LCD. |
| Enable LCD Backlight | true | This option controls the enabling of the backlight via code.<br/>Note: this is only available if "Type of display" is set to LCD. |
| LCD Enable bitmask | 0x04 | ADVANCED: This controls which bit to set for the enable pin of the LCD display.<br/>Note: this is only available if "Type of display" is set to LCD. |
| LCD Register Select bitmask | 0x01 | ADVANCED: This controls which bit is set for the register select pin of the LCD display.<br/>Note: this is only available if "Type of display" is set to LCD. |
| LCD Backlight Enable bitmask | 0x08 (backlight enabled)</br>0x00 (backlight disabled) | ADVANCED: This controls which bit is set for the backlight pin of the LCD display.<br/>Note: this is only available if "Type of display" is set to LCD. |

[Return to Status Display Configuration](#status-display-configuration)<br/>[Return to Configuration](#configuration)

##### OLED Configuration
| PARAM | Default | Description |
| ----- | ------- | ----------- |
| OLED Reset pin | -1 (unused) | This pin will be pulsed as part of initialization of the OLED. This is not always required and by default is disabled.<br/>Note: this is only available if "Type of display" is set to OLED. |
| Vertically flip | false | Enabling this option will vertically flip the rendering of the OLED display.<br/>Note: this is only available if "Type of display" is set to OLED. |
| Size of OLED display | 128x64 | This controls the size of the OLED.<br/>Note: this is only available if "Type of display" is set to OLED. |
| Contrast ratio | 128 | This controls the contrast setting of the OLED.<br/>Note: this is only available if "Type of display" is set to OLED. |
| OLED Font | Bold | This controls the size of the OLED.<br/>Note: this is only available if "Type of display" is set to OLED. |

[Return to Status Display Configuration](#status-display-configuration)<br/>[Return to Configuration](#configuration)

#### Status LED Configuration

| PARAM | Default | Description |
| ----- | ------- | ----------- |
| Enable Status LED | false | Enabling this option will allow usage of five addressable LEDs for status indication. |
| LED data pin | 22 | This controls which pin will be used to emit the signal for the addressable LEDs to consume on their DIN pin. |
| LED brightness | 128 | This controls how bright the LEDs will appear. |
| LED type | WS281x | What type of addressable LED is connected. |
| LED color order | Red, Green, Blue | This controls the color order that will be sent to the LEDs. |
| LED update frequency (milliseconds) | 450 | This controls how often the LEDs will be updated when there are changes. |

[Return to Configuration](#configuration)<br/>

[Return to ESP32 Command Station](./index.html)