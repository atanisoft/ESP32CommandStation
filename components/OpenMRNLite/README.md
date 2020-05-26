# OpenMRNLite Arduino Library
This library implements network protocols for model railroading. In the center
is the OpenLCB protocol suite (Open Layout Control Bus), which has been adopted
by the NMRA and referenced as LCC (Layout Command Control): a high-performance
and highly extensible communications protocol suite for model railroad
control. OpenMRN is one of the most extensible implementation of this protocol
suite. This Lite version has been adapted to work with the programming model and
drivers of the Arduino ecosystem.

## Supported platforms/architectures
At this time the only supported platforms are:

* ESP32 using the [arduino-esp32](https://github.com/espressif/arduino-esp32)
core as the underlying stack, version 1.0.1 or 1.0.2-rc2.
* STM32, but no examples are available at this time.

Additional platforms may be added in the future.

## Where to get the library

- by using the Arduino Library Manager to download a released version from the
  global Arduino index.
- by downloading a released source code ZIP file from GitHub
  (https://github.com/openmrn/OpenMRNLite/releases) and importing that ZIP file
  into the Arduino UI.
- by exporting directly from the OpenMRN source tree (under linux also symlinks
  are supported)

# ESP32 supported hardware
At this time the ESP32 supports both WiFi and hardware CAN adapters. Almost
all variants of the ESP32 boards can be used except the ESP32-SOLO-1 based
boards are not supported by [arduino-esp32](https://github.com/espressif/arduino-esp32).
For the hardware CAN support they will require two additional GPIO pins. WiFi
does not require any additional GPIO pins.

## ESP32 WiFi support
The Esp32WiFiManager should be used to manage the ESP32's WiFi connection to
the SSID, create a Soft AP (max of 4 clients connected to it) or to create both
a Soft AP and connect to an SSID. See the ESP32IOBoard example for how to use
the Esp32WiFiManager for how to connect to an SSID and have it automatically
establish an uplink to a GridConnect based TCP/IP Hub. Additional configuration
parameters can be configured via the CDI interface.

## Using an SD card for configuration data
When using an SD card for storage of the OpenMRN configuration data it is
recommended to use an AutoSyncFileFlow to ensure the OpenMRN configuration
data is persisted to the SD card. The default configuration of the SD virtual
file system driver is to use a 512 byte per-file cache and only persist
on-demand or when reading/writing outside this cached space. The
AutoSyncFileFlow will ensure the configuration file is synchronized to the SD
card on a regular basis. The SD VFS driver will check that the file has pending
changes before synchronizing them to the SD card and when no changes are
necessary no action is taken by the synchronization call.

Note that the Arduino-esp32 SPIFFS library and the underlying SPIFFS VFS driver
does use a cache but the AutoSyncFileFlow is not necessary due to the nature of
the SPIFFS file system (monolithic blob containing all files concatenated).

## ESP32 Hardware CAN support
The ESP32 has a built in CAN controller and needs an external CAN transceiver
only. There are two types of transceivers recommended by Espressif:
1. SN65HVD23x, also known as VP230, this is a 3.3V transceiver that typically
is only available as a breakout board with on board termination resistor.
2. MCP2551, this is a 5V transceiver that is available as a DIP-8, SMD or as a
breakout board. This does not include a termination resistor and for this
reason is preferred over the SN65HVD23x boards.

Connecting the CAN transceiver to the ESP32 requires four connections:
1. 3v3 or 5v to the VCC/VDD pin on the transceiver. Make sure to use the right
voltage for the transceiver.
2. GND to the GND pin on the transceiver.
3. GPIO for RX to the RX pin on the transceiver. This can be any unused GPIO
pin. Note, if you are using a 5V transceiver (ie: MCP2551) it is recommended to
use a 1k or higher resistor between the RX pin on the transceiver and the ESP32
GPIO pin to prevent damage to the ESP32.
4. GPIO for TX to the TX pin on the transceiver. This pin must be usable as an
output pin, GPIO 34-39 on the ESP32 are input only.

If you are using the MCP2551 transceiver and the ESP32 will be at the end of the
CAN bus, you should include a 120ohm resistor across the H and L lines to
terminate the CAN bus. This is necessary on the two ends of the CAN-bus.

## Powering the ESP32
It is not recommended to directly connect the CAN bus `PWR_POS` (7) to the VIN
pin on the ESP32 as the CAN bus will not supply the necessary current for the
ESP32 and the PWR_POS can be up to 15V DC which is more than the power
regulator on most ESP32 boards can handle. For these reasons, a DC-DC
step-down / buck converter is required to bring the voltage down to 3.3V DC or
5V DC before connecting it to the ESP32 VIN pin.

For standalone power supply it should be a regulated 3.3V DC or 5V DC 750mA
power supply. If multiple ESP32 devices will be supplied by this power supply
the amperage may need to be higher.

Some ESP32 boards may not have a pin marked as VIN, in these cases the VIN pin
would be either 5V or 3v3 depending on the voltage provided by the step-down /
buck converter. Some boards also have a BAT/VBAT pin, this is typically a 3.3V
DC pin. Some boards will have a pin marked USB, this is typically a 5V DC pin.
Be sure to consult the specification sheet for the ESP32 board you are using
before supplying power to it from the CAN bus or a standalone power supply.

The ESP32 devices will detect under current situations and will shutdown and
restart automatically and include the message "ESP_RST_BROWNOUT" as part of the
start-up messages.

## GPIO restricted and bootstrapping pins
There are a handful of GPIO pins on the ESP32 that have some restrictions on
their use.

GPIO 6-11 are connected to the on-board flash chip and can not be used by
application code.

IF GPIO 0 and 2 are LOW (GND) at the time the ESP32 is powered-up it will
enter "code download mode," this is used as part of the code flashing process.
GPIO 0 has an internal pull-up resistor and GPIO 2 has an internal pull-down
resistor. After power-up these pins can safely be used.

GPIO 12 (MTDI) controls the VDD_SDIO voltage, there is an internal pull-down
resistor on this pin. This pin *MUST* not be held HIGH at power-up otherwise
damage to the ESP32 flash is likely. After power-up this pin can safely be used.

GPIO 15 (MTDO) controls the debug printing on UART0 during power-up, there is an
internal pull-up resistor (enables debug output). This pin is also used in
conjunction with GPIO 5 to set the SDIO Slave timing. GPIO 5 also has an
internal pull-up resistor. After power-up both GPIO 5 and 15 can safely be
used.

Additional details on the GPIO pin restrictions can be found in this
(document)[https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32_datasheet_en.pdf]
