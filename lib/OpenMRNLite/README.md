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
The ESP32 WiFi stack has issues at times but is generally stable. If you
observe failures in connecting to WiFi add the following compiler option
to turn on additional diagnostic output from the
[WiFi](https://github.com/espressif/arduino-esp32/tree/master/libraries/WiFi)
library:
    `-DCORE_DEBUG_LEVEL=ARDUHAL_LOG_LEVEL_DEBUG`
This will give additional output on the Serial console which can help
to resolve the connection issue. The following is an example of the output
which may be observed:
```
    [D][WiFiGeneric.cpp:342] _eventCallback(): Event: 5 - STA_DISCONNECTED
    [W][WiFiGeneric.cpp:357] _eventCallback(): Reason: 2 - AUTH_EXPIRE
    [D][WiFiGeneric.cpp:342] _eventCallback(): Event: 0 - WIFI_READY
    [D][WiFiGeneric.cpp:342] _eventCallback(): Event: 2 - STA_START
    [D][WiFiGeneric.cpp:342] _eventCallback(): Event: 2 - STA_START
    [D][WiFiGeneric.cpp:342] _eventCallback(): Event: 5 - STA_DISCONNECTED
```

If you observe this output, this generally means there was a timeout condition
where the ESP32 did not receive a response from the access point. It generally
means that the ESP32 is too far from the AP for the AP to hear what the ESP32
is transmitting. Additional options to try if this does not resolve the
connection issues:
1. If the ESP32 board supports an external WiFi antenna use one, this will
provide a higher signal strength which should allow a more successful
connection.
2. Clear the persistent WiFi connection details from NVS:
```C
        #include <nvs_flash.h>
        nvs_flash_init();
```

This should not be done very often (i.e. do not do it at every startup!), but
is otherwise harmless. The same can also be achieved by using a flash erase
tool `esptool.py erase_flash ...` and reflashing the ESP32.

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
