# OpenMRN-lite Arduino Library
The OpenMRN-lite Arduino Library is a packaged version of the OpenMRN code that
has been designed to run in the Arduino environment. The OpenLCB features of
OpenMRN are available through this library.

## Supported platforms/architectures
At this time the only supported platforms are:

* ESP32 using the [arduino-esp32](https://github.com/espressif/arduino-esp32)
core as the underlying stack.
* STM32, but no examples are available at this time.

Additional platforms may be added in the future.

## Preparing the library for use in Arduino IDE or PlatformIO IDE
The OpenMRN-lite library is not currently delivered as a standalone released
library and must be generated using the libify.sh script. Executing this script
creates a library directory that is usable in the various IDEs that support
Arduino development.

### Executing libify.sh to create the OpenMRN-lite library
The libify.sh script requires a bash like environment for execution, on Windows
the Git bash commandline will work. On Linux/MacOS the native bash shell will
work.
    sh libify.sh {path to OpenMRN-lite creation directory} {path to OpenMRN}

#### Arduino IDE library generation
On Windows the Arduino IDE stores the libraries under
"Documents\Arduino\libraries", this can be accessed via the Git bash
commandline as:
```bash
    sh arduino/libify.sh "$USERPROFILE/Documents/Arduino/libraries/OpenMRN-lite" .
```
when executed from the OpenMRN repository root folder.

On Linux the location would normally be
/home/{user}/Documents/Arduino/libraries.

On macOS the location would normally be
/Users/{user}/Documents/Arduino/libraries.

#### PlatformIO IDE library generation
For PlatformIO IDE it would be recommended to put this into the project lib
folder instead. By default the PlatformIO build process will inspect the lib
folder for project specific libraries and will automatically include them in
the compilation.
```bash
    sh arduino/libify.sh "/path/to/project/lib/OpenMRN-lite" .
```
when executed from the OpenMRN repository root folder.

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
where the ESP32 did not receive a response from the access point. This timeout
is unfortuntely not configurable at this time. It is not known if this is due
to a poor signal strength or an underlying bug in the ESP-IDF WiFi driver. The
solution for this appears to be power down the ESP32 and restart the AP. The
ESP32 should successfully connect. Additional options to try if this does not
resolve the connection issues:
1. Before connecting to the AP add these lines:
```C++
        WiFi.mode(WIFI_STA);
        WiFi.disconnect(true);
```

The above two lines should be set before the call to WiFi.begin();
2. If the ESP32 board supports an external WiFi antenna use one, this will
provide a higher signal strength which should allow a more successful
connection.
3. Clear the persistent WiFi connection details from NVS:
```C
        #include <nvs_flash.h>
        nvs_flash_init();
```

This should be considered a last resort option as it will erase any data in the
NVS partition. There is no recovery of data from NVS after executing the above
function. This can also be achieved by using a flash erase tool
`esptool.py erase_flash ...` and reflashing the ESP32.

## ESP32 Hardware CAN support
The ESP32 has a built in CAN controller but lacks a CAN transceiver. There are
two types of transceivers recommended by Espressif:
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
use a resistor between the RX pin on the transceiver and the ESP32 GPIO pin to
prevent damage to the ESP32.
4. GPIO for TX to the TX pin on the transceiver. This pin must be usable as an
output pin, GPIO 34-39 on the ESP32 are input only.

If you are using the MCP2551 transceiver and the ESP32 will be at the end of
the CAN bus, you should include a 150ohm resistor across the H and L lines to
terminate the CAN bus. This is only necessary if there is not already another
node providing the required CAN bus termination.

## Powering the ESP32
It is not recommended to directly connect the CAN bus PWR_POS (7) to the VIN
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
