# Configuring build options

ESP32 Command Station is a complex project with many configuration options
available to customize it's behavior.  All configuration options are available
within the `menuconfig` utility under `ESP32 Command Station Configuration`.

## Graphical or command line configuration utility

There are two ways to configure the build options for ESP32 Command Station:
* Graphical
* Command line

Both are nearly identical in functionality.

### Using menuconfig within VSCode (with Espressif IDF plugin)

The [Espressif IDF plugin](https://marketplace.visualstudio.com/items?itemName=espressif.esp-idf-extension)
provides a built-in graphical `menuconfig` utility. With the ESP32 Command
Station code opened as the active project click the :gear: icon or press F1
and type in `menuconfig` to open the utility.

### Using menuconfig from commandline

When using an ESP-IDF enabled commandline environment the `menuconfig` utility
can be opened by running `idf.py menuconfig` from the ESP32CommandStation
source code root directory.

## Configuring ESP32 Command Station Options

Inside the `menuconfig` utility the `ESP32 Command Station Configuration`
section contains all configuration options that are typically customized. In
this section you will find:

| Component | Description |
| --------- | ----------- |
| [ESP32 / H-Bridge Selection](#esp32-h-bridge-selection) | Provides automatic configuration for various H-Bridge options. |
| [WiFi Configuration](#wifi-configuration) | Configures the WiFi settings. |
| [DCC Signal Configuration](#dcc-signal-configuration) | Configures the DCC Signal Generation. |
| [OpenLCB Configuration](#openlcb-configuration) | Configures OpenLCB (LCC) settings. |
| [Status Display Configuration](#status-display-configuration) | Configures an optional OLED/LCD display. |
| [RailCom Configuration](#railcom-configuration) | Configures RailCom functionality. |
| [GPIO Pin Assignment](#gpio-pin-assignment) | Configures how GPIO pins are used.<br/>**NOTE:** This option is only available when selecting `ESP32 (custom configuration)` under [ESP32 / H-Bridge Selection](#esp32-h-bridge-selection). |
| [Status LED Configuration](#status-led-configuration) | Configures addressable LEDs for optional status indicators. |
| [Accessory Decoder Management](#turnout-management) | Configures how turnouts are handled. |
| [Locomotive Roster](#locomotive-roster) | Configures how turnouts are handled. |
| [Crash Behavior](#crash-behavior) | Configures how to handle unexpected crashes. |

### ESP32 / H-Bridge Selection

This section allows automatic configuration of features and pin assignment based
on the ESP32 and H-Bridge in use. If the ESP32 and H-Bridge in use is not listed
then it will be necessary to configure all options manually.

| Config Key | NAme | Description |
| ---------- | ---- | ----------- |
| CONFIG_ESP32CS_L298 | ESP32 with Arduino Motor Shield (OPS and PROG) | Selecting this option configures both OPS and PROG tracks using the L298 A and B outputs. |
| CONFIG_ESP32CS_LMD18200 | ESP32 with LMD18200 (OPS Only) | Selecting this option configures only the OPS track output, PROG track support will be disabled. |
| CONFIG_ESP32CS_BTS7960B | ESP32 with BTS7960B (OPS Only) | Selecting this option configures only the OPS track output, PROG track support will be disabled. |
| CONFIG_ESP32CS_BTS7960B_X2 | ESP32 with 2x BTS7960B (OPS and PROG) | Selecting this option configures both OPS and PROG tracks using two BTS7960B devices. |
| CONFIG_ESP32CS_CUSTOM | ESP32 (custom configuration) | Selecting this option will require configuration of all GPIO pins. |

**NOTE:** The default selection by `menuconfig` is `ESP32 with Arduino Motor Shield (L298)`
and will influence all GPIO pin assignments.

[Return to "Configuring ESP32 Command Station Options"](#configuring-esp32-command-station-options)

### WiFi Configuration

This defines how the ESP32 Command Station utilizes the WiFi capabilities of
the ESP32. By default the ESP32 Command Station will create a SoftAP which can
be used to configure the WiFi connection at runtime rather than as part of the
build. All pre-built binaries will default to create a SoftAP.

| Parameter | Default | Description |
| --------- | ------- | ----------- |
| WiFi Mode | Create SoftAP | This controls how the WiFi support is used at runtime, supported options:<br/><li>Connect to SSID</li><li>Create SoftAP</li><li>Connect to SSID and create SoftAP</li><li>Disabled</li> |
| Hostname prefix | esp32cs_ | This is used to generate a unique hostname for the ESP32 Command Station.<br/>Note: the OpenLCB Node ID will be appended to this value. |
| [Station Configuration](#station-configuration) | | See [Station Configuration](#station-configuration) for more details |
| [SoftAP Configuration](#softap-configuration) | | See [SoftAP Configuration](#softap-configuration) for more details |
| [SNTP Configuration](#sntp-configuration) | | See [SNTP Configuration](#sntp-configuration) for more details |

[Return to "Configuring ESP32 Command Station Options"](#configuring-esp32-command-station-options)

#### Station Configuration

| Parameter | Default | Description |
| --------- | ------- | ----------- |
| Station SSID | not provided | This is the SSID to connect to when operating in "Connect to SSID" or "Connect to SSID and create SoftAP" mode. |
| Station Password | not provided | This is the password for the SSID when operating in "Connect to SSID" or "Connect to SSID and create SoftAP" mode. |

[Return to "WiFi Configuration"](#wifi-configuration) | [Return to "Configuring ESP32 Command Station Options"](#configuring-esp32-command-station-options)

#### SoftAP Configuration

| Parameter | Default | Description |
| --------- | ------- | ----------- |
| SoftAP SSID | esp32csap | Name of the AP to advertise when "Create SoftAP" is enabled. |
| SoftAP Password | esp32csap | Password for connecting to the AP to advertise when "Create SoftAP" is enabled. |
| SoftAP Channel | 1 | WiFi channel to use for the SoftAP. |

[Return to "WiFi Configuration"](#wifi-configuration) | [Return to "Configuring ESP32 Command Station Options"](#configuring-esp32-command-station-options)

#### SNTP Configuration

| Parameter | Default | Description |
| --------- | ------- | ----------- |
| Enable SNTP synchronizatio | disabled | Enabling this option will allow the ESP32 Command Station to retrieve the current time from a remote time server. |
| SNTP Server | pool.ntp.org | Remote time server to connect to. |
| Timezone | UTC0 | Timezone adjustment to use for the retrieved time data. A full list of supported values can be seen [here](https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv). |

[Return to "WiFi Configuration"](#wifi-configuration) | [Return to "Configuring ESP32 Command Station Options"](#configuring-esp32-command-station-options)

### DCC Signal Configuration

| Parameter | Default | Description |
| --------- | ------- | ----------- |
| Enable OPS track output | Off | Enabling this option will enable the OPS track signal.<br/>**NOTE:** This option will be enabled automatically for all [ESP32 / H-Bridge Selection](#esp32-h-bridge-selection) *EXCEPT* `ESP32 (custom configuration)` |
| Enable PROG track output | Off | Enabling this option will enable the PROG track signal.<br/>**NOTE:** This option will be enabled automatically for `ESP32 with Arduino Motor Shield (L298 for OPS & PROG)` and `ESP32 with 2x BTS7960B (OPS and PROG)` under [ESP32 / H-Bridge Selection](#esp32-h-bridge-selection). |
| Energize track upon startup | true (if OPS track is selected)<br/>false otherwise | Enabling this option will enable the OPS track output automatically on startup of the ESP32 Command Station rather than wait for the OpenLCB event to enable track power.<br/>**NOTE:** This option is only available when selecting `Enable OPS track output` |
| OPS H-Bridge type | L298 | This configures the type of H-Bridge used for the OPS track.<br/>Available options:<br/><li>L298 (2A limit)</li><li>LMD18200 (3A limit)</li><li>DRV880x (2.8A limit)</li><li>DRV8873 (10A limit)</li><li>DRV8873 (5A limit)</li><li>Pololu MC33926 (2.5A limit)</li><li>BTS7960B (5A limit)</li><li>BTS7960B (10A limit)</li><br/>**NOTE:** This option is only available when selecting `ESP32 (custom configuration)` under [ESP32 / H-Bridge Selection](#esp32-h-bridge-selection). |
| PROG H-Bridge type | L298 | This configures the type of H-Bridge used for the PROG track.<br/>Available options:<br/><li>L298 (2A limit)</li><li>LMD18200 (3A limit)</li><li>DRV880x (2.8A limit)</li><li>DRV8873 (10A limit)</li><li>DRV8873 (5A limit)</li><li>Pololu MC33926 (2.5A limit)</li><li>BTS7960B (5A limit)</li><li>BTS7960B (10A limit)</li><br/>**NOTE:** This option is only available when selecting `ESP32 (custom configuration)` under [ESP32 / H-Bridge Selection](#esp32-h-bridge-selection). |

**NOTE**: There are advanced configuration options but they typically do not
require updates from the default values.

[Return to "Configuring ESP32 Command Station Options"](#configuring-esp32-command-station-options)

### OpenLCB Configuration

ESP32 Command Station provides an OpenLCB Interface (both WiFi and TWAI/CAN).
OpenLCB is the underlying standards upon which the NMRA LCC (Layout Command
Control) standard is based.

| Parameter | Default | Description |
| --------- | ------- | ----------- |
| Node ID | 05.02.01.03.FF.FE | This is the OpenLCB Node Identifier to use for the ESP32 Command Station. The default value is part of a reserved range allocated to 
| Enable TWAI (CAN) interface | Disabled | When this option is enabled an external CAN Transceiver is used to communicate with other OpenLCB nodes. |
| [Fast Clock Configuration](#fast-clock-configuration) | | Configures the built-in Fast Clock functionality. |
| [Train Search Protocol](#train-search-protocol) | Configures OpenLCB Train Search Protocol support. |
| [Thermal Monitor Configuration](#thermal-monitor-configuration) | Configures thermal monitoring. |

**NOTE**: There are advanced configuration options but they typically do not
require updates from the default values.

[Return to "Configuring ESP32 Command Station Options"](#configuring-esp32-command-station-options)

#### Fast Clock Configuration

ESP32 Command Station supports generating OpenLCB events for two types of Fast
Clocks, fast and real-time.

| Parameter | Default | Description |
| --------- | ------- | ----------- |
| Enable FastClock (fast) | Disabled | When enabled the ESP32 Command Station will generate FastClock events via OpenLCB. |
| Enable FastClock (real-time) | Disabled | When enabled the ESP32 Command Station will generate FastClock events via OpenLCB. |
| [FastClock (fast)](#fastclock-fast) | | See [FastClock (fast)](#fastclock-fast) for more details. |
| [FastClock (real-time)](#fastclock-real-time) | | See [FastClock (real-time)](#fastclock-real-time) for more details. |

[Return to "OpenLCB Configuration"](#openlcb-configuration) | [Return to "Configuring ESP32 Command Station Options"](#configuring-esp32-command-station-options)

##### FastClock (fast)

This clock provides a faster (or slower) than real-time. This can be used as
part of scheduling of locomotives/events/etc via OpenLCB Events.

| Parameter | Default | Description |
| --------- | ------- | ----------- |
| Rate | 4 | Rate at which the FastClock will advance time. |
| Year | 1900 | Year at which to start the FastClock, this is persisted across restarts of the ESP32 Command Station. |
| Month | 1 (January) | Month at which to start the FastClock, this is persisted across restarts of the ESP32 Command Station. |
| Day | 1 | Day at which to start the FastClock, this is persisted across restarts of the ESP32 Command Station. |
| Hour (24hr) | 1 | Hour at which to start the FastClock, this is persisted across restarts of the ESP32 Command Station. |
| Minute | 1 | Minute at which to start the FastClock, this is persisted across restarts of the ESP32 Command Station. |
| Clock ID | 01.01.00.00.01.00 | OpenLCB Clock ID, this typically will not need to be changed. |

[Return to "Fast Clock Configuration"](#fast-clock-configuration) | [Return to "OpenLCB Configuration"](#openlcb-configuration) | [Return to "Configuring ESP32 Command Station Options"](#configuring-esp32-command-station-options)

##### FastClock (real-time)

This clock provides real-time updates roughly every minute. This requires that
SNTP is enabled and configured under [SNTP Configuration](#sntp-configuration).
This can be used as part of scheduling of locomotives/events/etc via OpenLCB
Events.

| Parameter | Default | Description |
| --------- | ------- | ----------- |
| Clock ID | 01.01.00.00.01.01 | OpenLCB Clock ID, this typically will not need to be changed. |

[Return to "Fast Clock Configuration"](#fast-clock-configuration) | [Return to "OpenLCB Configuration"](#openlcb-configuration) | [Return to "Configuring ESP32 Command Station Options"](#configuring-esp32-command-station-options)


#### Train Search Protocol

There are no configurable options, aside from advanced options, that can be
configured in this section. The advanced options typically do not require any
updates and are primarily for debug purposes.

[Return to "OpenLCB Configuration"](#openlcb-configuration) | [Return to "Configuring ESP32 Command Station Options"](#configuring-esp32-command-station-options)

#### Thermal Monitor Configuration

The ESP32 Command Station can monitor an external thermistor to estimate the
running temperature of the H-Bridge or PCB, whichever is closer/connected to
the thermistor. When the detected temperature rises above the configured
thresholds the ESP32 Command Station can emit a warning event or shutdown the
DCC track output(s) entirely until the temperature returns to below the
configured thresholds.

The ESP32 Command Station PCBs use an [MCP9701A](http://ww1.microchip.com/downloads/en/devicedoc/20001942g.pdf)
thermistor IC to detect the temperature of the DRV8873 H-Bridge which is
passively cooled. The DRV8801 H-Bridges used for PROG and OpenLCB outputs are
not directly monitored as they are limited to under 500mA output external limit
resistors, the PROG track is current limited to approximately 250mA via current
limiting circuitry on the power supply and OpenLCB output power supply is
limited by a 500mA voltage regulator.

| PARAM | Default | Description |
| ----- | ------- | ----------- |
| Thermistor/IC | MCP9701/MCP9701A | Type of Thermistor/IC in use:<br/><li>MCP9700/MCP9700A -- </li><li>MCP9701/MCP9701A -- </li><li>Custom -- </li> |
| Millivolts at 0C | varies:<br><li>MCP9700/MCP9700A -- 500mV</li><li>MCP9701/MCP9701A -- 400mV</li><li>0 -- default</li> | This is the voltage (in mV) that will be read when the thermistor is reading zero Celcius. |
| Millivolts per 1C | varies:<br><li>MCP9700/MCP9700A -- 10.0mV/C</li><li>MCP9701/MCP9701A -- 19.5mV/C</li><li>10 -- default</li> | This is used to calculate the temperature offset from 0C.<br/>**NOTE:** This value is multiplied by 10 to work with whole numbers as menuconfig does not support decimals in values. |
| Warning temperature (C) | 50 | Temperature (C) at which to raise a warning. |
| Shutdown temperature (C) | 80 | Temperature (C) at which to shutdown all DCC track output(s) and raise an event indicating shutdown. |

[Return to "Configuring ESP32 Command Station Options"](#configuring-esp32-command-station-options)

### Status Display Configuration

ESP32 Command Station supports connecting an external I2C display (LCD or OLED)
which is used to report near real-time status updates.

| Parameter | Default | Description |
| --------- | ------- | ----------- |
| Display Type | None | This configures the type of external display:<br><li>[LCD](#lcd-display-configuration)</li><li>[OLED](#oled-display-configuration)</li> |

[Return to "Configuring ESP32 Command Station Options"](#configuring-esp32-command-station-options)

#### LCD Display Configuration

| PARAM | Default | Description |
| ----- | ------- | ----------- |
| Display Size | 20 columns, four lines | This configures how much information is displayed on the connected LCD:<br/><li>20 columns, 4 lines -- Displays maximum information</li><li>16 columns, 4 lines -- Displays maximum information, some truncation in labels</li><li>20 columns, 2 lines -- Displays primarily status information</li><li>16 columns, 2 lines -- Displays primarily status information, some truncation in labels</li> |
| Enable Backlight | On | Setting this to enabled will turn on the backlight automatically as part of the display initialization. Some LCDs require this to be disabled. |

**NOTE**: There are advanced configuration options but they typically do not
require updates from the default values.

[Return to "Status Display Configuration"](#status-display-configuration) | [Return to "Configuring ESP32 Command Station Options"](#configuring-esp32-command-station-options)

#### OLED Display Configuration

| PARAM | Default | Description |
| ----- | ------- | ----------- |
| Display Size | 128x64 | This configures the size of the connected display:<br/><li>128x64 pixels -- maximum information displayed.</li><li>128x32 pixels -- Displays most information.</li><li>96x16 pixels -- Displays only critical information.</li> |
| Font | Bold | This selects which font to use on the connected display:<br/><li>Thin</li><li>Bold</li><br/>There is no functional difference between the two selections. |
| Vertically flip display | Off | Enabling this option will flip the screen orientation vertically. |
| Contrast Level | 128 | This can be used to adjust the contrast level of the connected display. |

[Return to "Status Display Configuration"](#status-display-configuration) | [Return to "Configuring ESP32 Command Station Options"](#configuring-esp32-command-station-options)


### RailCom Configuration

| Parameter | Default | Description |
| --------- | ------- | ----------- |
| Mode | Disabled | When enabled the ESP32 Command Station will generate a RailCom cut-out period and capture feedback data via a RailCom receiver circuit. |
| UART | UART2 | The ESP32 has three hardware UART devices, the first one (UART0) is used via the USB connector as console output. There are no functional differences between UART1 or UART2. |

**NOTE**: There are advanced configuration options but they typically do not
require updates from the default values.

[Return to "Configuring ESP32 Command Station Options"](#configuring-esp32-command-station-options)

### GPIO Pin Assignment

ESP32 Command Station utilizes nearly all available GPIO pins on ESP32 and
ESP32-S3 devices. This section configures how they are used.

| Section | Description |
| ------- | ----------- |
| [DCC](#dcc) | This section configures which GPIO pins will be used for DCC signal generation. |
| [OPS](#ops) | This section configures which GPIO pins are used for the OPS track. |
| [PROG](#prog) | This section configures which GPIO pins are used for the PROG track. |
| [RailCom Receiver](#railcom-receiver) | This section configures which GPIO pins are used by the RailCom receiver circuitry. |
| [I2C](#i2c) | This section configures which GPIO pins will be used for I2C communication. |
| [Status Display](#status-display) | This section configures which GPIO pins will be used by an attached I2C display. |
| [Buttons](#buttons) | This section configures which GPIO pins are connected to external buttons. |
| [TWAI (CAN)](#twai-can) | This section configures which GPIO pins are used for TWAI (CAN). |
| [Thermal Sensor](#thermal-sensor) | This section configures an external thermal monitoring of the H-Bridge/PCB. |
| [SD Card](#sd-card) | Tihs section configures how an SD card can be accessed. |

[Return to "Configuring ESP32 Command Station Options"](#configuring-esp32-command-station-options)

#### DCC

ESP32 Command Station generates a single DCC signal which is consumed by OPS,
PROG and OpenLCB DCC outputs. In some cases this may require utilization of a
jumper wire to connect OPS and PROG DCC signal inputs.

| Parameter | Default | Description |
| --------- | ------- | ----------- |
| Track signal/direction pin | 16 (default)<br/>19 (L298 / LMD18200)<br/>10 (ESP32-S3) | This GPIO pin will have the encoded DCC signal, this will typically connect to the direction or phase pin on the H-Bridge. |
| Track brake pin | -1 (not used) | Selecting a valid GPIO pin for this option enables the generation of a RailCom cut-out period in the encoded DCC signal. Enabling this option alone will not provide full RailCom support. |
| OpenLCB DCC Enable Pin | -1 (not used) | Selecting a valid GPIO pin for this option allows enabling/disabling of the OpenLCB DCC signal independent from the OPS track signal.<br/>**NOTE:** This option is only available on the ESP32-S3. |

[Return to "GPIO Pin Assignment"](#gpio-pin-assignment) | [Return to "Configuring ESP32 Command Station Options"](#configuring-esp32-command-station-options)

#### OPS

This section defines how the ESP32 Command Station should control and monitor
the OPS track output.

| Parameter | Default | Description |
| --------- | ------- | ----------- |
| OPS Track enable/pwm pin | 17 (default)<br/>25 (L298 / LMD18200)<br/>11 (ESP32-S3) | This GPIO pin is used to enable/disable the OPS track H-Bridge. |
| OPS Track current sense pin | ADC1 Channel 0 | This GPIO pin is used to detect short circuits and estimate the current draw on the OPS track. |

**NOTE:** This section will only be visible when
[DCC Signal Configuration](#dcc-signal-configuration) `Track Outputs` option
includes `OPS` being enabled.

[Return to "GPIO Pin Assignment"](#gpio-pin-assignment) | [Return to "Configuring ESP32 Command Station Options"](#configuring-esp32-command-station-options)

#### PROG

This section defines how the ESP32 Command Station should control and monitor
the PROG track output.

| Parameter | Default | Description |
| --------- | ------- | ----------- |
| PROG Track enable/pwm pin | 18 (default)<br/>23 (L298 / LMD18200)<br/>9 (ESP32-S3) | This GPIO pin is used to enable/disable the PROG track H-Bridge. |
| PROG Track current sense pin | ADC1 Channel 3 | This GPIO pin is used to detect PROG track "ACK" pulses and short circuit detection. |

**NOTE:** This section will only be visible when
[DCC Signal Configuration](#dcc-signal-configuration) `Track Outputs` option
includes `PROG` being enabled.

[Return to "GPIO Pin Assignment"](#gpio-pin-assignment) | [Return to "Configuring ESP32 Command Station Options"](#configuring-esp32-command-station-options)

#### RailCom Receiver

ESP32 Command Station can receive RailCom feedback from compatible DCC mobile
or stationary decoders, this requires an external receiver circuit which can be
controlled via 3v3 TTL logic.

| Parameter | Default | Description |
| --------- | ------- | ----------- |
| Enable pin | 12 (ESP32)<br/>21 (ESP32-S3) | This GPIO pin enables the RailCom receiver circuitry. |
| Data pin | 26 (ESP32)<br/>47 (ESP32-S3) | This GPIO pin is used to receive RailCom data. |
| Direction pin | 25 (ESP32)<br/>48 (ESP32-S3) | This GPIO pin is used to detect direction of travel. |
| Short pin | 45 | **NOTE:** This option is only available for the ESP32-S3 and is used to detect shorts in the RailCom circuitry. |

**NOTE:** This section will only be visible when
[RailCom Configuration](#railcom-configuration) `Mode` option is set to one of
the following values:
- Generate cut-out and receive data.
- Generate cut-out only.

[Return to "GPIO Pin Assignment"](#gpio-pin-assignment) | [Return to "Configuring ESP32 Command Station Options"](#configuring-esp32-command-station-options)

#### I2C

ESP32 Command Station uses I2C to communicate with external display devices and
current sense ICs (ESP32-S3 only). These pins should have external 3v3 pull-up
resistors added, typically around 2.4 kOhm. Without pull-up resistors the I2C
devices may be unreliable.

| Parameter | Default | Description |
| --------- | ------- | ----------- |
| SCL pin | 21 (ESP32)<br/>18 (ESP32-S3) | This GPIO pin is used for I2C clock pulses. |
| SDA pin | 23 (ESP32)<br/>8 (ESP32-S3) | This GPIO pin is used for I2C data. |

[Return to "GPIO Pin Assignment"](#gpio-pin-assignment) | [Return to "Configuring ESP32 Command Station Options"](#configuring-esp32-command-station-options)

#### Status LED

ESP32 Command Station supports up to five externally connected LEDs as status
indicators. For the `ESP32` these must be Addressable RGB LEDs due to
insufficient GPIO pins. For the `ESP32-S3` these must be discrete LEDs due to
insufficient RMT (Remote) TX buffers which are also used for the DCC Signal.

##### ESP32 Options

| Parameter | Default | Description |
| --------- | ------- | ----------- |
| Addressable RGB Data pin | -1 (unused) | This GPIO pin is used for generating addressable RGB LED data for five addressable LEDs.<br/>See [Status LED Configuration](#status-led-configuration) for configuration of Addressable RGB LEDs. |

##### ESP32-S3 Options

| Parameter | Default | Description |
| --------- | ------- | ----------- |
| WiFi Active LED pin | 15 | This GPIO pin will be LOW when the WiFi driver has connected to the SSID or alternating HIGH/LOW when it is trying to connect. |
| SoftAP Active LED pin | 7 | This GPIO pin will be LOW when the WiFi driver has created the SoftAP. |
| Bootloader Active LED pin | 6 | This GPIO pin will be LOW when the OpenLCB Bootloader is active, HIGH otherwise. |
| OPS Track Active LED pin | 5 | This GPIO pin will be LOW when the OPS Track is ON, HIGH otherwise. |
| PROG Track Active LED pin | 4 | This GPIO pin will be LOW when the PROG Track is ON, HIGH otherwise. |

[Return to "GPIO Pin Assignment"](#gpio-pin-assignment) | [Return to "Configuring ESP32 Command Station Options"](#configuring-esp32-command-station-options)

#### Status Display

Some external status displays require additional GPIO pins, these can be
configured here.

| Parameter | Default | Description |
| --------- | ------- | ----------- |
| OLED Reset pin | 0 (ESP32)<br/>3 (ESP32-S3) | This GPIO pin is used to generate a reset pulse which may be needed by some I2C OLED displays. Set to -1 if this is not needed.<br/>NOTE: This will only be used if [Status Display Configuration](#status-display-configuration) is also configured. |

[Return to "GPIO Pin Assignment"](#gpio-pin-assignment) | [Return to "Configuring ESP32 Command Station Options"](#configuring-esp32-command-station-options)

#### Buttons

ESP32 Command Station supports using external user input buttons to trigger
certain actions during startup. These should read HIGH (3v3) when they are
not active and read LOW (0v) when the action should be taken.

**NOTE:** It is not recommended (or supported) to apply 5v to any GPIO pin on
ESP32 devices. Doing so may cause irrepariable damage.

| Parameter | Default | Description |
| --------- | ------- | ----------- |
| Factory Reset pin | -1 (unused) | If this GPIO pin reads LOW for ten seconds during startup, all persistent configuration data will be cleared.<br/>If this GPIO pin reads low for at least five seconds (but less ten seconds) during startup, OpenLCB events will be regenerated. |
| Bootloader Request pin | -1 (unused) | If this GPIO pin reads LOW during startup, the OpenLCB Bootloader will be started and new firmware can be loaded via the TWAI (CAN) physical connection.<br/>NOTE: This option will only be visible if [OpenLCB Configuration](#openlcb-configuration) "Enable TWAI (CAN) interface" is enabled. |

[Return to "GPIO Pin Assignment"](#gpio-pin-assignment) | [Return to "Configuring ESP32 Command Station Options"](#configuring-esp32-command-station-options)

#### TWAI (CAN)

ESP32 Command Station requires an external CAN transceiver to communicate with
other OpenLCB nodes via the
[CAN Physical](https://github.com/openlcb/documents/blob/master/standards/CanPhysicalS.pdf)
interface. This interface can be enabled via the `Enable TWAI (CAN) interface`
option under [OpenLCB Configuration](#openlcb-configuration).

| Parameter | Default | Description |
| --------- | ------- | ----------- |
| RX pin | varies:<br/><li>4 (ESP32 CS PCB)</li><li>16 (ESP32-S3 CS PCB)</li><li>-1 (others)</li> | This GPIO pin would be connected to the external CAN Transceiver RX pin. |
| TX pin | varies:<br/><li>5 (ESP32 CS PCB)</li><li>17 (ESP32-S3 CS PCB)</li><li>-1 (others)</li> | This GPIO pin would be connected to the external CAN Transceiver TX pin. |

[Return to "GPIO Pin Assignment"](#gpio-pin-assignment) | [Return to "Configuring ESP32 Command Station Options"](#configuring-esp32-command-station-options)

#### Thermal Sensor

ESP32 Command Station can use an external thermistor to estimate the running
temperature of an H-Bridge or the PCB itself as a deciding factor for taking
pre-defined actions (send warning or shutdown track outputs).

| Parameter | Default | Description |
| --------- | ------- | ----------- |
| Thermistor IC pin | Disabled | This GPIO pin should be connected to an external theristor which can be used to approximate the temperature of the PCB and/or H-Bridge.<br/>See [Thermal Monitor Configuration](thermal-monitor-configuration) for more details. |

[Return to "GPIO Pin Assignment"](#gpio-pin-assignment) | [Return to "Configuring ESP32 Command Station Options"](#configuring-esp32-command-station-options)

#### SD Card

ESP32 Command Station maintains a handful of configuration and database files.
These can be stored either on an externally connected SD card or internally via
SPIFFS. An external SD card is preferred as it does not create wear on the
built-in flash on the ESP32/ESP32-S3.

| Parameter | Default | Description |
| --------- | ------- | ----------- |
| SD Mode | Disabled | Enabling this option will utilize an external SD card for persistent configuration data.<br/>Supported options:<li>SD SPI</li><li>SD MMC</li> |

On the ESP32-S3 additional configuration options are available to re-map the SD SPI and SD MMC pins from the defaults.

[Return to "GPIO Pin Assignment"](#gpio-pin-assignment) | [Return to "Configuring ESP32 Command Station Options"](#configuring-esp32-command-station-options)


### Status LED Configuration

This section configures five addressable RGB LEDs which are used as status
indicators.

The ESP32 Command Station PCB (ESP32) uses five TX1812 LEDs.

| Parameter | Default | Description |
| --------- | ------- | ----------- |
| Brightness | 64 | This controls how intense/bright the LEDs will be illuminated, lower values will reduce the intensity. |
| Type | WS281X | This is the type of addressable RGB LEDs in use:<br/><li>WS281X -- NeoPixel, WS2812, WS2813.</li><li>WS281X (800k) -- Same as WS281X but faster data rate.</li><li>WS281X (400k) -- Same as WS281X but slower data rate.</li><li>WS2811 -- WS2811 (or similar LEDs).</li><li>SK6812 -- SK6812 (or similar LEDs).</li><li>LC6812 -- LC6812 (or similar LEDs).</li><li>APA106 -- APA106 is a through-hole LED with similar properties to WS2812 LEDs.</li><li>TX1812 -- TX1812 similar to SK6812 LEDs.</li> |
| Color order | Green, Red, Blue (WS2812)<br>Red, Green, Blue (others) | This defines the order in which to transmit color data and if White is included. |
| Update frequency (milliseconds) | 450 | This controls how often the LEDs will be updated. |

[Return to "Configuring ESP32 Command Station Options"](#configuring-esp32-command-station-options)

### Accessory Decoder Management

The ESP32 Command Station provides a database of known Accessory Decoders, both DCC and virtual DCC decoders are supported. Virtual DCC decoders listen for a DCC accessory decoder events and translate into one (or more) OpenLCB events.

| Parameter | Default | Description |
| --------- | ------- | ----------- |
| Create accessory decoders on first use | Enabled | When enabled the ESP32 Command Station will intercept accessory decoder packets to create a record of the address as a DCC Accessory Decoder (turnout). These records can later be converted to virtual accessory decoders which are translated to one (or more) OpenLCB events. |

**NOTE**: There are advanced configuration options but they typically do not
require updates from the default values.

[Return to "Configuring ESP32 Command Station Options"](#configuring-esp32-command-station-options)

### Locomotive Roster

The ESP32 Command Station provides a database of known Locomotives, this is used by the web interface and OpenLCB Throttles.

| Parameter | Default | Description |
| --------- | ------- | ----------- |
| Enable automatic idle for newly created roster entries | Disabled | When enabled, any newly added locomotive will automatically be sent idle DCC packets (speed step zero). This setting can be adjusted for all locomotive roster entries individually. |
| Create roster entry for new locomotives on first use | Enabled | When enabled, any locomotive address requested that does not match an existing record will result in a new roster entry being created. |

**NOTE**: There are advanced configuration options but they typically do not
require updates from the default values.

[Return to "Configuring ESP32 Command Station Options"](#configuring-esp32-command-station-options)

### Crash Behavior

The ESP32 Command Station provides a database of known Locomotives, this is used by the web interface and OpenLCB Throttles.

| Parameter | Default | Description |
| --------- | ------- | ----------- |
| Collect dump on crash | Enabled | When enabled, the ESP32 Command Station will attempt to collect a crash dump which can be used in debugging crashes. |
| Halt startup when dump detected | Disabled | When enabled, if the ESP32 Command Station detects the presence of a crash dump it will halt normal startup and provide visual indication via the status LEDs in an alternating blink pattern (yellow and red for addressable RGB LEDs, on/off for discrete LEDs on the ESP32-S3).|
| Number of seconds to allow clearing of old core dump | 15 | Number of seconds to wait for the Factory Reset button to be pressed before automatically cleaning up any existing crash dump. |

**NOTE:** Starting with ESP-IDF v4.4 the ESP32 Command Station will automatically convert any detected crash dump to a text file and persist it on either SD or SPIFFS as `coredump.txt` and proceed with normal startup regardless of configuration options.

[Return to "Configuring ESP32 Command Station Options"](#configuring-esp32-command-station-options)

## Building

With the software configured you can now start [building](building.md).

##

[home](README.md)
