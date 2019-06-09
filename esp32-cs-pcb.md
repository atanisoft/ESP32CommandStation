---
layout: default
---

# ESP32 Command Station PCB
The ESP32 Command Station works quite well with off the shelf components but there isn't a standard design for integrating all of the components. This PCB provides a standard board for the ESP32 Command Station project based on either the TTGO-T1 ESP32 board or the ESP32 DevKit-C board with all components on-board to provided:
1) OPS Track output (3 Amp) supporting RailCom detection.
2) PROG Track output (250 mA).
3) Native LCC interface with two RJ45 ports.
4) Two I2C interfaces for OLED/LCD display usage (additional usages may be developed in the future).
5) UART interface for LocoNet or Nextion display usage.
6) Integrated JTAG interface for easier debugging and flashing of firmware.
7) Single Power Supply interface with reverse polarity protection.

## PCB Schematic and Bill of Materials
The PCB schematic is available [here (SVG)](ESP32-CS-LMD18200.svg) or [here (PDF)](ESP32-CS-LMD18200.pdf).
The BOM can be viewed [here](ESP32-CS-BOM.html).

## Command Station PCB Kits
Kits will be made available soon in three varieties:
1) Fully assembled and tested.
2) Partially assembled and tested (SMD components will be soldered to the PCB and tested for proper functionality).
3) PCB and parts only (no assembly or testing).

## ESP32 Pins used on the PCB
The PCB uses almost every pin available on the TTGO-T1/DevKit-C module.

| GPIO Pin | Usage |
| -------- | ----- |
| 0 | FREE (restrictions apply) |
| 1 | UART0 TX |
| 2 | SD-MISO |
| 3 | UART0 RX |
| 4 | CAN RX |
| 5 | CAN TX |
| 6-11 | NOT AVAILABLE (connected to on chip flash) |
| 12 | FREE (restrictions apply) |
| 13 | SD-CS |
| 14 | SD-CLK |
| 15 | SD-MOSI |
| 16 | OPS DCC ENABLE |
| 17 | OPS DCC SIGNAL |
| 18 | PROG DCC ENABLE |
| 19 | PROG DCC SIGNAL |
| 20 | NOT AVAILABLE |
| 21 | I2C - SCL |
| 22 | ONBOARD LED, CS Status LED. |
| 23 | I2C - SDA |
| 24 | NOT AVAILABLE |
| 25 | Serial1 RX |
| 26 | Serial1 TX |
| 27 | RAILCOM ENABLE |
| 28-31 | NOT AVAILABLE |
| 32 | OPS BRAKE |
| 33 | RAILCOM DATA |
| 34 | RAILCOM SHORT |
| 35 | OPS-THERM |
| 36 (SVP) | OPS CURRENT SENSE (INPUT ONLY) |
| 37 | NOT AVAILABLE |
| 38 | NOT AVAILABLE |
| 39 (SVP) | PROG CURRENT SENSE (INPUT ONLY) |

## DCC signal strength
The on-board LMD18200 h-bridge will provide up to 3A for the OPS track output, the Command Station code will limit this to around 2.75A for safety. This should be sufficient for most scales when splitting a larger layout into smaller segments powered by boosters. The on-board L298 h-bridge provides up to 250mA for the PROG track output as well as provides the LCC ALT_L/ALT_H signals.

### DCC Power Supply requirements
The DCC-POWER connections supply power to the DCC track outputs as well as the PCB itself. It is recommended to use a 14-18v DC power supply rated for 5 Amps. The LMD18200 and L298 h-bridges used by the PCB will reduce the track output voltage by approximately 2v compared to the DCC-POWER supply voltage.

## Command Station Status LED
GPIO 22 is connected to three APA106/WS2812 (or similar) RGB LEDs. APA106 and WS2812 use different pin assignments for GND and 5V, care must be taken to account for this.

| LED | Usage | Output Details |
| --- | ----- | -------------- |
| 1 | WiFi Status | Green - Connected<br/>Green (flashing) - Connecting<br/>Red - Disconnected<br/>Yellow - Connection failed<br/>Yellow (flashing) - AP not found |
| 2 | OPS Track Status | Green - On<br/> Clear/Off - Off<br/>Red - Fault/Short<br/> Red (flashing) - H-bridge thermal warning |
| 2 | PROG Track Status | Green - On<br/> Clear/Off - Off<br/>Red - Fault/Short |

Flashing LEDs use an on/off frequency of 450ms-500ms.

### LED 5V and GND pin connections
There are currently two common pin assignments for the RGB LEDs supported by the Command Station. The 5V and GND pins are swapped depending on the specific type of LED being used and who manufactured it. The WS2812 based LEDs are typically (1) DATA IN, (2) GND, (3) 5V, (4) DATA OUT. The APA106 LEDs are typically (1) DATA IN, (2) 5V, (3) GND, (4) DATA OUT.

The LED-PIN2 and LED-PIN3 solder jumper pads on the back side of the PCB can be used to switch the LED pins to match the LEDs being used, these default to the APA106 pin assignment.

The LEDs included in the PCB kit are F5 diffused APA106 LEDs.

## Expansion interfaces
The PCB comes equipped with two I2C ports, UART1, OPS-EXT, SD-Module and JTAG. These have can a variety of uses with the most common listed below:

| Port | Usage |
| ---- | ----- |
| I2C | OLED or LCD Screen. |
| UART1 | Nextion Interface, LocoNet Interface. |
| SD-Module | When using the ESP32 DevKit-C board it is highly recommended to use an SD card adapter module to reduce wear on the ESP32 built-in Flash (used for SPIFFS). |
| JTAG | Used as a debugger or firmware flashing interface, this has overlapping pins with the SD-Module (and SD card slot on the TTGO-T1). |
| OPS-EXT | This allows for an expansion board to provide the OPS track signal and DCC power via other h-bridges that will not be monitored by the Command Station. Note: The signal provided by this interface will have the RailCom cut-out and will be in a LOW state for up to 488uS between DCC packets. |