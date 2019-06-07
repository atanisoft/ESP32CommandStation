# ESP32 DCC Command Station PCB
The ESP32 DCC Command Station works well with off the shelf components but it doesn't provide a standard design for integrating all of the components. This PCB is designed to provide a standard board for the ESP32 DCC Command Station project based on either the TTGO-T1 ESP32 board or the ESP32 DevKit-C board since these provide most of the required functionality and expose virtually all usable GPIO pins.

## PCB Features
The PCB provides two RJ45 jacks, MCP2551 IC for the LCC integration and onboard h-bridges for for DCC signal generation.
In addition to LCC the PCB provides headers for I2C (2x) and a Serial1 (for Nextion, HC12 or LocoNet).

## TTGO-T1 Pin usage map
0 : FREE (restrictions apply)
1 : UART0 TX
2 : SD-MISO
3 : UART0 RX
4 : CAN RX
5 : CAN TX
6-11 : NOT AVAILABLE (connected to on chip flash)
12 : FREE (restrictions apply)
13 : SD-CS
14 : SD-CLK
15 : SD-MOSI
16 : OPS DCC ENABLE
17 : OPS DCC SIGNAL
18 : PROG DCC ENABLE
19 : PROG DCC SIGNAL
20 : NOT AVAILABLE
21 : I2C - SCL
22 : ONBOARD LED, possible usage for RGB LED (WS2811 or similar) for CS status.
23 : I2C - SDA
24 : NOT AVAILABLE
25 : Serial1 RX
26 : Serial1 TX
27 : RAILCOM ENABLE
28-31 : NOT AVAILABLE
32 : OPS BRAKE
33 : RAILCOM DATA
34 : RAILCOM SHORT
35 : OPS-THERM
SVP (36) : OPS CURRENT SENSE (INPUT ONLY)
37 : NOT AVAILABLE
38 : NOT AVAILABLE
SVP (39) : PROG CURRENT SENSE (INPUT ONLY)

## PCB Schematic
TBD

## DCC signal strength
The on-board LMD18200 chip will provide up to 3A for the OPS output but the code limits this to ~2.75A. This should be sufficient for most scales when splitting a larger layout into smaller segments powered by boosters.

## CS Status via RGB LED
Using GPIO 22 it is possible to wire an RGB LED using WS2811 (or similar) to display the CS status, current design considerations are as follows:

LED 1: WiFi status (GREEN = connected, RED = disconnected, FLASHING GREEN = connecting, FLASHING YELLOW = AP not found, YELLOW = connection failed)
LED 2: OPS track status (GREEN = ON, BLACK = OFF, RED = FAULT/SHORT, FLASHING RED = THERMAL FAULT)
LED 3: PROG track status (GREEN = ON, BLACK = OFF, RED = FAULT/SHORT)

Additional status LEDs could be added if needed.