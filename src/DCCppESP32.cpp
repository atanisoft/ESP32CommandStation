/**********************************************************************
DCC++ BASE STATION FOR ESP32

COPYRIGHT (c) 2017 Mike Dunston

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see http://www.gnu.org/licenses
**********************************************************************/

/**********************************************************************

DCC++ESP32 BASE STATION is a C++ program written for the ESP32 using
PlatformIO IDE.

It allows an ESP32 with an Arduino Motor Shield (as well as others) to be used
as a fully-functioning digital command and control (DCC) base station for
controlling model train layouts that conform to current National Model Railroad
Association (NMRA) DCC standards.

This version of DCC++ BASE STATION supports:
  * 2-byte and 4-byte locomotive addressing
  * Simultaneous control of multiple locomotives
  * 128-step speed throttling
  * Cab functions F0-F28
  * Activate/de-activate accessory functions using 512 addresses, each with 4 sub-addresses
      - includes optional functionailty to monitor and store of the direction of any connected turnouts
  * Programming on the Main Operations Track
      - write configuration variable bytes
      - set/clear specific configuration variable bits
  * Programming on the Programming Track
      - write configuration variable bytes
      - set/clear specific configuration variable bits
      - read configuration variable bytes

DCC++ESP32 BASE STATION is controlled with simple text commands received via
a WiFi interface.  Users can control the base station via a built in web
interface accessible via mobile devices or a web browser.

With the exception of a standard 15V power supply that can be purchased in
any electronics store, no additional hardware is required.

REFERENCES:

  NMRA DCC Standards:          http://www.nmra.org/index-nmra-standards-and-recommended-practices
  Arduino:                     http://www.arduino.cc/
  Processing:                  http://processing.org/
  GNU General Public License:  http://opensource.org/licenses/GPL-3.0

BRIEF NOTES ON THE THEORY AND OPERATION OF DCC++ BASE STATION:

DCC++ESP32 BASE STATION for the ESP32 configures the four hardware timers, to
generate separate 0-3.3V unipolar signals that each properly encode zero and one
bits conforming with DCC timing standards.

Series of DCC bit streams are bundled into Packets that each form the basis of
a standard DCC instruction.  Packets are stored in Packet Registers that contain
methods for updating and queuing according to text commands sent by the user
(or another program) over the serial interface.  There is one set of registers
that controls the main operations track and one that controls the programming
track.

For the main operations track, packets to store cab throttle settings are stored
in virtual registers. It is generally considered good practice to continuously
send throttle control packets to every cab so that if an engine should
momentarily lose electrical connectivity with the tracks, it will very quickly
receive another throttle control signal as soon as connectivity is restored
(such as when a trin passes over  rough portion of track or the frog of a turnout).

DCC++ESP32 BASE STATION therefore sequentially loops through each locomotive
register in use every 50ms. As it loops through the locomotive registers, each
locomotive will be converted to a throttle control DCC packet and placed in the
Signal Generator queue for delivery. Each locomotive register should be for a
unique locomotive, if two throttles use the same locomotive register the
previous locomotive will be replaced.

An Arduino Motor Shield (or similar), powered by a standard 15V DC power supply
and attached to the ESP32, is used to transform the 0-3.3V DCC logic signals
produced by the ESP32's Timer interrupts into proper 0-15V bi-polar DCC signals.

This is accomplished on the ESP32 by connecting GPIO25 to the Motor Shield's
PWM-A input (pin 3 on Arduino), GPIO19 to the Motor Shield's DIRECTION-A input
(pin 12 on Arduino), GPIO23 to the Motor Shield's PWM-B input (pin 11 on
Arduino), GPIO18 to the Motor Shield's DIRECTION-B input (pin 13 on Arduino).

When configured as such, the CHANNEL A and CHANNEL B outputs of the Motor Shield
may be connected directly to the tracks. This software assumes CHANNEL A is
connected to the Main Operations Track, and CHANNEL B is connected to the
Programming Track.

DCC++ESP32 BASE STATION in split into multiple modules, each with its own header file:

  DCCppESP32:       declares required global objects and contains initial
										setup() and loop() functions.

  DCCppProtocol:    contains methods to read and interpret text commands,
										process those instructions.

	InfoScreen:       contains methods to display information on an OLED, LCD or
										Serial display of status, etc.

	Locomotive:       contains methods to convert Locomotive instructions into
										compatible DCC Packets for delivery on the MAIN operations
										DCC signal.

  MotorBoard:       contains methods to monitor and report the current drawn
										the Motor Shield, and shut down power if a short-circuit
										overload is detected.

	Outputs:          contains methods to configure one or more ESP32 pins as an
										output for your own custom use.

	Sensor:           contains methods to monitor and report on the status of
										optionally-defined sensors connected to various pins on the
										ESP32.

	SignalGenerator:  contains methods to generate the DCC signal for PROGRAMMING
										and OPERATIONS tracks, additional methods are present for
										reading and writing CV values on both PROGRAMMING and
										OPERATIONS tracks.

  Turnouts:         contains methods to operate and store the status of any
										optionally-defined turnouts controlled by a DCC stationary
										accessory decoder.

	WebServer:        contains methods to for the built in web server which
										provides a throttle, programming interface and ability to
										add/remove/update turnouts, sensors and output pins.

	WebSocketClient:  contains adapter code for WebSockets used by the web based
										throttle.

  WiFiInterface:		contains methods to connect the DCC++ESP32 BASE STATION to
										a wireless access point and manages the WebServer and
										WebSocket clients.

DCC++ESP32 BASE STATION is configured through the Config.h file that contains
all user-definable parameters except for Motor Shield declarations which are
present in DCCppESP32.cpp in the setup() method.
**********************************************************************/

#include "DCCppESP32.h"
#include "MotorBoard.h"
#include "Locomotive.h"
#include "Outputs.h"
#include "Turnouts.h"
#include "Sensors.h"
#include "SignalGenerator.h"

const char * buildTime = __DATE__ " " __TIME__;
Preferences configStore;
WiFiInterface wifiInterface;

void setup() {
	Serial.begin(115200L);
	log_i("DCC++ ESP starting up");
	// set up ADC1 here since we use it for all motor boards
	adc1_config_width(ADC_WIDTH_BIT_12);

	// Initialize the Configuration storage
	configStore.begin("DCCpp");

	InfoScreen::init();
	InfoScreen::printf(0, INFO_SCREEN_STATION_INFO_LINE, F("DCC++ESP: v%s"), VERSION);
#if INFO_SCREEN_STATION_INFO_LINE == INFO_SCREEN_IP_ADDR_LINE
	delay(250);
#endif
	wifiInterface.begin();
  MotorBoardManager::registerBoard(MOTORBOARD_CURRENT_SENSE_MAIN,
		MOTORBOARD_ENABLE_PIN_MAIN, MOTORBOARD_TYPE_MAIN, MOTORBOARD_NAME_MAIN);
  MotorBoardManager::registerBoard(MOTORBOARD_CURRENT_SENSE_PROG,
		MOTORBOARD_ENABLE_PIN_PROG, MOTORBOARD_TYPE_PROG, MOTORBOARD_NAME_PROG);
#if INFO_SCREEN_TRACK_POWER_LINE >= 0
	InfoScreen::printf(0, INFO_SCREEN_TRACK_POWER_LINE, F("TRACK POWER: OFF"));
#endif
	DCCPPProtocolHandler::init();
	OutputManager::init();
	TurnoutManager::init();
	SensorManager::init();
#if defined(S88_ENABLED) && S88_ENABLED
	S88SensorManager::init();
#endif
	configureDCCSignalGenerators();
	log_i("DCC++ READY!");
}

void loop() {
	wifiInterface.update();
	InfoScreen::update();
	MotorBoardManager::check();
	SensorManager::check();
	#if defined(S88_ENABLED) && S88_ENABLED
		S88SensorManager::check();
	#endif
	LocomotiveManager::update();
}
