/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2017-2019 Mike Dunston

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

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE WHICH PINS ARE USED FOR S88 INTERFACE
//
// NOTE: The S88 Bus will require a data pin in addition to the pins that are common
// to all S88 Buses.
//

#define S88_CLOCK_PIN 17
#define S88_RESET_PIN 16
#define S88_LOAD_PIN 27

// S88 sensors are dynamically assigned based on the BUS ID * S88_MAX_SENSORS_PER_BUS + S88_FIRST_SENSOR
// This define allows shifting the S88 Sensors to start at a default value, you
// can start them at zero or as below S88_MAX_SENSORS_PER_BUS (default).
// S88_MAX_SENSORS_PER_BUS is defined as 512.
//#define S88_FIRST_SENSOR S88_MAX_SENSORS_PER_BUS

#define S88_ENABLED true

/////////////////////////////////////////////////////////////////////////////////////
