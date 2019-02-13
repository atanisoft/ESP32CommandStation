/**********************************************************************
DCC++ BASE STATION FOR ESP32

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
// DEFINE WHICH PINS ARE USED BY MAIN/PROG MOTOR SHIELDS
//
// CURRENT SENSE PIN MAPPINGS:
// ADC1_CHANNEL_0 : 36
// ADC1_CHANNEL_1 : 37 -- NOT USABLE
// ADC1_CHANNEL_2 : 38 -- NOT USABLE
// ADC1_CHANNEL_3 : 39
// ADC1_CHANNEL_4 : 32
// ADC1_CHANNEL_5 : 33
// ADC1_CHANNEL_6 : 34
// ADC1_CHANNEL_7 : 35
//
// MOST ESP32 BOARDS DO NOT EXPOSE GPIO38 so ADC1_CHANNEL_2 MAY NOT BE USABLE.
//
// SUPPORTED MOTORBOARD TYPES:
// ARDUINO_SHIELD : Arduino Motor shield Rev3 based on the L298 chip. Max Output 2A per channel https://store.arduino.cc/usa/arduino-motor-shield-rev3
// POLOLU         : Pololu MC33926 Motor Driver (shield or carrier). Max Output 2.5A per channel https://www.pololu.com/product/1213 / https://www.pololu.com/product/2503
// BTS7960B_5A    : Infineon Technologies BTS 7960 Motor Driver Module. Max Output 5A (43A actual max) https://www.infineon.com/dgdl/bts7960b-pb-final.pdf
// BTS7960B_10A   : Infineon Technologies BTS 7960 Motor Driver Module. Max Output 10A (43A actual max) https://www.infineon.com/dgdl/bts7960b-pb-final.pdf

// MAIN TRACK MOTORBOARD NAME
#define MOTORBOARD_NAME_OPS "OPS"
// MAIN TRACK NOTORBOARD ENABLED PIN
#define MOTORBOARD_ENABLE_PIN_OPS 25
// MAIN TRACK MOTORBOARD CURRENT SENSE ADC PIN
#define MOTORBOARD_CURRENT_SENSE_OPS ADC1_CHANNEL_0
// MAIN TRACK MOTORBOARD MOTOR_BOARD_TYPE
#define MOTORBOARD_TYPE_OPS ARDUINO_SHIELD

// PROG TRACK MOTORBOARD NAME
#define MOTORBOARD_NAME_PROG "PROG"
// PROG TRACK NOTORBOARD ENABLED PIN
#define MOTORBOARD_ENABLE_PIN_PROG 23
// PROG TRACK MOTORBOARD CURRENT SENSE ADC PIN
#define MOTORBOARD_CURRENT_SENSE_PROG ADC1_CHANNEL_3
// PROG TRACK MOTORBOARD MOTOR_BOARD_TYPE
#define MOTORBOARD_TYPE_PROG ARDUINO_SHIELD

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE WHICH PINS ARE USED FOR DCC SIGNAL GENERATION
//
// OPERATIONS TRACK DCC SIGNAL PIN
#define DCC_SIGNAL_PIN_OPERATIONS 19

// PROGRAMMING TRACK DCC SIGNAL PIN
#define DCC_SIGNAL_PIN_PROGRAMMING 18

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE THE CURRENT SENSE ATTENUATION. THIS IS USED BY THE ADC SYSTEM TO SCALE
// THE CURRENT SENSE VALUES BASED ON THE MOTOR SHIELD.
//
// SUPPORTED VALUES: ADC_ATTEN_DB_11, ADC_ATTEN_DB_6, ADC_ATTEN_DB_2_5, ADC_ATTEN_DB_0
//
// IF LEFT UNDEFINED ADC_ATTEN_DB_11 WILL BE USED.

//#define ADC_CURRENT_ATTENUATION ADC_ATTEN_DB_11

/////////////////////////////////////////////////////////////////////////////////////
