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
// NOTE: GPIO 37 and GPIO 38 aare not usable as they are connected to GPIO 36 and
// GPIO 39 internally with a capacitor. Therefore ADC1_CHANNEL_1 and ADC1_CHANNEL_2
// are not suitable for usage, regardless of if the ESP32 board exposes these pins.
//
// SUPPORTED MOTORBOARD TYPES:
// ARDUINO_SHIELD : Arduino Motor shield Rev3 based on the L298 chip. Max Output 2A per channel https://store.arduino.cc/usa/arduino-motor-shield-rev3
// LMD18200       : Texas Instruments LMD18200 55V 3A h-bridge. http://www.ti.com/lit/ds/symlink/lmd18200.pdf
// POLOLU         : Pololu MC33926 Motor Driver (shield or carrier). Max Output 2.5A per channel https://www.pololu.com/product/1213 / https://www.pololu.com/product/2503
// BTS7960B_5A    : Infineon Technologies BTS 7960 Motor Driver Module. Max Output 5A (43A actual max) https://www.infineon.com/dgdl/bts7960b-pb-final.pdf
// BTS7960B_10A   : Infineon Technologies BTS 7960 Motor Driver Module. Max Output 10A (43A actual max) https://www.infineon.com/dgdl/bts7960b-pb-final.pdf

// MAIN TRACK MOTORBOARD NAME
#define MOTORBOARD_NAME_OPS "OPS"
// MAIN TRACK NOTORBOARD ENABLED PIN
// MAIN TRACK NOTORBOARD ENABLED PIN
#ifdef CONFIG_MOTORBOARD_ENABLE_PIN_OPS
  #define MOTORBOARD_ENABLE_PIN_OPS CONFIG_MOTORBOARD_ENABLE_PIN_OPS
#else
  #define MOTORBOARD_ENABLE_PIN_OPS 25
#endif
// MAIN TRACK MOTORBOARD CURRENT SENSE ADC PIN
#ifdef CONFIG_MOTORBOARD_CURRENT_SENSE_OPS
  #define MOTORBOARD_CURRENT_SENSE_OPS ((adc1_channel_t)CONFIG_MOTORBOARD_CURRENT_SENSE_OPS)
#else
  #define MOTORBOARD_CURRENT_SENSE_OPS ADC1_CHANNEL_0
#endif
// MAIN TRACK MOTORBOARD MOTOR_BOARD_TYPE
#ifdef CONFIG_MOTORBOARD_TYPE_OPS
  #define MOTORBOARD_TYPE_OPS ((MOTOR_BOARD_TYPE)CONFIG_MOTORBOARD_TYPE_OPS)
#else
  #define MOTORBOARD_TYPE_OPS ARDUINO_SHIELD
#endif

// PROG TRACK MOTORBOARD NAME
#define MOTORBOARD_NAME_PROG "PROG"
// PROG TRACK NOTORBOARD ENABLED PIN
#ifdef CONFIG_MOTORBOARD_ENABLE_PIN_PROG
  #define MOTORBOARD_ENABLE_PIN_PROG CONFIG_MOTORBOARD_ENABLE_PIN_PROG
#else 
  #define MOTORBOARD_ENABLE_PIN_PROG 23
#endif

// PROG TRACK MOTORBOARD CURRENT SENSE ADC PIN
#ifdef CONFIG_MOTORBOARD_CURRENT_SENSE_PROG
  #define MOTORBOARD_CURRENT_SENSE_PROG (adc1_channel_t)CONFIG_MOTORBOARD_CURRENT_SENSE_PROG
#else
  #define MOTORBOARD_CURRENT_SENSE_PROG ADC1_CHANNEL_3
#endif

// PROG TRACK MOTORBOARD MOTOR_BOARD_TYPE
#ifdef CONFIG_MOTORBOARD_TYPE_PROG
  #define MOTORBOARD_TYPE_PROG ((MOTOR_BOARD_TYPE)CONFIG_MOTORBOARD_TYPE_PROG)
#else
  #define MOTORBOARD_TYPE_PROG ARDUINO_SHIELD
#endif

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE WHICH PINS ARE USED FOR DCC SIGNAL GENERATION
//
// OPERATIONS TRACK DCC SIGNAL PIN
#ifdef CONFIG_DCC_SIGNAL_PIN_OPERATIONS
  #define DCC_SIGNAL_PIN_OPERATIONS CONFIG_DCC_SIGNAL_PIN_OPERATIONS
#else
  #define DCC_SIGNAL_PIN_OPERATIONS 19
#endif

// PROGRAMMING TRACK DCC SIGNAL PIN
#ifdef CONFIG_DCC_SIGNAL_PIN_PROGRAMMING
  #define DCC_SIGNAL_PIN_PROGRAMMING CONFIG_DCC_SIGNAL_PIN_PROGRAMMING
#else
  #define DCC_SIGNAL_PIN_PROGRAMMING 18
#endif

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
