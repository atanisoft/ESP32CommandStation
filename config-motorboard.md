---
layout: default
---

# Configuring the MotorBoard module
Open include/Config_MotorBoard.h and adjust values to match your configuration, the defaults are set for an Arduino Uno form factor ESP32 with an attached Arduino motor shield.

| PARAM | Description |
| ----- | ----------- |
| MOTORBOARD_NAME_OPS | This is the name for the Operations track DCC signal. |
| MOTORBOARD_ENABLE_PIN_OPS | This is the ESP32 pin that is connected to the motor board "enable" pin, for Arduino motor shields this is typically "PWMA". |
| MOTORBOARD_CURRENT_SENSE_OPS | This is the ESP32 Analog input channel as defined below that is connected to the motor shield's current sense output pin, for Arduino motor shields this is usually A0. |
| MOTORBOARD_TYPE_OPS | This tells the Command Station what type of motor board is connected for the Operations track, this controls the current limiting configuration. Details for supported values are below. |
| DCC_SIGNAL_PIN_OPERATIONS | This is the DCC signal pin for the Operations track, this should be connected to the "direction" pin of the motor board, for Arduino motor shields this is the "DIRA" pin. |

| PARAM | Description |
| ----- | ----------- |
| MOTORBOARD_NAME_PROG | This is the name for the Programming track DCC signal. |
| MOTORBOARD_ENABLE_PIN_PROG | This is the ESP32 pin that is connected to the motor board "enable" pin, for Arduino motor shields this is typically "PWMB". |
| MOTORBOARD_CURRENT_SENSE_PROG | This is the ESP32 Analog input channel as defined below that is connected to the motor shield's current sense output pin, for Arduino motor shields this is usually A1. |
| MOTORBOARD_TYPE_PROG | This tells the Command Station what type of motor board is connected for the Programming track, this controls the current limiting configuration. Details for supported values are below. |
| DCC_SIGNAL_PIN_PROGRAMMING | This is the DCC signal pin for the Programming track, this should be connected to the "direction" pin of the motor board, for Arduino motor shields this is the "DIRB" pin. |

## Supported Motor Boards
When configuring the motor board module you will need to pick the type of motor board that is being used, the following table shows the supported options and their current limits:

| MOTORBOARD TYPE | Name | Max Current (Amps) | Current Limit (Amps) |
| --------------- | ---- | ------------------ | -------------------- |
| ARDUINO_SHIELD | Arduino Motor Shield (L298 compatible) | 2 Amp | 1.75 Amp |
| LMD18200 | LDM18200 Motor Driver | 3 Amd | 2.75 Amp |
| POLOLU | Pololu MC33926 Motor Driver (or Carrier) | 2.5 Amp | 2.25 Amp |
| BTS7960B_5A | BTS 7960B | 43 Amp | 5 Amp |
| BTS7960B_10A | BTS 7960B | 43 Amp | 10 Amp |

## Supported Analog Channels
With the ESP32 there are 16 analog inputs, unfortunately many of these are not reliable when WiFi is active and only those connected to ADC1 should be used. The table below provides the channel names and pin numbers for them:

| CHANNEL | PIN |
| ------- | --- |
| ADC1_CHANNEL_0 | 36 (listed as SVP or VP on some ESP32 boards) |
| ADC1_CHANNEL_3 | 39 (listed as SVN or VN on some ESP32 boards) |
| ADC1_CHANNEL_4 | 32 |
| ADC1_CHANNEL_5 | 33 |
| ADC1_CHANNEL_6 | 34 |
| ADC1_CHANNEL_7 | 35 |

Note that on the Arduino Uno form factor ESP32 boards, the A0 and A1 pins may connect to GPIO 0 and GPIO 4 and a pair of jumpers will be required for successful current sense reporting. On these boards a jump from A0 to A4 and A1 to A5 will work for ADC1_CHANNEL_0 and ADC1_CHANNEL_3 as listed above, or a jumper A0 to A2 and use ADC1_CHANNEL_7 for OPS and A1 to A3 and use ADC1_CHANNEL_6 for PROG.

## DCC Signal Splitting
Some motor boards require a split signal pair rather than a single pin. For these a circuit similar to the one below will be required:

![DCC Signal Split](dcc-signal-split.png)

## Arduino Motor Shield (L298)
If you are using an UNO formfactor ESP32 device this is by far the easiest to configure. Simply plug the motor shield into the ESP32 device and add two jumpers from A0 to A4 and A1 to A5. The jumpers are required due to the ESP32 devices typically having GPIO 0 and GPIO 2 in the A0 and A1 locations, these use ADC2 and are not usable.

## LMD18200 Motor Driver
The LMD18200 h-bridge IC operates similar to the L298 h-bridge used in the Arduino Motor Shield. However, it has a higher amperage rating.

| ESP32 pin | LMD18200 pin |
| --------- | ---------- |
| NO CONNECTION | VS (see note #1) |
| GND | GND |
| MOTORBOARD_ENABLE_PIN_MAIN (25) | PWM |
| MOTORBOARD_CURRENT_SENSE_MAIN (36/SVP/VP) | SENSE |
| DCC_SIGNAL_PIN_OPERATIONS (19) | DIRECTION |
| NO CONNECTION | THERMAL (see note #2) |

Note #1: The LMD18200 h-bridge is powered by the DCC track power supply, the VS pin is the positive power input and should not be connected to the ESP32 since it will have a much higher voltage than the ESP32 onboard voltage regulators can handle.
Note #2: The THERMAL pin is an active-low output from the h-bridge, it can be connected to an LED with appropriate resistor to light up when the h-bridge exceeds the thermal threshold specified in the datasheet. It can also be left unconnected.

## BTS7960B connections
The BTS7960B motor driver, also known at IBT_2, is a high amperage half h-bridge based motor driver. It is best suited as a standalone booster for the OPS DCC signal but can be used directly connected to the ESP32 Command Station. It is not known if this motor driver is suitable for use on the PROG track.

| ESP32 pin | BTS7960B pin |
| --------- | ---------- |
| 5V | VCC |
| GND | GND |
| MOTORBOARD_ENABLE_PIN_MAIN (25) | R_EN and L_EN |
| MOTORBOARD_CURRENT_SENSE_MAIN (36/SVP/VP) | R_IS and L_IS |
| DCC_SIGNAL_PIN_OPERATIONS (19) | The DCC Signal Split circuit above will be required for this board. |

## Pololu Motor Driver Shield connections
For the [Pololu MC33926 Motor Driver](https://www.pololu.com/product/2503) Shield you will need to make the following connections:

| ESP32 pin | Pololu pin |
| --------- | ---------- |
| 5V | VDD |
| GND | GND |
| MOTORBOARD_ENABLE_PIN_MAIN (25) | D2 and M1PWM |
| MOTORBOARD_CURRENT_SENSE_MAIN (36/SVP/VP) | M1FB |
| DCC_SIGNAL_PIN_OPERATIONS (19) | M1DIR |
| MOTORBOARD_ENABLE_PIN_PROG (23) | M2PWM |
| MOTORBOARD_CURRENT_SENSE_PROG (39/SVN/VN) | M2FB |
| DCC_SIGNAL_PIN_PROGRAMMING (18) | M2DIR |

WARNING: Be sure to remove the VIN/VOUT jumper otherwise the ESP32 may be damaged by the track power supply.

## Pololu Motor Driver Carrier connections
For the [Pololu MC33926 Motor Driver Carrier](https://www.pololu.com/product/1213) you will need to make the following connections:

| ESP32 pin | Pololu pin |
| --------- | ---------- |
| 5V | VDD |
| GND | GND |
| 5V | EN |
| MOTORBOARD_ENABLE_PIN_MAIN (25) | M1 PWM / INV D2 and M1 INV PWM / D1 (see note #1) |
| MOTORBOARD_CURRENT_SENSE_MAIN (36/SVP/VP) | M1 FB |
| DCC_SIGNAL_PIN_OPERATIONS (19) | M1 IN1 |
| see note #2 | M1 IN2 |
| MOTORBOARD_ENABLE_PIN_PROG (23) | M2 PWM / INV D2 and M2 INV PWM / D1 (see note #1) |
| MOTORBOARD_CURRENT_SENSE_PROG (39/SVN/VN) | M2 FB |
| DCC_SIGNAL_PIN_PROGRAMMING (18) | M2 IN1 |
| see note #2 | M1 IN2 |

Note #1: The M1 D1 and M2 D1 pins need to be pulled LOW when the PWM pin is pulled HIGH, this can be accomplished in a number of ways with the easiest likely being an NPN transistor. Failure to connect these pins will result in the track outputs remaining OFF.
Note #2: Similar to the BTS7960B motor driver, the [Pololu MC33926 Motor Driver Carrier](https://www.pololu.com/product/1213) requires an inverted DCC signal to be provided, the DCC Signal Split circuit can be used for this.

[Return to Building ESP32 Command Station](./building-esp32cs.html)