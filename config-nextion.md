---
layout: default
---

# Nextion Module
The Nextion interface is an optional module that allows to control up to three trains and manage turnouts. In the future it will be possible to use this as a general command station management interface.

## Configuration

| PARAM | Description |
| ----- | ----------- |
| NEXTION_UART_NUM | This defines which of the built in hardware UART devices will be used, this can be set to 1 or 2. UART 0 is used internally for the serial console logging. |
| NEXTION_UART_BAUD | This is the speed at which the ESP32 will talk to the Nextion screen, this should remain at 115200 unless the HMI file is also updated to reflect a different speed. |
| NEXTION_UART_RX_PIN | This is the ESP32 pin connected to the Nextion TX pin (blue wire). Default is 14, any unused pin can be used. |
| NEXTION_UART_TX_PIN | This is the ESP32 pin connected to the Nextion RX pin (yellow wire). Default is 27, any unused pin can be used. |

## Supported Displays
There are many sizes of Nextion displays available on the market. At this time only the following displays are supported:

| Size (inches) | Type | Part # (prefix only) | HMI File |
| ------------- | ---- | -------------------- | -------- |
| 3.2 | Basic | NX4024T | nextion/ESP32CS_Nextion3.2_B.HMI |
| 3.2 | Enhanced | NX4024K | nextion/ESP32CS_Nextion3.2_B.HMI |
| 3.5 | Basic | NX4832T | nextion/ESP32CS_Nextion3.5_B.HMI |
| 3.5 | Enhanced | NX4832K | nextion/ESP32CS_Nextion3.5_E.HMI |
| 5.0 | Basic | NX8048T | nextion/ESP32CS_Nextion5.0_B.HMI |
| 5.0 | Enhanced | NX8048K | nextion/ESP32CS_Nextion5.0_E.HMI |

For each of these, the HMI file will need to be compiled and uploaded to the Nextion device using the [Nextion Editor](https://nextion.itead.cc/resources/download/nextion-editor/).

[Return to Building ESP32 Command Station](./building-esp32cs.html)<br/>
[Return to ESP32 Command Station](./index.html)