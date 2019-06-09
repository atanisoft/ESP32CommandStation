---
layout: default
---

# Configuring the Nextion module
The Nextion interface is an optional module that allows to control up to three trains and manage turnouts. In the future it will be possible to use this as a general command station management interface.

| PARAM | Description |
| ----- | ----------- |
| NEXTION_UART_NUM | This defines which of the built in hardware UART devices will be used, this can be set to 1 or 2. UART 0 is used internally for the serial console logging. |
| NEXTION_UART_BAUD | This is the speed at which the ESP32 will talk to the Nextion screen, this should remain at 115200 unless the HMI file is also updated to reflect a different speed. |
| NEXTION_RX_PIN | This is the ESP32 pin connected to the Nextion RX pin. Default is 14, any unused pin can be used. |
| NEXTION_TX_PIN | This is the ESP32 pin connected to the Nextion TX pin. Default is 27, any unused pin can be used. |

Note: currently only Nextion 3.2" displays are supported and the nextion/DCCppESP32.hmi file will need to be compiled and uploaded to the Nextion screen via the Nextion Editor.

[Return to Building ESP32 Command Station](./building-esp32cs.html)<br/>
[Return to ESP32 Command Station](./index.html)