---
layout: default
---

# LCC / Layout Command Control
LCC is Layout Command Control, an NMRA standard. The ESP32 Command Station can connect with LCC devices either via WiFi or physical CAN bus connections. Details about LCC can be found [here](https://openlcb.org/).

## Supported LCC well-known event IDs
When this module is used the Command Station will receive and respond to certain LCC well-known events listed below with their function:

| Event Name | Event ID (hex) | Description |
| ---------- | -------------- | ----------- |
| EMERGENCY_OFF_EVENT | 01.00.00.00.00.00.FF.FF | Turns off track power |
| CLEAR_EMERGENCY_OFF_EVENT | 01.00.00.00.00.00.FF.FE | Turns on track power |
| EMERGENCY_STOP_EVENT | 01.00.00.00.00.00.FF.FD | Sends a broadcast DCC Emergency Stop packet to PROG and OPS track outputs |
| CLEAR_EMERGENCY_STOP_EVENT | 01.00.00.00.00.00.FF.FC | Currently Ignored |
| ACTIVATE_BASIC_DCC_ACCESSORY_EVENT | 01.01.02.00.00.FF.00.00 through 01.01.02.00.00.FF.07.FC (2044 accessory decoders supported) | Processed as an Accessory Decoder DCC packet on the OPS track output |
| INACTIVATE_BASIC_DCC_ACCESSORY_EVENT | 01.01.02.00.00.FE.00.00 through 01.01.02.00.00.FE.07.FC (2044 accessory decoders supported) | Processed as an Accessory Decoder DCC packet on the OPS track output |
| IS_TRAIN_EVENT | 01.01.00.00.00.00.03.03 | To be implemented. This will allow control of locomotives via the Traction specification |

## LCC Configuration

| PARAM | Description |
| ----- | ----------- |
| LCC_NODE_ID | This is the unique 64bit node ID for the ESP32 Command Station. You are encouraged to have your own unique ID but it is not mandatory. You can get a unique ID range [here](https://registry.openlcb.org/requestuniqueidrange) and assign one ID from the range here. The default value is 05.01.01.01.3F.00 (without dots) which indicates it is the first ID in the 05.01.01.01.3F.{00-FF} range. |
| LCC_CAN_RX_PIN | This is the pin connected to the CAN transceiver RX pin. This is optional, if left as -1 the CAN connection will not be configured. |
| LCC_CAN_TX_PIN | This is the pin connected to the CAN transceiver TX pin. This is optional, if left as -1 the CAN connection will not be configured. |

[Return to Building ESP32 Command Station](./building-esp32cs.html)<br/>
[Return to ESP32 Command Station](./index.html)