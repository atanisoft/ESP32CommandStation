---
layout: default
---

# LCC Events
The ESP32 Command Station generates and consumes various LCC events.

### Well-known events CONSUMED
The following LCC well-known event IDs will be processed by the ESP32 Command Station

| Event Name | Event ID (hex) | Usage |
| ---------- | -------------- | ----- |
| EMERGENCY_OFF_EVENT | 01.00.00.00.00.00.FF.FF | Turns off track power |
| CLEAR_EMERGENCY_OFF_EVENT | 01.00.00.00.00.00.FF.FE | Turns on track power |
| EMERGENCY_STOP_EVENT | 01.00.00.00.00.00.FF.FD | Sends a broadcast DCC Emergency Stop packet to PROG and OPS track outputs |
| CLEAR_EMERGENCY_STOP_EVENT | 01.00.00.00.00.00.FF.FC | Currently Ignored |
| ACTIVATE_BASIC_DCC_ACCESSORY_EVENT | 01.01.02.00.00.FF.00.00 through 01.01.02.00.00.FF.07.FC (2044 accessory decoders supported) | Processed as an Accessory Decoder DCC packet on the OPS track output |
| INACTIVATE_BASIC_DCC_ACCESSORY_EVENT | 01.01.02.00.00.FE.00.00 through 01.01.02.00.00.FE.07.FC (2044 accessory decoders supported) | Processed as an Accessory Decoder DCC packet on the OPS track output |

## DCC Track Output Events PRODUCED
| Event Name | Event ID (hex) | Usage |
| ---------- | -------------- | ----- |
| event_short (OPS) | {node-id}.00<br/>(05.01.01.01.3F.00.00) | This event will be raised when the OPS track detects an over-current condition. |
| event_short_clear (OPS) | 05.01.01.01.3F.00.01 | This event will be raised when the OPS track detects an over-current condition has cleared. |
| event_shutdown (OPS) | 05.01.01.01.3F.00.02 | This event will be raised when the OPS track detects an over-current condition beyond 90% capacity. |
| event_shutdown_clear (OPS) | 05.01.01.01.3F.00.03 | This event will be raised when the OPS track detects an over-current condition beyond 90% capacity has cleared. |
| event_thermal_shutdown (OPS) | 05.01.01.01.3F.00.04 | This event will be raised when the OPS track detects a thermal shutdown of the h-bridge.<br/>Note: This is currently only supported for the LMD18200 h-bridge. |
| event_thermal_shutdown_clear (OPS) | 05.01.01.01.3F.00.05 | This event will be raised when the OPS track detects a thermal shutdown of the h-bridge has cleared.<br/>Note: This is currently only supported for the LMD18200 h-bridge. |
| event_short (PROG) | 05.01.01.01.3F.00.06 | This event will be raised when the PROG track detects an over-current condition. |
| event_short_clear (PROG) | 05.01.01.01.3F.00.07 | This event will be raised when the PROG track detects an over-current condition has cleared. |
| event_shutdown (PROG) | 05.01.01.01.3F.00.08 | This event will be raised when the PROG track detects an over-current condition beyond 90% capacity. |
| event_shutdown_clear (PROG) | 05.01.01.01.3F.00.09 | This event will be raised when the PROG track detects an over-current condition beyond 90% capacity has cleared. |

[Return to ESP32 Command Station](./index.html)