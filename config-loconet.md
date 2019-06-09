---
layout: default
---

# Configuring the LocoNet module
If you have an existing LocoNet bus the ESP32 Command Station can interact with these devices. Currently only the following OP Codes are implemented:

## LocoNet Configuration

| PARAM | Description |
| ----- | ----------- |
| LOCONET_RX_PIN | This should be connected to the RX input from the LocoNet interface. |
| LOCONET_TX_PIN | This should be connected to the TX output from the LocoNet interface. |
| LOCONET_UART | This is the hardware UART on the ESP32 to use for the LocoNet interface. |

## LocoNet Interface circuit
John Plocher created the circuit shown below, it works great for a DIY interface as it only requires a handful of components.
![LocoNet Interface](LocoNetInterface.png)

Newer versions of this circuit can be found on his [website](http://www.spcoast.com/wiki/index.php/LocoShield), they use a different IC and have not been tested.

## LocoNet Supported OpCodes
| OPC | Description |
| --- | ----------- |
| OPC_GPON | Turns track power on. |
| OPC_GPOFF | Turns track power off. |
| OPC_IDLE | Issues an emergency stop (track power will remain on if already on). |
| OPC_LOCO_ADR | Assigns locomotive to a throttle. |
| OPC_LOCO_SPD | Adjusts locomotive speed from a throttle. |
| OPC_LOCO_DIRF | Adjusts locomotive direction and functions 0-4 from a throttle. |
| OPC_LOCO_SND | Adjusts locomotive functions 5-8 from a throttle. |
| OPC_WR_SL_DATA | This is currently limited to only the Programming Track support. Fast Clock or other features may be implemented in the future. |

The following are not implemented but are planned:

| OPC | Description |
| --- | ----------- |
| OPC_INPUT_REP | Sensor inputs |
| OPC_SW_REQ | Turnout requests |
| OPC_SW_REP | Turnout report |


[Return to Building ESP32 Command Station](./building-esp32cs.html)<br/>
[Return to ESP32 Command Station](./index.html)