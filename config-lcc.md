---
layout: default
---

# LCC / Layout Command Control
LCC is Layout Command Control, an NMRA standard, that allows compatible devices to interact on a dedicated bus seperate from the DCC bus. Common usages of this would be for track side signals and block detection. The ESP32 Command Station can connect with LCC devices either via WiFi or physical CAN bus connections. Details about LCC can be found [here](https://openlcb.org/).

## Event IDs
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

## Configuration

| PARAM | Description |
| ----- | ----------- |
| LCC_NODE_ID | This is the unique 64bit node ID for the ESP32 Command Station. You are encouraged to have your own unique ID but it is not mandatory. You can get a unique ID range [here](https://registry.openlcb.org/requestuniqueidrange) and assign one ID from the range here. The default value is 05.01.01.01.3F.00 (without dots) which indicates it is the first ID in the 05.01.01.01.3F.{00-FF} range. |
| LCC_CAN_RX_PIN | This is the pin connected to the CAN transceiver RX pin. This is optional, if left as NOT_A_PIN the CAN connection will not be configured. |
| LCC_CAN_TX_PIN | This is the pin connected to the CAN transceiver TX pin. This is optional, if left as NOT_A_PIN the CAN connection will not be configured. |
| LCC_CONFIG_DIR | This is the directory where LCC configuration data will be stored. |
| LCC_CDI_FILE | This is where the CDI.xml file will be generated, this should be under the LCC_CONFIG_DIR. |
| LCC_CONFIG_FILE | This is where OpenMRNLite will store the node configuration data, this should be under the LCC_CONFIG_DIR. |
| LCC_USE_SPIFFS | This configures the usage of SPIFFS for the persistence layer used by the LCC interface. Currently this is the only option availble for persistence, in the near future storage on an SD card will be supported. |
| LCC_FORCE_FACTORY_RESET_ON_STARTUP | This flag will trigger a forced factory reset of the node by removing LCC_CDI_FILE and LCC_CONFIG_FILE from the persistence layer. This should generally not be enabled. |

## CDI Data
The CDI currently does not contain many configurable parameters outside of the "WiFi Configuration" node which controls the node WiFi LCC behavior. The default behavior is as a client searching for an uplink hub.

### WiFi Configuration

| PARAM | Description |
| ----- | ----------- |
| sleep | Enabling this option will allow the ESP32 to power down the WiFi radio to conserve power. This is generally not recommended unless the node is powered via a battery or similarly limited supply. For the Command Station this should not be enabled. |
| hub | This contains the configuration parameters to allow the ESP32 to be an LCC hub. |
| uplink | This contains the uplink configuration parameters. |

### Hub Configuration
When the ESP32 is operating as an LCC hub, it will advertise itself via mDNS as available for nodes to connect to. Due to the memory constraints of the ESP32 it is generally not recommended to enable the hub functionality.

| PARAM | Description |
| ----- | ----------- |
| enable | Enabling this will enable the hub functionality on the ESP32 node. Keep in mind that the ESP32 is a limited resources device and the maximum number of incoming connections is limited. |
| port | This is the TCP/IP port which the hub will be listening on, default is 12021. |
| service_name | This is the mDNS service name that the hub will advertise when it is active, default is _openlcb-can._tcp |

### Uplink Configuration
By default the ESP32 will attempt to search for an uplink hub. This set of configuration parameters controls how/when this behavior is used.

| PARAM | Description |
| ----- | ----------- |
| search_mode | This controls the uplink behavior, default is "auto, manual" which means search for an mDNS advertised hub and fallback to manual configuration if none are found. Supported values: "auto, manual", "manual, auto", "auto only", "manual only". |
| manual_address | This is the uplink hub that will be used when the ESP32 switches to manual mode, this contains an ip_address (hostname) and port (default: 12021). |
| auto_address | This contains the mDNS service name and a preferred hostname to connect to when using the auto mode. Default service name is "_openlcb-can._tcp" and hostname is blank. |
| reconnect | When enabled the ESP32 will automatically try and connect to the last hub that it successfully connected to. This is enabled by default. |

[Return to Building ESP32 Command Station](./building-esp32cs.html)<br/>
[Return to ESP32 Command Station](./index.html)