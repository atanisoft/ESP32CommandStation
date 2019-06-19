---
layout: default
---

# WiFi Module
The WiFi module is one of the core modules of the ESP32 Command Station. It allows the command station to be accessible via a WiFi network.

## Configuration

| PARAM | Description |
| ----- | ----------- |
| WIFI_SSID | This is the WiFi Access Point the Command Station should connect to on startup. |
| WIFI_PASSWORD | This is the password the Command Station should use when connecting to the WiFi Access Point defined above. |
| HOSTNAME | This is the hostname that the Command Station will advertise in mDNS and can be customized if you desire a different name than the default (DCCpp32) |

### Using a Static IP instead of DHCP
If you prefer to use a static IP instead of DHCP you can set the static IP details in these parameters:

| PARAM | Description |
| ----- | ----------- |
| WIFI_STATIC_IP_ADDRESS | This is the static IP for the Command Station. |
| WIFI_STATIC_IP_GATEWAY | This is the gateway IP that the Command Station will route all requests though. |
| WIFI_STATIC_IP_SUBNET | This is the subnet mask to use when configuring the static IP for the Command Station. |
| WIFI_STATIC_IP_DNS | This is an optional parameter and if left undefined the default DNS server is "8.8.8.8" (a Google provided DNS server). |

### Remote Sensor support (Optional)
By default the Command Station supports remote sensors reporting their status via the http/tcp interfaces. If you prefer to have the Command Station perform a scan for any remote sensors on startup you can configure this behavior with the following options:

| PARAM | Description |
| ----- | ----------- |
| SCAN_REMOTE_SENSORS_ON_STARTUP | By uncommenting this line it will trigger a WiFi AP scan on startup searching for any WiFi Access Points with the prefix defined in REMOTE_SENSORS_PREFIX. |
| REMOTE_SENSORS_PREFIX | WiFi SSID prefix used by remote sensors. Any remote sensors should use this as the start of their SoftAP name with a unique ID appended, the unique ID will be used as the sensor ID. |
| REMOTE_SENSORS_DECAY | This controls the lifespan of a remote sensor's "active" status, if the sensor does not report an update within this number of milliseconds the sensor will automatically transition to inactive status. |
| REMOTE_SENSORS_FIRST_SENSOR | This is the first sensor ID that will be used for any remote sensors the Command Station manages and reports via the ```<S>``` command. |

[Return to Building ESP32 Command Station](./building-esp32cs.html)<br/>
[Return to ESP32 Command Station](./index.html)