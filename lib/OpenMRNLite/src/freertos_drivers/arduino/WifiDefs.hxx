#ifndef _FREERTOS_DRIVERS_COMMON_WIFIDEFS_HXX_
#define _FREERTOS_DRIVERS_COMMON_WIFIDEFS_HXX_

/// Wifi not associated to access point: continuous short blinks.
#define WIFI_BLINK_NOTASSOCIATED  0b1010
/// Waiting for IP address: double short blink, pause, double short blink, ...
#define WIFI_BLINK_ASSOC_NOIP  0b101000
/// Connecting to hub: long blinks
#define WIFI_BLINK_CONNECTING  0b1100
/// Connecting to hub: long blinks
#define WIFI_BLINK_FAILED  0b10101100

enum class WlanState : uint8_t
{
    OK = 0,
    NOT_ASSOCIATED = 1,
    NO_IP,
    NO_CONNECTION,
    CONNECTING,
    MDNS_LOOKUP = 5,
    CONNECT_MDNS,
    CONNECT_STATIC,
    CONNECT_FAILED,
    CONNECTION_LOST,
    UPDATE_DISPLAY = 20,
};

/** Operating Role.
 */
enum class WlanRole : uint8_t
{
    UNKNOWN = 0, /**< Wi-Fi station mode */
    STA,         /**< Wi-Fi station mode */
    AP           /**< Wi-Fi access point mode */
};

enum class CountryCode : uint8_t
{
    US, ///< United States
    EU, ///< European Union
    JP, ///< Japan
    UNKNOWN, ///< unknown country code
};

extern "C" {
/// Name of wifi accesspoint to connect to.
extern char WIFI_SSID[];
/// Password of wifi connection. If empty, use no encryption.
extern char WIFI_PASS[];
/// Hostname at which the OpenLCB hub is at.
extern char WIFI_HUB_HOSTNAME[];
/// Port number of the OpenLCB hub.
extern int WIFI_HUB_PORT;
}

#endif // _FREERTOS_DRIVERS_COMMON_WIFIDEFS_HXX_
