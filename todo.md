# ESP32 Command Station Feature/Bug Tracking list
This document tracks features and bug fixes that are planned.

## v1.5.0
The primary focus of v1.5.0 is improvements to the DCC signal code, RailCom
support, expanded LCC integration and general stability improvements.

### TODO for v1.5.0
Remaining tasks for v1.5.0 release:

-   [ ] DCC Signal Generation tasks:
    -   [ ] Test RailCom detector code.
-   [ ] Documentation tasks:
    -   [ ] Write migration guide for building, what has changed since v1.2.3, future of legacy protocols (DCC++, non-LCC JMRI)
-   [] Web Interface tasks:
    -   [ ] Test all endpoints to ensure proper functionality after httpd rework.
    -   [ ] Investigate websocket compatibility issue with iOS devices

### Completed for v1.5.0
These tasks have been completed and integrated for v1.5.0:

-   [x] Build changes:
    -   [x] Migrate to ESP-IDF v4.0 with cmake.
    -   [x] Add PCB build defaults for ESP-IDF.
    -   [x] Integrate "menuconfig" instead of requiring a lot of header file modifications.
-   [x] Configuration changes:
    -   [x] Dynamic config for LCC and WiFi stored on SD/SPIFFS.
    -   [x] SoftAP support for initial config and "non-home" network.
    -   [x] Split up monolithic json files to instead be individual files.
-   [x] DCC Signal Generation changes:
    -   [x] Allow adjustment of the DCC preamble bit count, default is 11 (OPS) and 22 (PROG).
    -   [x] Switch to using RMT for signal generation.
    -   [x] Remove hardware timer legacy code.
    -   [x] Refactor signal generation to use: dcc::Packet, UpdateLoop, RailcomHub, etc.
-   [x] DCC Turnout Management changes:
    -   [x] Switch fully to DCC address only support, for DCC++ compatibility decode the data and convert.
-   [x] JMRI Interface (DCC++ protocol) changes:
    -   [x] Convert to StateFlow pattern.
    -   [x] Replaced WiFiServer code with socket_listener from OpenMRNLite.
-   [x] Locomotive Roster changes:
    -   [x] Implement delete loco.
    -   [x] Implement auto-idle loco.
    -   [x] Roster Entries should contain function id mappings.
-   [x] LCC (OpenLCB) changes:
    -   [x] Add LCC metrics to the InfoScreen.
    -   [x] CS CDI web interface.
    -   [x] CS Node ID reset from web interface.
    -   [x] Force factory reset when node id changes.
    -   [x] Integrate CV memory space.
    -   [x] Integrate the WiFiConfiguration CDI element.
    -   [x] Migrate to Esp32WiFiManager instead of Arduino WiFi library.
    -   [x] Traction Protocol integration.
    -   [x] TrainSearch protocol support:
        -   [x] Basic integration is functional (loco request works for dynamic locos)
        -   [x] Persisted locos that are not active can not be requested from UWT-100.
    -   [x] Add fallback option for Esp32WiFiManager to not reboot the node when SSID connect fails and SoftAP is active.
    -   [x] Update Esp32WiFiManager to use SimpleStackBase instead of SimpleCanStack.
-   [x] S88 Sensor changes:
    -   [x] Add delay in s88SensorTask so that it gives time for other tasks to execute between updates.
    -   [x] Convert to use openlcb::Polling model rather than tasks.
    -   [x] Restore custom web rendering of sensor data.
-   [x] Status Display changes:
    -   [x] Move InfoScreen to StateFlow interface.
    -   [x] Status LED output for WiFi, OPS and PROG, EXT_1 and EXT_2. EXT_1 and EXT_2 unused currently.
-   [x] Web Interface changes:
    -   [x] Add busy/wait spinner for when data is loading (or being refreshed) in the web interface.
    -   [x] Add dialog for failed CS requests.
    -   [x] Add space in footer for clock so date/time are not smashed together.
    -   [x] Auto-connect WebSocket from initPage()
    -   [x] Auto-refresh of status pages
    -   [x] Configure Station SSID/PW
    -   [x] Fixed S88 section hiding.
    -   [x] Fixed S88 sensor bus creation/edit json payload parameters.
    -   [x] Hide power button for prog track when it is off.
    -   [x] Reintroduce websocket usage for pushing commands back to the CS.
    -   [x] Replace ArduinoJson with "JSON for Modern C++"
    -   [x] Replace webserver code with StateFlow based server.
    -   [x] Remove overall power on/off as it no longer makes sense.
    -   [x] Implement support for application/x-www-form-urlencoded POST/PUT data.
    -   [x] Entering loco name as "loco #3" results in "loco 33" being registered.
    -   [x] Delete of roster entry doesn't always force refresh of section.

## Future planning
These entries are items being tracked for a future release, these are not listed in priority order.

-   [] DCC Signal Generation changes:
    -   [ ] Concurrency guards for ProgrammingTrackBackend.
    -   [ ] Continue sending eStop packet until eStop is cleared.
    -   [ ] Reimplement DCC Prog Track interface so it supports multiple requests (serialized).
    -   [ ] Introduced priority queue mechanism for DCC packets.
    -   [ ] Expire inactive locos that are not auto-idle.
-   [ ] GPIO changes:
    -   [ ] Expose Outputs, Sensors, S88 events on LCC.
    -   [ ] Add S88 sensor data to InfoScreen status line, 16 sensor output rotation.
    -   [ ] Convert Sensors to use openlcb::Polling model rather than tasks.
-   [] LCC (OpenLCB) changes:
    -   [ ] Rewrite HW Can driver.
    -   [ ] Broadcast events for turnout state change.
    -   [ ] Discard turnout events when turnout already in expected state (drop duplicate events).
    -   [ ] Expose PROG track via CDI (needed for UWT-100 support).
    -   [ ] Reimplement Loco Consist leveraging LCC Traction Consist functionality.
    -   [ ] Connect Loco CDI to TrainDB.
-   [ ] Miscellaneous changes:
    -   [ ] Switch to shared_ptr instead of raw pointers.
    -   [ ] WiThrottle support (https://github.com/atanisoft/ESP32CommandStation/issues/15)
    -   [ ] Combine usages of openlcb::RefreshLoop.
-   [ ] Locomotive Roster changes:
    -   [ ] Expose db entries via R/W CDI.
-   [ ] Nextion Interface changes:
    -   [ ] rewrite from scratch.
    -   [x] Lock to title screen until WiFi connects.
    -   [x] Switch to timer based speed increment/decrement on the nextion mcu side.
    -   [ ] Add support for Nextion Upload via OTA in NeoNextion.
    -   [ ] Add notification of turnout state change when changed external to the nextion code.
    -   [ ] Auto turn on of track power from Nextion when interacting with loco/turnouts.
    -   [ ] Adjust screen size detection for 5" and 7" displays which send the same screen type prefix.
    -   [ ] Implement automatic resolver for component id during page initialization so we can drop component IDs from the argument list.
    -   [ ] Move screen detection code into NeoNextion lib.
    -   [ ] Reimplement NeoNextion using OpenMRN StateFlow pattern with uart select() usage.
    -   [ ] Replace Routes page with a Setup page which will include route creation.
-   [ ] Over-the-air (OTA) firmware update changes:
    -   [ ] Support via JMRI / LCC using OpenMRN Bootloader.
    -   [ ] Return to normal mode on Nextion when OTA fails.
-   [ ] Web Interface changes:
    -   [ ] Add strict validation of input parameter data.
    -   [ ] Expose Loco Consist creation.
    -   [ ] Expose Loco function map names.
    -   [ ] Add urldecode to table rendering code.

## PCB related tasks:
-   [ ] LMD18200 heatsink evaluation:
    https://www.mouser.com/ProductDetail/593202B03500G
    https://www.mouser.com/ProductDetail/MCP9701AT-E-TTVAO