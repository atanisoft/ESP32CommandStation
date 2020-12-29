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
-   [ ] OpenLCB (LCC):
    -   [ ] Support via JMRI / LCC using OpenMRN Bootloader.

### Completed for v1.5.0
These tasks have been completed and integrated for v1.5.0:

-   [x] Build / Configuration:
    -   [x] Added default configurations for ESP-IDF functionality that is required for stable operations.
    -   [x] Added PCB supplemental configuration defaults for ESP-IDF.
    -   [x] Migrate to ESP-IDF v4.0 with cmake.
    -   [x] Migrate to "menuconfig" for CS configuration.
    -   [x] Split up monolithic json files to instead be individual files.
-   [x] DCC Signal Generation:
    -   [x] Allow adjustment of the DCC preamble bit count, default is 11 (OPS) and 22 (PROG).
    -   [x] Switch to using RMT for signal generation.
    -   [x] Remove hardware timer legacy code.
    -   [x] Refactor signal generation to use: dcc::Packet, UpdateLoop, RailcomHub, etc.
-   [x] DCC Turnout Management:
    -   [x] Switch fully to DCC address only support, for DCC++ compatibility decode the data and convert.
    -   [x] Automatic persistence of Turnout definitions (includes last known state).
-   [x] JMRI Interface (DCC++ protocol):
    -   [x] Convert to StateFlow pattern.
    -   [x] Replaced WiFiServer code with socket_listener from OpenMRNLite.
-   [x] Locomotive Roster:
    -   [x] Support for automatic idling of locomotives when CS starts.
    -   [x] Expose function mappings in roster entries.
-   [x] OpenLCB (LCC):
    -   [x] Add metrics to the InfoScreen.
    -   [x] Basic CS CDI web interface.
    -   [x] CS Node ID reset from web interface.
    -   [x] Force factory reset when node id changes.
    -   [x] Integrate CV memory space.
    -   [x] Traction Protocol integration.
    -   [x] TrainSearch protocol support:
        -   [x] Basic integration is functional (loco request works for dynamic locos)
        -   [x] Persisted locos that are not active can not be requested from UWT-100.
    -   [x] Default configuration via "menuconfig" with override ability via web interface.
    -   [x] Rewrite HW Can driver.
    -   [x] Expose WiFi config in CDI
-   [x] S88 Sensors:
    -   [x] Add delay in s88SensorTask so that it gives time for other tasks to execute between updates.
    -   [x] Convert to use openlcb::Polling model rather than tasks.
    -   [x] Restore custom web rendering of sensor data.
-   [x] Status Display:
    -   [x] Migrated to StateFlow interface.
    -   [x] Status LED output for WiFi, OPS and PROG, EXT_1 and EXT_2. EXT_1 and EXT_2 unused currently.
-   [x] Web Interface:
    -   [x] Add busy/wait spinner for when data is loading (or being refreshed) in the web interface.
    -   [x] Add dialog for failed CS requests.
    -   [x] Add space in footer for clock so date/time are not smashed together.
    -   [x] Auto-connect WebSocket from initPage()
    -   [x] Auto-refresh of status pages
    -   [x] Fixed S88 section hiding.
    -   [x] Fixed S88 sensor bus creation/edit json payload parameters.
    -   [x] Hide power button for prog track when it is off.
    -   [x] Use websocket usage for sending throttle commands to the CS.
    -   [x] Replace ArduinoJson with "JSON for Modern C++"
    -   [x] Switched to new StateFlow based http server.
    -   [x] Remove overall power on/off as it no longer makes sense.
    -   [x] Implement support for application/x-www-form-urlencoded POST/PUT data.
    -   [x] Entering loco name as "loco #3" results in "loco 33" being registered.
    -   [x] Delete of roster entry doesn't always force refresh of section.
-   [x] WiFi Management:
    -   [x] Migrated to OpenMRN Esp32WiFiManager.
    -   [x] Integrate the WiFiConfiguration CDI element to CS CDI.
    -   [x] Add fallback option for Esp32WiFiManager to not reboot the node when SSID connect fails and SoftAP is active.
    -   [x] Update Esp32WiFiManager to use SimpleStackBase instead of SimpleCanStack.
    -   [x] Add ability to reconfigure Station SSID/PW via web interface.
    -   [x] Default configuration via "menuconfig" with override ability via web interface.
    -   [x] SoftAP support for initial config and "non-home" network.

## Future planning
These entries are items being tracked for a future release, these are not listed in priority order.

-   [] DCC Signal Generation:
    -   [ ] Concurrency guards for ProgrammingTrackBackend.
    -   [ ] Continue sending eStop packet until eStop is cleared.
    -   [ ] Reimplement DCC Prog Track interface so it supports multiple requests (serialized).
    -   [ ] Introduced priority queue mechanism for DCC packets.
    -   [ ] Expire inactive locos that are not auto-idle.
-   [ ] RailCom detector:
    -   [ ] Connect the RailComHub data into other parts of the stack.
-   [ ] GPIO:
    -   [ ] Expose Outputs, Sensors, S88 events on LCC.
    -   [ ] Add S88 sensor data to InfoScreen status line, 16 sensor output rotation.
    -   [ ] Convert Sensors to use openlcb::Polling model rather than tasks.
-   [ ] HTTP Server 
    -   [ ] Investigate websocket compatibility issue with iOS devices
-   [ ] Locomotive Roster:
    -   [ ] Expose db entries via R/W CDI?
-   [ ] Miscellaneous:
    -   [ ] Switch to shared_ptr instead of raw pointers.
    -   [ ] WiThrottle support (https://github.com/atanisoft/ESP32CommandStation/issues/15)
    -   [ ] Combine usages of openlcb::RefreshLoop.
-   [ ] Nextion Interface:
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
-   [ ] OpenLCB (LCC):
    -   [ ] Broadcast events for turnout state change.
    -   [ ] Discard turnout events when turnout already in expected state (drop duplicate events).
    -   [ ] Expose PROG track via CDI (needed for UWT-100 support).
    -   [ ] Reimplement Loco Consist leveraging LCC Traction Consist functionality.
    -   [ ] Connect Loco CDI to TrainDB.
-   [ ] Over-the-air (OTA) firmware update:
    -   [ ] Return to normal mode on Nextion when OTA fails.
-   [ ] Web Interface:
    -   [ ] Add strict validation of input parameter data.
    -   [ ] Expose Loco Consist creation.
    -   [ ] Expose Loco function map names.
    -   [ ] Add urldecode to table rendering code.

## PCB related tasks:
-   [ ] LMD18200 heatsink evaluation:
    https://www.mouser.com/ProductDetail/593202B03500G
-   [x] Ambient Thermal Monitoring:
    https://www.mouser.com/ProductDetail/MCP9701AT-E-TTVAO
-   [x] Consider swapping two LM393 for one LM339.