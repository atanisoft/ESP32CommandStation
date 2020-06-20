# ESP32 Command Station Feature/Bug Tracking list
This document tracks features and bug fixes that are planned.

## v1.5.0
The primary focus of v1.5.0 is improvements to the DCC signal code, RailCom
support, expanded LCC integration and general stability improvements.

### TODO for v1.5.0
These are must have features/bug fixes for v1.5.0:

-   [ ] DCC: Rewrite/Test RailCom detector code.
-   [ ] DOC: Write migration guide for building, what has changed since v1.2.3, future of legacy protocols (DCC++, non-LCC JMRI)
-   [x] LCC: TrainSearch protocol.
    -   [x] Basic integration is functional (loco request works for dynamic locos)
    -   [x] Persisted locos that are not active can not be requested from UWT-100.
-   [x] LCC: Add fallback option for Esp32WiFiManager to not reboot the node when SSID connect fails and SoftAP is active.
-   [x] Misc: Scrub code for TODO comments or unimplemented methods.
-   [x] Roster: Implement delete loco.
-   [x] Roster: Implement auto-idle loco.
-   [x] Roster: Roster Entries should contain function id mappings.
-   [x] S88: Convert to use openlcb::Polling model rather than tasks.
-   [x] Turnouts: Switch fully to DCC address only support, for DCC++ compatibility decode the data and convert.
-   [x] Web: Entering loco name as "loco #3" results in "loco 33" being registered.
-   [x] Web: Delete of roster entry doesn't always force refresh of section.
-   [ ] Web: Test all endpoints to ensure proper functionality after httpd rework.

### Completed for v1.5.0
These tasks have been completed and integrated for v1.5.0:

-   [x] Build: Migrate to ESP-IDF v4.0 with cmake.
-   [x] Build: Add PCB build defaults for ESP-IDF.
-   [x] Build: Integrate "menuconfig" instead of requiring a lot of header file modifications.
-   [x] Config: Dynamic config for LCC and WiFi stored on SD/SPIFFS.
-   [x] Config: SoftAP support for initial config and "non-home" network.
-   [x] Config: Split up monolithic json files to instead be individual files.
-   [x] DCC: Allow adjustment of the DCC preamble bit count, default is 16 (OPS) and 22 (PROG).
-   [x] DCC: Switch to using RMT for signal generation.
-   [x] DCC: Remove hardware timer legacy code.
-   [x] DCC: Refactor signal generation to use: dcc::Packet, UpdateLoop, RailcomHub, etc.
-   [x] JMRI: Convert to StateFlow pattern.
-   [x] JMRI: Replaced WiFiServer code with socket_listener from OpenMRNLite.
-   [x] LCC: Add LCC metrics to the InfoScreen.
-   [x] LCC: CS CDI web interface.
-   [x] LCC: CS Node ID reset from web interface.
-   [x] LCC: Force factory reset when node id changes.
-   [x] LCC: Integrate CV memory space.
-   [x] LCC: Integrate the WiFiConfiguration CDI element.
-   [x] LCC: Migrate to Esp32WiFiManager instead of Arduino WiFi library.
-   [x] LCC: Traction Protocol integration.
-   [x] Misc: Remove usage of log_X macros in favor of LOG, some places use ets_printf().
-   [x] Nextion: Lock to title screen until WiFi connects.
-   [x] Nextion: Switch to timer based speed increment/decrement on the nextion mcu side.
-   [x] Status: Move InfoScreen to StateFlow interface.
-   [x] Status: Status LED output for WiFi, OPS and PROG, EXT_1 and EXT_2. EXT_1 and EXT_2 unused currently.
-   [x] S88: Add delay in s88SensorTask so that it gives time for other tasks to execute between updates.
-   [x] S88: Restore custom web rendering of sensor data.
-   [x] Web: Add busy/wait spinner for when data is loading (or being refreshed) in the web interface.
-   [x] Web: Add dialog for failed CS requests.
-   [x] Web: Add space in footer for clock so date/time are not smashed together.
-   [x] Web: Auto-connect WebSocket from initPage()
-   [x] Web: Auto-refresh of status pages
-   [x] Web: Configure Station SSID/PW
-   [x] Web: Fixed S88 section hiding.
-   [x] Web: Fixed S88 sensor bus creation/edit json payload parameters.
-   [x] Web: Hide power button for prog track when it is off.
-   [x] Web: Reintroduce websocket usage for pushing commands back to the CS.
-   [x] Web: Replace ArduinoJson with "JSON for Modern C++"
-   [x] Web: Replace webserver code with StateFlow based server.
-   [x] Web: Remove overall power on/off as it no longer makes sense.
-   [x] Web: Implement support for application/x-www-form-urlencoded POST/PUT data.

## Future planning
These entries are items being tracked for a future release, these are not listed in priority order.

-   [ ] DCC: Concurrency guards for ProgrammingTrackBackend.
-   [ ] DCC: Continue sending eStop packet until eStop is cleared.
-   [ ] DCC: Reimplement DCC Prog Track interface so it supports multiple requests (serialized).
-   [ ] DCC: Introduced priority queue mechanism for DCC packets.
-   [ ] DCC: Expire inactive locos that are not auto-idle.
-   [ ] GPIO: Expose Outputs, Sensors, S88 events on LCC.
-   [ ] LCC: Rewrite HW Can driver.
-   [ ] LCC: Broadcast events for turnout state change.
-   [ ] LCC: Discard turnout events when turnout already in expected state (drop duplicate events).
-   [ ] LCC: Expose PROG track via CDI (needed for UWT-100 support).
-   [ ] LCC: Reimplement Loco Consist leveraging LCC Traction Consist functionality.
-   [ ] LCC: Update Esp32WiFiManager to use SimpleStackBase instead of SimpleCanStack.
-   [ ] LCC: Connect Loco CDI to TrainDB.
-   [ ] Misc: Switch to shared_ptr instead of raw pointers.
-   [ ] Misc: WiThrottle support (https://github.com/atanisoft/ESP32CommandStation/issues/15)
-   [ ] Misc: Combine usages of openlcb::RefreshLoop.
-   [ ] Nextion: rewrite from scratch.
-   [ ] Nextion: Add support for Nextion Upload via OTA in NeoNextion.
-   [ ] Nextion: Add notification of turnout state change when changed external to the nextion code.
-   [ ] Nextion: Auto turn on of track power from Nextion when interacting with loco/turnouts.
-   [ ] Nextion: Adjust screen size detection for 5" and 7" displays which send the same screen type prefix.
-   [ ] Nextion: Implement automatic resolver for component id during page initialization so we can drop component IDs from the argument list.
-   [ ] Nextion: Move screen detection code into NeoNextion lib.
-   [ ] Nextion: Reimplement NeoNextion using OpenMRN StateFlow pattern with uart select() usage.
-   [ ] Nextion: Replace Routes page with a Setup page which will include route creation.
-   [ ] OTA: OTA support via JMRI / LCC using OpenMRN Bootloader.
-   [ ] OTA: Return to normal mode on Nextion when OTA fails.
-   [ ] Roster: Expose db entries via R/W CDI.
-   [ ] S88: Add S88 sensor data to InfoScreen status line, 16 sensor output rotation.
-   [ ] Sensors: Convert to use openlcb::Polling model rather than tasks.
-   [ ] Web: Add strict validation of input parameter data.
-   [ ] Web: Expose Loco Consist creation.
-   [ ] Web: Expose Loco function map names.
-   [ ] Web: Auto-refresh tables when delete/edit completes.
-   [ ] Web: Add urldecode to table rendering code.
-   [ ] Web: active locos capped at 28 speed steps instead of 128.
