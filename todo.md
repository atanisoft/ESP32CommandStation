# Esp32 Command Station v2.0 TODO list

## General TODO list

* Investigate heap corruption issues.
* DCC: select() VFS API?
* DCC: ULP current sense / ACK.
* DCC: consider on-demand translation and drop tx complete hook.
* DCC: ProgrammingTrackBackend support (UWT-100 dependency).
* Deps: Remove nlohmann_json dependency in favor of cJSON.
* OpenLCB: Verify bootloader firmware update works as expected.
* RailCom: Migrate Esp32RailComDriver to use Timer and UART HAL APIs.
* RailCom: Verify timing of cut-out and adjust delays.
* RailCom: Integrate RailCom hub with rest of stack (TrainSearch, Prog Backend, etc)
* RailCom: Verify incoming data stream with logic analyzer.
* StatusLED: Rework to depend on Esp32Ws2812 instead of NeoPixelBus.
* TrainDB: reduce function types to: light, horn, bell, mute, coupler, generic. horn with momentary flag.
* TrainDB: Remove Marklin support
* TrainDB: CBOR data format?
* TrainDB: Expose via Memory Space?
* TrainDB: Configurable timeout for idle trains
* TrainDB / TrainSearch: Ensure CDI edits are in sync with DB.
* TrainSearch: General code formatting / cleanup
* TrainSearch: Move e-stop handling to this module rather than main.
* TurnoutDB: CBOR data format?
* WebUI: Cross check against WebServer.cpp for uniformity in parameters/json.
* WebUI: Add function name/label editing via web interface.
* WebUI: Test all endpoints.
* WebUI: urldecode all field data from json.
* WebUI: compressed ws stream?
* WiThrottle support
* migrate to shared_ptr instead of raw pointers.

## OpenMRNIDF TODO list

* Remove dedicated task from Esp32WiFiManager in favor of state flows.
* Further refinement in Esp32Gpio.
* Add option for inline translation in Esp32Ws2812 rather than on-demand translation.
* Add support for more LED types in Esp32Ws2812.
* Verify OSSelectWakeup is not using VFS sem after invalidation.
* Verify TWAI is not using VFS sem after invalidation.
