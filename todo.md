# Esp32 Command Station v2.0 TODO list

## Command Station TODO list

### Required for v2.0

* [x] Adjust tasks to pin to specific cores:
    * WiFi - PRO_CPU
    * LwIP - APP_CPU
    * OpenMRN - PRO_CPU (inherit app_main)
    * Esp32WiFiManager - float priority 2
    * StatusLED - APP_CPU priority 3
    * HttpServer - APP_CPU priority 5
    * TWAI - ISR APP_CPU, task float priority is one less than LwIP.
    * RMT - ISR PRO_CPU
* [ ] Investigate heap corruption issues.
* [ ] DCC: ULP current sense / ACK.
* [x] DCC: ProgrammingTrackBackend support (UWT-100 dependency).
* [ ] Deps: Remove nlohmann_json dependency in favor of cJSON.
* [ ] OpenLCB: Verify bootloader firmware update works as expected.
* [ ] RailCom: Verify timing of cut-out and adjust timing delay counts.
* [ ] RailCom: Verify incoming data stream with logic analyzer.
* [ ] TrainDB: reduce function types to: light, horn, bell, mute, coupler, generic. horn with momentary flag.
* [ ] TrainDB: Remove Marklin support
* [ ] AccessoryDecoderDB: Verify all functionality
* [x] AccessoryDecoderDB: Directly consume OpenLCB events.
* [ ] WebUI: Cross check against WebServer.cpp for uniformity in parameters/json.
* [ ] WebUI: Add function name/label editing via web interface.
* [ ] WebUI: Test all endpoints.
* [ ] FastClock: Re-add FastClock support

### Nice to have for v2.0

* RailCom: Migrate Esp32RailComDriver to use Timer and UART HAL APIs.
* RailCom: Integrate RailCom hub with rest of stack (TrainSearch, Prog Backend, etc)
* TrainDB: Configurable timeout for idle trains
* TrainDB / TrainSearch: Enable editing via CDI.
* TrainSearch: General code formatting / cleanup
* TrainSearch: Move e-stop handling to this module rather than main.
* WiThrottle support
* WebUI: urldecode all field data from json.

### Not required for v2.0

* DCC: select() VFS API?
* DCC: on-demand translation and drop tx complete hook?
* StatusLED: Rework to depend on Esp32Ws2812 instead of NeoPixelBus.
* TrainDB: CBOR data format?
* TrainDB: Expose via Memory Space?
* AccessoryDecoderDB: CBOR data format?
* WebUI: compressed ws stream?
* migrate to shared_ptr instead of raw pointers.

## OpenMRNIDF TODO list

### Required for v2.0

[ ] Remove dedicated task from Esp32WiFiManager in favor of state flows.
[ ] Verify OSSelectWakeup and TWAI is not using VFS sem after invalidation.

### Nice to have for v2.0

* Further refinements in Esp32Gpio.

### Not required for v2.0

* Add option for inline translation in Esp32Ws2812 rather than on-demand translation.
* Add support for more LED types in Esp32Ws2812.
