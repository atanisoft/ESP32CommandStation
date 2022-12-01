# Esp32 Command Station v2.0 TODO list

## Command Station TODO list

### Required for v2.0

* [x] Adjust tasks to pin to specific cores:
    - WiFi: PRO_CPU
    - LwIP: PRO_CPU
    - OpenMRN: PRO_CPU (inherit app_main)
    - Esp32WiFiManager: APP_CPU priority 3
    - StatusLED: APP_CPU priority 3
    - HttpServer: Listener runs standalone, HttpServer leverages Esp32WiFiManager Executor.
    - TWAI: ISR APP_CPU, task float priority is one less than LwIP.
* [x] AccessoryDecoderDB: Verify all functionality.
* [x] AccessoryDecoderDB: Directly consume OpenLCB events.
* [x] AccessoryDecoderDB: Add name to decoder.
* [x] Build: Add automatic config selector for PCB, "Uno" form-factor with L298, etc.
* [x] Build: Migrate to IDF components rather than submodules?
* [ ] DCC: ULP current sense / ACK.
    - [x] Implementation of ULP code to read ADC and wake main SoC when thresholds breached.
    - [x] Disable track when short occurs.
    - [x] Enable track when short has cleared.
    - [x] Send short/shutdown events for OPS.
    - [x] Shunt support
* [x] DCC: ProgrammingTrackBackend support (UWT-100 dependency).
* [x] Deps: Remove nlohmann_json dependency in favor of cJSON.
* [ ] Docs: Add user guide and how to build guide under docs tree.
* [x] FastClock: Re-add FastClock support.
* [ ] OpenLCB: Verify bootloader firmware update works as expected.
* [ ] RailCom: Verify timing of cut-out and adjust timing delay counts.
* [ ] RailCom: Verify incoming data stream with logic analyzer.
* [x] TempSensor: Move ADC read into ULP.
* [x] TrainDB: reduce function types to: light, horn (momentary), bell, mute, coupler, other, other (momentary).
* [x] TrainDB: Remove Marklin support.
* [x] TrainDB: Enable editing via loco CDI.
* [x] TrainDB: Expose via Memory Space?
* [x] TrainSearch: General code formatting / cleanup
* [x] WebUI: Cross check against WebServer.cpp for uniformity in parameters/json.
* [x] WebUI: Roster save via WS.
* [x] WebUI: Expose FastClock configuration (non-realtime).
* [ ] WebUI: Expose Programming Track
* [ ] WebUI: Test all endpoints.

### Nice to have for v2.0

* Build: Improve robustness of gzip search.
* RailCom: Migrate Esp32RailComDriver to use HAL APIs (portability work)
* RailCom: Login packet support.
* RailCom: Integrate RailCom hub with rest of stack (TrainSearch, Prog Backend, etc).
* TrainDB: Configurable timeout for idle trains.
* TrainDB: Customizable function label names.
* WebServer: Document all endpoints and data formats (including WS)
* WebUI: urldecode all field data from json.
* WebUI: Add function name/label editing via web interface.
* WebUI: Show remaining characters for various text fields.
* WebUI: Filtering of accessories/locos.

### Future planning

* AccessoryDecoderDB: Routes.
* Import roster/routes/accessories from JMRI
* WiThrottle support.
* Build: IDF v5+ migrate from driver/timer.h to driver/gptimer.h

## OpenMRNIDF TODO list

### Required for v2.0

* [x] Remove dedicated task from Esp32WiFiManager in favor of state flows.
* [x] Verify OSSelectWakeup and TWAI is not using VFS sem after invalidation.

### Nice to have for v2.0

* Further refinements in Esp32Gpio.

### Not required for v2.0

* DCC: DCC-14 speed step support
* Add option for inline translation in Esp32Ws2812 rather than on-demand translation.
* Add support for more LED types in Esp32Ws2812.

## HttpServer TODO list

* Expose HttpRequest::param(string name, string default)