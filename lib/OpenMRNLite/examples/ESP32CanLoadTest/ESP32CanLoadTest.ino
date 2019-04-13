/** \copyright
 * Copyright (c) 2019, Mike Dunston
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \file ESP32IOBoard.ino
 *
 * Main file for the io board application on an ESP32.
 *
 * @author Mike Dunston
 * @date 13 January 2019
 */

#include <Arduino.h>
#include <ESPmDNS.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <vector>
#include <esp_spi_flash.h>

#include <OpenMRNLite.h>
#include <openlcb/TcpDefs.hxx>

#include <openlcb/MultiConfiguredConsumer.hxx>
#include <utils/GpioInitializer.hxx>
#include <freertos_drivers/arduino/CpuLoad.hxx>

// Pick an operating mode below, if you select USE_WIFI it will expose
// this node on WIFI if you select USE_CAN, this node will be available
// on CAN.
// Enabling both options will allow the ESP32 to be accessible from
// both WiFi and CAN interfaces.

#define USE_WIFI
#define USE_CAN

// Uncomment the line below to have this node advertise itself via mDNS as a
// hub. When this is enabled, other devices can find and connect to this node
// via mDNS, treating it as a hub. Note this requires USE_WIFI to be enabled
// above and should only be enabled on one node which is acting as a hub.
// #define BROADCAST_MDNS

// uncomment the line below to have all packets printed to the Serial
// output. This is not recommended for production deployment.
//#define PRINT_PACKETS

#include "config.h"

/// This is the node id to assign to this device, this must be unique
/// on the CAN bus.
static constexpr uint64_t NODE_ID = UINT64_C(0x050101011823);

#if defined(USE_WIFI)
// Configuring WiFi accesspoint name and password
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// There are two options:
// 1) edit the sketch to set this information just below. Use quotes:
//     const char* ssid     = "linksys";
//     const char* password = "superSecret";
// 2) add a new file to the sketch folder called something.cpp with the
// following contents:
//     #include <OpenMRNLite.h>
//
//     char WIFI_SSID[] = "linksys";
//     char WIFI_PASS[] = "theTRUEsupers3cr3t";

/// This is the name of the WiFi network (access point) to connect to.
const char *ssid = WIFI_SSID;

/// Password of the wifi network.
const char *password = WIFI_PASS;

/// This is the hostname which the ESP32 will advertise via mDNS, it should be
/// unique.
const char *hostname = "esp32mrn";

OVERRIDE_CONST(gridconnect_buffer_size, 3512);
//OVERRIDE_CONST(gridconnect_buffer_delay_usec, 200000);
OVERRIDE_CONST(gridconnect_buffer_delay_usec, 2000);
OVERRIDE_CONST(gc_generate_newlines, CONSTANT_TRUE);
OVERRIDE_CONST(executor_select_prescaler, 60);
OVERRIDE_CONST(gridconnect_bridge_max_outgoing_packets, 2);


#endif // USE_WIFI

#if defined(USE_CAN)
/// This is the ESP32 pin connected to the SN65HVD23x/MCP2551 R (RX) pin.
/// Recommended pins: 4, 16, 21.
/// Note: Any pin can be used for this other than 6-11 which are connected to
/// the onboard flash.
/// Note: If you are using a pin other than 4 you will likely need to adjust
/// the GPIO pin definitions for the outputs.
constexpr gpio_num_t CAN_RX_PIN = GPIO_NUM_4;

/// This is the ESP32 pin connected to the SN65HVD23x/MCP2551 D (TX) pin.
/// Recommended pins: 5, 17, 22.
/// Note: Any pin can be used for this other than 6-11 which are connected to
/// the onboard flash and 34-39 which are input only.
/// Note: If you are using a pin other than 5 you will likely need to adjust
/// the GPIO pin definitions for the outputs.
constexpr gpio_num_t CAN_TX_PIN = GPIO_NUM_5;

#endif // USE_CAN

/// This is the primary entrypoint for the OpenMRN/LCC stack.
OpenMRN openmrn(NODE_ID);

// note the dummy string below is required due to a bug in the GCC compiler
// for the ESP32
string dummystring("abcdef");

/// ConfigDef comes from config.h and is specific to this particular device and
/// target. It defines the layout of the configuration memory space and is also
/// used to generate the cdi.xml file. Here we instantiate the configuration
/// layout. The argument of offset zero is ignored and will be removed later.
static constexpr openlcb::ConfigDef cfg(0);

#if defined(USE_WIFI)
Esp32WiFiManager wifi_mgr(ssid, password, openmrn.stack(), cfg.seg().wifi());
#endif // USE_WIFI

// Declare output pins
// NOTE: pins 6-11 are connected to the onboard flash and can not be used for
// any purpose and pins 34-39 are INPUT only.
GPIO_PIN(IO0, GpioOutputSafeLow, 15);
GPIO_PIN(IO1, GpioOutputSafeLow, 2);
GPIO_PIN(IO2, GpioOutputSafeLow, 16);
GPIO_PIN(IO3, GpioOutputSafeLow, 17);
GPIO_PIN(IO4, GpioOutputSafeLow, 13);
GPIO_PIN(IO5, GpioOutputSafeLow, 12);
GPIO_PIN(IO6, GpioOutputSafeLow, 14);
GPIO_PIN(IO7, GpioOutputSafeLow, 27);

// List of GPIO objects that will be used for the output pins. You should keep
// the constexpr declaration, because it will produce a compile error in case
// the list of pointers cannot be compiled into a compiler constant and thus
// would be placed into RAM instead of ROM.
constexpr const Gpio *const outputGpioSet[] = {
    IO0_Pin::instance(), IO1_Pin::instance(), //
    IO2_Pin::instance(), IO3_Pin::instance(), //
    IO4_Pin::instance(), IO5_Pin::instance(), //
    IO6_Pin::instance(), IO7_Pin::instance()  //
};

openlcb::MultiConfiguredConsumer gpio_consumers(openmrn.stack()->node(), outputGpioSet,
    ARRAYSIZE(outputGpioSet), cfg.seg().consumers());

// Declare input pins, these are using analog pins as digital inputs
// NOTE: pins 25 and 26 can not safely be used as analog pins while
// WiFi is active. All analog pins can safely be used for digital
// inputs regardless of WiFi being active or not.
GPIO_PIN(IO8, GpioInputPU, 32);
GPIO_PIN(IO9, GpioInputPU, 33);
GPIO_PIN(IO10, GpioInputPU, 34);
GPIO_PIN(IO11, GpioInputPU, 35);
GPIO_PIN(IO12, GpioInputPU, 36);
GPIO_PIN(IO13, GpioInputPU, 39);
GPIO_PIN(IO14, GpioInputPU, 25);
GPIO_PIN(IO15, GpioInputPU, 26);

openlcb::ConfiguredProducer IO8_producer(
    openmrn.stack()->node(), cfg.seg().producers().entry<0>(), IO8_Pin());
openlcb::ConfiguredProducer IO9_producer(
    openmrn.stack()->node(), cfg.seg().producers().entry<1>(), IO9_Pin());
openlcb::ConfiguredProducer IO10_producer(
    openmrn.stack()->node(), cfg.seg().producers().entry<2>(), IO10_Pin());
openlcb::ConfiguredProducer IO11_producer(
    openmrn.stack()->node(), cfg.seg().producers().entry<3>(), IO11_Pin());
openlcb::ConfiguredProducer IO12_producer(
    openmrn.stack()->node(), cfg.seg().producers().entry<4>(), IO12_Pin());
openlcb::ConfiguredProducer IO13_producer(
    openmrn.stack()->node(), cfg.seg().producers().entry<5>(), IO13_Pin());
openlcb::ConfiguredProducer IO14_producer(
    openmrn.stack()->node(), cfg.seg().producers().entry<6>(), IO14_Pin());
openlcb::ConfiguredProducer IO15_producer(
    openmrn.stack()->node(), cfg.seg().producers().entry<7>(), IO15_Pin());


// Create an initializer that can initialize all the GPIO pins in one shot
typedef GpioInitializer<
    IO0_Pin,  IO1_Pin,  IO2_Pin,  IO3_Pin,  // outputs 0-3
    IO4_Pin,  IO5_Pin,  IO6_Pin,  IO7_Pin,  // outputs 4-7
    IO8_Pin,  IO9_Pin,  IO10_Pin, IO11_Pin, // inputs 0-3
    IO12_Pin, IO13_Pin, IO14_Pin, IO15_Pin // inputs 4-7
    > GpioInit;

// The producers need to be polled repeatedly for changes and to execute the
// debouncing algorithm. This class instantiates a refreshloop and adds the
// producers to it.
openlcb::RefreshLoop producer_refresh_loop(openmrn.stack()->node(),
    {
        IO8_producer.polling(),
        IO9_producer.polling(),
        IO10_producer.polling(),
        IO11_producer.polling(),
        IO12_producer.polling(),
        IO13_producer.polling(),
        IO14_producer.polling(),
        IO15_producer.polling()
    }
);

class FactoryResetHelper : public DefaultConfigUpdateListener {
public:
    UpdateAction apply_configuration(int fd, bool initial_load,
                                     BarrierNotifiable *done) OVERRIDE {
        AutoNotify n(done);
        return UPDATED;
    }

    void factory_reset(int fd) override
    {
        cfg.userinfo().name().write(fd, openlcb::SNIP_STATIC_DATA.model_name);
        cfg.userinfo().description().write(
            fd, "OpenLCB + Arduino-ESP32 on an ESP32.");
        for(int i = 0; i < openlcb::NUM_OUTPUTS; i++)
        {
            cfg.seg().consumers().entry(i).description().write(fd, "");
        }
        for(int i = 0; i < openlcb::NUM_INPUTS; i++)
        {
            cfg.seg().producers().entry(i).description().write(fd, "");
            CDI_FACTORY_RESET(cfg.seg().producers().entry(i).debounce);
        }
    }
} factory_reset_helper;

namespace openlcb
{
    // Name of CDI.xml to generate dynamically.
    const char CDI_FILENAME[] = "/spiffs/cdi.xml";

    // This will stop openlcb from exporting the CDI memory space upon start.
    extern const char CDI_DATA[] = "";

    // Path to where OpenMRN should persist general configuration data.
    extern const char *const CONFIG_FILENAME = "/spiffs/openlcb_config";

    // The size of the memory space to export over the above device.
    extern const size_t CONFIG_FILE_SIZE = cfg.seg().size() + cfg.seg().offset();

    // Default to store the dynamic SNIP data is stored in the same persistant
    // data file as general configuration data.
    extern const char *const SNIP_DYNAMIC_FILENAME = CONFIG_FILENAME;
}

CpuLoad cpu_load;
hw_timer_t * timer = nullptr;
CpuLoadLog* cpu_log = nullptr;

void IRAM_ATTR onTimer()
{
    if (spi_flash_cache_enabled())
    {
        // Retrieves the vtable pointer from the currently running executable.
        unsigned *pp = (unsigned *)openmrn.stack()->executor()->current();
        cpuload_tick(pp ? pp[0] | 1 : 0);
    }
}

void setup()
{
#ifdef USE_WIFI
    //wifi_mgr.enable_verbose_logging();
#endif    
    Serial.begin(115200L);

    timer = timerBegin(3, 80, true); // timer_id = 3; divider=80; countUp = true;
    timerAttachInterrupt(timer, &onTimer, true); // edge = true
    // 1MHz clock, 163 ticks per second desired.
    timerAlarmWrite(timer, 1000000/163, true);
    timerAlarmEnable(timer);

    // Initialize the SPIFFS filesystem as our persistence layer
    if (!SPIFFS.begin())
    {
        printf("SPIFFS failed to mount, attempting to format and remount\n");
        if (!SPIFFS.begin(true))
        {
            printf("SPIFFS mount failed even with format, giving up!\n");
            while (1)
            {
                // Unable to start SPIFFS successfully, give up and wait
                // for WDT to kick in
            }
        }
    }

    // Create the CDI.xml dynamically
    openmrn.create_config_descriptor_xml(cfg, openlcb::CDI_FILENAME);

    // Create the default internal configuration file
    openmrn.stack()->create_config_file_if_needed(cfg.seg().internal_config(),
        openlcb::CANONICAL_VERSION, openlcb::CONFIG_FILE_SIZE);

    // initialize all declared GPIO pins
    GpioInit::hw_init();

    // Start the OpenMRN stack
    openmrn.begin();
    openmrn.start_executor_thread();
    cpu_log = new CpuLoadLog(openmrn.stack()->service());


#if defined(PRINT_PACKETS)
    // Dump all packets as they are sent/received.
    // Note: This should not be enabled in deployed nodes as it will
    // have performance impact.
    openmrn.stack()->print_all_packets();
#endif // PRINT_PACKETS

#if defined(USE_CAN)
    // Add the hardware CAN device as a bridge
    openmrn.add_can_port(
        new Esp32HardwareCan("esp32can", CAN_RX_PIN, CAN_TX_PIN));
#endif // USE_CAN
}

void loop()
{
    // Call the OpenMRN executor, this needs to be done as often
    // as possible from the loop() method.
    openmrn.loop();
}
