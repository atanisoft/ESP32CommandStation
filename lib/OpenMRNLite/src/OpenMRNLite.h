/** \copyright
 * Copyright (c) 2018, Balazs Racz
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
 * \file OpenMRNLite.h
 *
 * Main include file for the OpenMRN library to be used in an Arduino
 * compilation environment.
 *
 * @author Balazs Racz
 * @date 24 July 2018
 */

#ifndef _ARDUINO_OPENMRNLITE_H_
#define _ARDUINO_OPENMRNLITE_H_

#include <Arduino.h>

#include "freertos_drivers/arduino/ArduinoGpio.hxx"
#include "freertos_drivers/arduino/Can.hxx"
#include "freertos_drivers/arduino/WifiDefs.hxx"
#include "openlcb/SimpleStack.hxx"
#include "utils/GridConnectHub.hxx"
#include "utils/Uninitialized.hxx"
#include "utils/FileUtils.hxx"

#if defined(ESP32)

#include <esp_task.h>
#include <esp_task_wdt.h>

namespace openmrn_arduino {

/// Default stack size to use for all OpenMRN tasks on the ESP32 platform.
constexpr uint32_t OPENMRN_STACK_SIZE = 4096L;

/// Default thread priority for any OpenMRN owned tasks on the ESP32
/// platform. ESP32 hardware CAN RX and TX tasks run at lower priority
/// (-1 and -2 respectively) of this default priority to ensure timely
/// consumption of CAN frames from the hardware driver.
constexpr UBaseType_t OPENMRN_TASK_PRIORITY = ESP_TASK_TCPIP_PRIO;

} // namespace openmrn_arduino

#include "freertos_drivers/esp32/Esp32HardwareCanAdapter.hxx"
#include "freertos_drivers/esp32/Esp32HardwareSerialAdapter.hxx"
#include "freertos_drivers/esp32/Esp32WiFiManager.hxx"

// On the ESP32 we have persistent file system access so enable
// dynamic CDI.xml generation support
#define HAVE_FILESYSTEM

#endif // ESP32

namespace openmrn_arduino {

/// Bridge class that connects an Arduino API style serial port (sending CAN
/// frames via gridconnect format) to the OpenMRN core stack. This can be
/// generally used for USB ports or TCP sockets.
template <class SerialType> class SerialBridge : public Executable
{
public:
    /// Constructor.
    ///
    /// @param port is the Arduino serial implementation (usually &Serial).
    /// @param can_hub is the core CAN frame router of the OpenMRN stack,
    /// usually comes from stack()->can_hub().
    SerialBridge(SerialType *port, CanHubFlow *can_hub)
        : service_(can_hub->service())
        , port_(port)
    {
        GCAdapterBase::CreateGridConnectAdapter(&txtHub_, can_hub, false);
        txtHub_.register_port(&writePort_);
    }

    /// @todo (balazs.racz): add destructor and tear down the link without
    /// memory leaks.

    /// Called by the loop.
    void run() override
    {
        loop_for_write();
        loop_for_read();
    }

private:
    /// Handles data going out of OpenMRN and towards the serial port.
    void loop_for_write()
    {
        if (!writeBuffer_)
        {
            return;
        }
        size_t len = port_->availableForWrite();
        if (!len)
        {
            return;
        }
        size_t to_write = writeBuffer_->data()->size() - writeOfs_;
        if (len > to_write)
            len = to_write;
        port_->write(writeBuffer_->data()->data() + writeOfs_, len);
        writeOfs_ += len;
        if (writeOfs_ >= writeBuffer_->data()->size())
        {
            writeBuffer_ = nullptr;
            // wakes up state flow to release buffer and take next from the
            // queue.
            writePort_.notify();
        }
    }

    /// Handles data coming in from the serial port and sends it to OpenMRN.
    void loop_for_read()
    {
        int av = port_->available();
        if (av <= 0)
        {
            return;
        }
        auto *b = txtHub_.alloc();
        b->data()->skipMember_ = &writePort_;
        b->data()->resize(av);
        port_->read(b->data()->data(), b->data()->size());
        txtHub_.send(b);
    }

    friend class WritePort;

    /// Finite state machine running in the OpenMRN cooperative executor,
    /// registered to receive data that needs to be sent to the port. It will
    /// keep the pointer to the output buffer while the loop processes the data
    /// in it.
    class WritePort : public HubPort
    {
    public:
        /// Constructor.
        ///
        /// @param parent is the bridge object that owns *this.
        /// @param service holds the cooperative executor coming from the
        /// OpenMRN stack. Any service from OpenMRN will do.
        WritePort(SerialBridge *parent, Service *service)
            : HubPort(service)
            , parent_(parent)
        {
        }

        /// State machine state when a new buffer of data to be sent shows
        /// up. Must not block.
        Action entry() override
        {
            parent_->writeBuffer_ = message();
            parent_->writeOfs_ = 0;
            // Pauses the state machine until an external event calls notify().
            return wait_and_call(STATE(write_done));
        }

        /// We get to this state when the output buffer's data has fully been
        /// sent to the hardware and the notofy() call was made. It releases
        /// the memory and continues processing any further data piled up in
        /// the input queue.
        Action write_done()
        {
            return release_and_exit();
        }

    private:
        /// Parent that owns *this.
        SerialBridge *parent_;
    };

    /// Access to the stack's executor.
    Service *service_;
    /// Instance of the state machine.
    WritePort writePort_{this, service_};
    /// Arduino device instance.
    SerialType *port_;
    /// Buffer we are writing the output from right now. These bytes go from
    /// OpenMRN to the device.
    Buffer<HubData> *writeBuffer_{nullptr};
    /// Offset in the output buffer of the next byte to write.
    size_t writeOfs_;
    /// Hub for the textual data.
    HubFlow txtHub_{service_};
};

/// Bridge class that connects a native CAN controller to the OpenMRN core
/// stack, sending and receiving CAN frames directly. The CAN controller must
/// have a driver matching the Can controller base class defined in
/// OpenMRN/arduino.
class CanBridge : public Executable
{
public:
    /// Constructor.
    ///
    /// @param port is the CAN hardware driver implementation.
    /// @param can_hub is the core CAN frame router of the OpenMRN stack,
    /// usually comes from stack()->can_hub().
    CanBridge(Can *port, CanHubFlow *can_hub)
        : port_(port)
        , canHub_(can_hub)
    {
        port_->enable();
        can_hub->register_port(&writePort_);
    }

    ~CanBridge()
    {
        port_->disable();
    }

    /// Called by the loop.
    void run() override
    {
        loop_for_write();
        loop_for_read();
    }

private:
    /// Handles data going out of OpenMRN and towards the CAN port.
    void loop_for_write()
    {
        if (!writeBuffer_)
        {
            return;
        }
        if (port_->availableForWrite() <= 0)
        {
            return;
        }
        port_->write(writeBuffer_->data());
        writeBuffer_ = nullptr;
        writePort_.notify();
    }

    /// Handles data coming from the CAN port.
    void loop_for_read()
    {
        while (port_->available())
        {
            auto *b = canHub_->alloc();
            port_->read(b->data());
            b->data()->skipMember_ = &writePort_;
            canHub_->send(b);
        }
    }

    friend class WritePort;
    class WritePort : public CanHubPort
    {
    public:
        WritePort(CanBridge *parent)
            : CanHubPort(parent->canHub_->service())
            , parent_(parent)
        {
        }

        Action entry() override
        {
            parent_->writeBuffer_ = message();
            return wait_and_call(STATE(write_done));
        }

        Action write_done()
        {
            return release_and_exit();
        }

    private:
        CanBridge *parent_;
    };

    /// Hardware driver.
    Can *port_;
    /// Next buffer we are trying to write into the driver's FIFO.
    Buffer<CanHubData> *writeBuffer_{nullptr};
    /// Connection to the stack.
    CanHubFlow *canHub_;
    /// State flow with queues for output frames generated by the stack.
    WritePort writePort_{this};
};

/// Main class to declare the OpenMRN stack. Create one instance of this in the
/// root file of your sketch. Prefer to supply the Node ID during construction.
class OpenMRN : private Executable
{
public:
    /// Constructor if the Node ID is not known. Must call init(...) with the
    /// Node ID beofre using anything related to the stack, including
    /// instantiating objects that depend on the stack. Prefer the other
    /// constructor.
    OpenMRN()
    {
    }

    /// Use this constructor if stack() needs to be accessed during the time of
    /// the static construction.
    OpenMRN(openlcb::NodeID node_id);

    /// Call this function once if the empty constructor was used.
    void init(openlcb::NodeID node_id)
    {
        stack_.emplace(node_id);
    }

    /// @return pointer to the OpenMRN stack. Do not call before init().
    openlcb::SimpleCanStack *stack()
    {
        return stack_.operator->();
    }

    /// Call this function from the setup() function of the Arduino sketch
    /// after all other Arduino subsystems or libraries have been initialized.
    void begin()
    {
        stack_->start_stack(false);
    }

    /// Call this function from the loop() function of the Arduino sketch.
    void loop()
    {
        for (auto *e : loopMembers_)
        {
#if defined(ESP32)
            // Feed the watchdog so it doesn't reset the ESP32
            esp_task_wdt_reset();
#endif // ESP32
            e->run();
        }
    }

#ifndef OPENMRN_FEATURE_SINGLE_THREADED
    static void thread_entry(void *arg)
    {
        OpenMRN *p = (OpenMRN *)arg;
        p->stack()->executor()->thread_body();
    }

    void start_executor_thread()
    {
        haveExecutorThread_ = true;
#ifdef ESP32
        xTaskCreatePinnedToCore(&thread_entry, "OpenMRN", OPENMRN_STACK_SIZE,
            this, OPENMRN_TASK_PRIORITY, nullptr, 0);
        // Remove IDLE0 task watchdog, because the openmrn task sometimes uses
        // 100% cpu and it is pinned to CPU 0.
        disableCore0WDT();
#else
        stack_->executor()->start_thread(
            "OpenMRN", OPENMRN_TASK_PRIORITY, OPENMRN_STACK_SIZE);
#endif
    }
#endif

    /// Adds a serial port to the stack speaking the gridconnect protocol, for
    /// example to do a USB connection to a computer. This is the protocol that
    /// USB-CAN adapters for LCC are speaking to the computer.
    ///
    /// Example:
    /// void setup() {
    ///   ...
    ///   openmrn.begin();
    ///   openmrn.add_gridconnect_port(&Serial);
    ///   ...
    /// }
    ///
    /// @param port is the serial port instance from Arduino.
    template <class SerialType> void add_gridconnect_port(SerialType *port)
    {
        loopMembers_.push_back(
            new SerialBridge<SerialType>(port, stack()->can_hub()));
    }

    /// Adds a hardware CAN port to the stack. If multiple ports are added,
    /// OpenMRN will be forwarding traffic frames between them: the simplest
    /// CAN-USB sketch just adds the serial port connecting to the computer and
    /// the hardware CAN port.
    void add_can_port(Can *port)
    {
        loopMembers_.push_back(new CanBridge(port, stack()->can_hub()));
    }

#if defined(HAVE_FILESYSTEM)
    /// Creates the XML representation of the configuration structure and saves
    /// it to a file on the filesystem. Must be called after SPIFFS.begin() but
    /// before calling the {\link create_config_file_if_needed} method. The
    /// config file will be re-written whenever there was a change in the
    /// contents. It is also necessary to declare the static compiled-in CDI to
    /// be empty:
    /// ```
    ///    namespace openlcb {
    ///    // This will stop openlcb from exporting the CDI memory space
    ///    // upon start.
    ///    extern const char CDI_DATA[] = "";
    ///    }  // namespace openlcb
    /// ```
    /// @param cfg is the global configuration instance (usually called cfg).
    /// @param filename is where the xml file can be stored on the
    /// filesystem. For example "/spiffs/cdi.xml".
    template <class ConfigDef>
    void create_config_descriptor_xml(
        const ConfigDef &config, const char *filename)
    {
        string cdi_string;
        ConfigDef cfg(config.offset());
        cfg.config_renderer().render_cdi(&cdi_string);

        bool need_write = false;
        FILE *ff = fopen(filename, "rb");
        if (!ff)
        {
            need_write = true;
        }
        else
        {
            fclose(ff);
            string current_str = read_file_to_string(filename);
            if (current_str != cdi_string)
            {
                need_write = true;
            }
        }
        if (need_write)
        {
            LOG(INFO, "Updating CDI file %s (len %u)", filename,
                cdi_string.size());
            write_string_to_file(filename, cdi_string);
        }

        // Creates list of event IDs for factory reset.
        auto *v = new vector<uint16_t>();
        cfg.handle_events([v](unsigned o) { v->push_back(o); });
        v->push_back(0);
        stack()->set_event_offsets(v);
        // We leak v because it has to stay alive for the entire lifetime of
        // the stack.

        // Exports the file memory space.
        openlcb::MemorySpace *space = new openlcb::ROFileMemorySpace(filename);
        stack()->memory_config_handler()->registry()->insert(
            stack()->node(), openlcb::MemoryConfigDefs::SPACE_CDI, space);
    }
#endif // HAVE_FILESYSTEM

private:
    /// Callback from the loop() method. Internally called.
    void run() override
    {
        if (!haveExecutorThread_)
        {
            stack_->executor()->loop_some();
        }
    }

    /// Storage space for the OpenLCB stack. Will be constructed in init().
    uninitialized<openlcb::SimpleCanStack> stack_;

    /// List of objects we need to call in each loop iteration.
    vector<Executable *> loopMembers_{{this}};

    /// True if there is a separate thread running the executor.
    bool haveExecutorThread_{false};
};

} // namespace openmrn_arduino

using openmrn_arduino::OpenMRN;

#endif // _ARDUINO_OPENMRNLITE_H_
