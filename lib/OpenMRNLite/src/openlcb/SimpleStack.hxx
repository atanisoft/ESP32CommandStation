/** \copyright
 * Copyright (c) 2015, Balazs Racz
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
 * \file SimpleStack.hxx
 *
 * A complete OpenLCB stack for use in straightforward OpenLCB nodes.
 *
 * @author Balazs Racz
 * @date 10 Mar 2015
 */

#ifndef _OPENLCB_SIMPLESTACK_HXX_
#define _OPENLCB_SIMPLESTACK_HXX_

#include <fcntl.h>

#include "executor/Executor.hxx"
#include "openlcb/AliasAllocator.hxx"
#include "openlcb/ConfigRepresentation.hxx"
#include "openlcb/ConfigUpdateFlow.hxx"
#include "openlcb/DatagramCan.hxx"
#include "openlcb/DefaultNode.hxx"
#include "openlcb/EventService.hxx"
#include "openlcb/EventHandlerTemplates.hxx"
#include "openlcb/IfCan.hxx"
#include "openlcb/MemoryConfig.hxx"
#include "openlcb/NodeInitializeFlow.hxx"
#include "openlcb/ProtocolIdentification.hxx"
#include "openlcb/SimpleNodeInfo.hxx"
#include "openlcb/TractionTrain.hxx"
#include "openlcb/TrainInterface.hxx"
#include "nmranet_config.h"
#include "utils/GcTcpHub.hxx"
#include "utils/GridConnectHub.hxx"
#include "utils/HubDevice.hxx"
#include "utils/HubDeviceNonBlock.hxx"
#ifdef __FreeRTOS__
#include "utils/HubDeviceSelect.hxx"
#endif

namespace openmrn_arduino {
class OpenMRN;
}

namespace openlcb
{

/// This symbol contains the embedded text of the CDI xml file.
extern const char CDI_DATA[];

/// This symbol must be defined by the application to tell which file to open
/// for the configuration listener.
extern const char *const CONFIG_FILENAME;
/// This symbol must be defined by the application. It defines the size of the
/// config (or eeprom) file in bytes.
extern const size_t CONFIG_FILE_SIZE;

/// Helper class for bringing up all components needed for a typical OpenLCB
/// node.
///
/// Usage: create a global variable of type SimpleCanStack with the node's
/// NodeID as argument. For any additional components needed use the accessors
/// (such as executor(), service(), or memory_config_handler()) to instantiate
/// them. In the beginning of appl_main define how to access the bus, for
/// example by add_can_port_async() or add_gridconnect_port() or
/// connect_tcp_gridconnect_hub(). At the end of appl_main start the stack's
/// executor by calling either loop_executor() or start_executor_thread().
///
/// Example: applications/async_blink/main.cxx
class SimpleCanStackBase
{
public:
    static const unsigned EXECUTOR_PRIORITIES = 5;

    SimpleCanStackBase(const openlcb::NodeID node_id);

    /// @returns the executor that's controlling the main thread of the OpenLCB
    /// stack.
    Executor<EXECUTOR_PRIORITIES> *executor()
    {
        return &executor_;
    }

    /// @returns a plain service bound to the main thread's executor.
    Service *service()
    {
        return &service_;
    }

    /// @returns the openlcb Interface object.
    IfCan *iface()
    {
        return &ifCan_;
    }

    /// @returns the datagram service for registering new datagram handlers or
    /// acquiring datagram client objects.
    DatagramService *dg_service()
    {
        return &datagramService_;
    }

    /// Accessor for clients that have their custom SNIP-like handler.
    SimpleInfoFlow *info_flow()
    {
        return &infoFlow_;
    }

    /// @returns the CanHubFlow to which this stack is talking to. This hub
    /// flow usually has two members: the interface object from the software
    /// stack and the hardware connection via which to connect to the physical
    /// bus (which may be a device driver or a gridconnect protocol converter).
    CanHubFlow *can_hub()
    {
        return &canHub0_;
    }

    /// @returns the virtual node pointer of the main virtual node of the stack
    /// (as defined by the NodeID argument of the constructor).
    virtual Node *node() = 0;

    /// @return the handler for the memory configuration protocol. This is
    /// needed for registering additional memory spaces.
    MemoryConfigHandler *memory_config_handler()
    {
        return &memoryConfigHandler_;
    }

    /// Adds a CAN bus port with synchronous driver API.
    void add_can_port_blocking(const char *device)
    {
        int can_fd = ::open(device, O_RDWR);
        HASSERT(can_fd >= 0);
        auto *port = new FdHubPort<CanHubFlow>(
            &canHub0_, can_fd, EmptyNotifiable::DefaultInstance());
        additionalComponents_.emplace_back(port);
    }

#ifdef __FreeRTOS__
    /// Adds a CAN bus port with asynchronous driver API.
    ///
    /// @deprecated: most current FreeRTOS drivers use the the select-based
    /// asynchronous API, so they need add_can_port_select().
    void add_can_port_async(const char *device)
    {
        auto *port = new HubDeviceNonBlock<CanHubFlow>(&canHub0_, device);
        additionalComponents_.emplace_back(port);
    }

    /// Adds a CAN bus port with select-based asynchronous driver API.
    void add_can_port_select(const char *device)
    {
        auto *port = new HubDeviceSelect<CanHubFlow>(&canHub0_, device);
        additionalComponents_.emplace_back(port);
    }

    /// Adds a CAN bus port with select-based asynchronous driver API.
    /// @param fd file descriptor to add to can hub
    /// @param on_error Notifiable to wakeup on error
    void add_can_port_select(int fd, Notifiable *on_error = nullptr)
    {
        auto *port = new HubDeviceSelect<CanHubFlow>(&canHub0_, fd, on_error);
        additionalComponents_.emplace_back(port);
    }
#endif

    /// Adds a gridconnect port to the CAN bus.
    void add_gridconnect_port(const char *path, Notifiable *on_exit = nullptr);

#if defined(__linux__) || defined(__MACH__)
    /// Adds a gridconnect port to the CAN bus with setting the TTY options to
    /// raw. Suitablefor linux /dev/ttyACMxx devices. The most important option
    /// this call sets is to not echo characters coming in from the device back
    /// to the device. Echoing data back causes alias allocation problems and
    /// nodes on the bus repeatedly dropping their allocated aliases.
    void add_gridconnect_tty(const char *device, Notifiable *on_exit = nullptr);
#endif
#if defined(__linux__)
    /// Adds a CAN bus port with select-based asynchronous driver API.
    /// @params device CAN device name, for example: "can0" or "can1"
    /// @params loopback 1 to enable loopback localy to other open references,
    ///                  0 to enable loopback localy to other open references,
    ///                  in most cases, this paramter won't matter
    void add_socketcan_port_select(const char *device, int loopback = 1);
#endif

    /// Starts a TCP server on the specified port in listening mode. Each
    /// incoming connection will be assumed to be in gridconnect protocol and
    /// will be added to the gridconnect hub.
    void start_tcp_hub_server(int port = 12021)
    {
        /// @TODO (balazs.racz) make this more efficient by rendering to string
        /// only once for all connections.
        /// @TODO (balazs.racz) do not leak this.
        new GcTcpHub(&canHub0_, port);
    }

    /// Connects to a CAN hub using TCP with the gridconnect protocol.
    void connect_tcp_gridconnect_hub(const char *host, int port)
    {
        int fd = ConnectSocket(host, port);
        HASSERT(fd >= 0);
        create_gc_port_for_can_hub(&canHub0_, fd);
    }

    /// Causes all CAN packets to be printed to stdout.
    void print_all_packets(bool timestamped = false)
    {
        auto *port = new DisplayPort(&service_, timestamped);
        gridconnect_hub()->register_port(port);
        additionalComponents_.emplace_back(port);
    }

    /// Returns the hub to be used for gridconnect-format CANbus. You can
    /// inject text CAN packets to this hub, add printers and in general connect
    /// devices and sockets using the gridconnect protocol to talk CANbus.
    ///
    /// The actual gridconnect parser / renderer objects will be created upon
    /// the first call to this function.
    HubFlow *gridconnect_hub()
    {
        if (!gcHub_)
        {
            gcHub_.reset(new HubFlow(&service_));
            gcAdapter_.reset(GCAdapterBase::CreateGridConnectAdapter(
                gcHub_.get(), &canHub0_, false));
        }
        return gcHub_.get();
    }

    ConfigUpdateService *config_service()
    {
        return &configUpdateFlow_;
    }

    /// Reinitializes the node. Useful to call after the connection has flapped
    /// (gone down and up).
    void restart_stack();

    friend class ::openmrn_arduino::OpenMRN;

    /// Donates the current thread to the executor. Never returns.
    /// @param delay_start if true, then prevents sending traffic to the bus
    void loop_executor(bool delay_start = false)
    {
        start_stack(delay_start);
        executor_.thread_body();
    }

    /// Call this function when you used delay_start upon starting the
    /// executor.
    void start_after_delay();

    /// Instructs the executor to create a new thread and run in there.
    /// @param name is the thread name for the executor thread
    /// @param priority is the executor thread priority (used only for freertos)
    /// @param stack_size is the executor stack in bytes (used only for
    /// freertos)
    /// @param delay_start if true, then prevents sending traffic to the bus
    void start_executor_thread(
        const char *name, int priority, size_t stack_size, bool delay_start = false)
    {
        start_stack(delay_start);
        executor_.start_thread(name, priority, stack_size);
    }

    /// Tries to open the config file; if not existant, the size too small, or
    /// the version number is mismatched, then creates a new file of the given
    /// size with all 0xFF bytes inside. This will internally do everything
    /// done by the check_version_and_factory_reset call.
    ///
    /// @param ofs tells where in the file the versioninfo structure lies.
    ///
    /// @param expected_verison is the correct version of the config file.
    ///
    /// @param file_size is the minimum required size of the config file.
    ///
    /// @return file descriptor for config file.
    int create_config_file_if_needed(const InternalConfigData &ofs,
        uint16_t expected_version, unsigned file_size);

    /// Checks the version information in the EEPROM and performs a factory
    /// reset if incorrect or if force is set.
    /// @return file descriptor for config file.
    int check_version_and_factory_reset(const InternalConfigData &ofs,
        uint16_t expected_version, bool force = false);

    /// Overwrites all events in the eeprom with a brand new event ID.
    void factory_reset_all_events(const InternalConfigData &ofs, int fd);

    /// Call this function at the beginning of appl_main, just before {\link
    /// check_version_and_factory_reset} or {\link
    /// create_config_file_if_needed} if the list of event offsets are
    /// dyamically created instead of statically linked.
    /// @param offsets is a vector with the data. The last entry must be
    /// zero. This vector must outlive the SimpleStack object.
    void set_event_offsets(const vector<uint16_t> *offsets);

    /// Helper function to send an event report to the bus. Performs
    /// synchronous (dynamic) memory allocation so use it sparingly and when
    /// there is sufficient amount of RAM available.
    /// @param event_id is the event to send off.
    void send_event(uint64_t event_id)
    {
        auto *b = node()->iface()->global_message_write_flow()->alloc();
        b->data()->reset(Defs::MTI_EVENT_REPORT, node()->node_id(),
            eventid_to_buffer(event_id));
        node()->iface()->global_message_write_flow()->send(b);
    }

    /// Sends an addressed message to the bus. Performs
    /// synchronous (dynamic) memory allocation so use it sparingly and when
    /// there is sufficient amount of RAM available.
    /// @param mti is the message to send
    /// @param dst is the node to send message to.
    /// @param payload is the contents of the message
    void send_message_to(
        Defs::MTI mti, NodeHandle dst, const string &payload = EMPTY_PAYLOAD)
    {
        auto *b = node()->iface()->addressed_message_write_flow()->alloc();
        b->data()->reset(mti, node()->node_id(), dst, payload);
        node()->iface()->addressed_message_write_flow()->send(b);
    }

protected:
    /// Call this function once after the actual IO ports are set up. Calling
    /// before the executor starts looping is okay.
    void start_stack(bool delay_start);

    /// Hook for clients to initialize the node-specific components.
    virtual void start_node() = 0;

    /// Exports the memory config spaces that are typically used for a complex
    /// node. Expected to be called from start_node().
    void default_start_node();

    /// This executor's threads will be handled
    Executor<EXECUTOR_PRIORITIES> executor_{NO_THREAD()};
    /// Default service on the particular executor.
    Service service_{&executor_};
    /// Abstract CAN bus in-memory.
    CanHubFlow canHub0_{&service_};
    /// NMRAnet interface for sending and receiving messages, formatting them
    /// to the CAN bus port and maintaining the conversion flows, caches etc.
    IfCan ifCan_{&executor_, &canHub0_, config_local_alias_cache_size(),
        config_remote_alias_cache_size(), config_local_nodes_count()};
    /// Calls the config listeners with the configuration FD.
    ConfigUpdateFlow configUpdateFlow_{&ifCan_};
    /// The initialization flow takes care for node startup duties.
    InitializeFlow initFlow_{&service_};
    /// Dispatches event protocol requests to the event handlers.
    EventService eventService_{&ifCan_};
    /// General flow for simple info requests.
    SimpleInfoFlow infoFlow_{&ifCan_};

    CanDatagramService datagramService_{&ifCan_,
        config_num_datagram_registry_entries(), config_num_datagram_clients()};
    MemoryConfigHandler memoryConfigHandler_{
        &datagramService_, nullptr, config_num_memory_spaces()};

    /// All packets are forwarded to this hub in gridconnect format, if
    /// needed. Will be initialized upon first use.
    std::unique_ptr<HubFlow> gcHub_;
    /// Bridge between canHub_ and gcHub_. Lazily initialized.
    std::unique_ptr<GCAdapterBase> gcAdapter_;

    /// Stores and keeps ownership of optional components.
    std::vector<std::unique_ptr<Destructable>> additionalComponents_;
};

/// CAN-based stack with DefaultNode.
class SimpleCanStack : public SimpleCanStackBase
{
public:
    SimpleCanStack(const openlcb::NodeID node_id);

    /// @returns the virtual node pointer of the main virtual node of the stack
    /// (as defined by the NodeID argument of the constructor).
    Node *node() override
    {
        return &node_;
    }

private:
    static const auto PIP_RESPONSE = Defs::EVENT_EXCHANGE | Defs::DATAGRAM |
        Defs::MEMORY_CONFIGURATION | Defs::ABBREVIATED_DEFAULT_CDI |
        Defs::SIMPLE_NODE_INFORMATION | Defs::CDI;

    void start_node() override { default_start_node(); }

    /// The actual node.
    DefaultNode node_;
    /// Handles PIP requests.
    ProtocolIdentificationHandler pipHandler_{&node_, PIP_RESPONSE};
    /// Handles SNIP requests.
    SNIPHandler snipHandler_{&ifCan_, &node_, &infoFlow_};
};

/// CAN-based stack with TrainNode.
class SimpleTrainCanStack : public SimpleCanStackBase
{
public:
    /// Creates a train node OpenLCB stack.
    ///
    /// @param train the implementation of the train
    /// @param fdi_xml XML file to export as the FDI for train functions
    SimpleTrainCanStack(openlcb::TrainImpl *train, const char *fdi_xml, NodeID node_id);

    /// @returns the virtual node pointer of the main virtual node of the stack
    /// (as defined by the NodeID argument of the constructor).
    Node *node() override
    {
        return &trainNode_;
    }

private:
    static const auto PIP_RESPONSE = openlcb::Defs::SIMPLE_PROTOCOL_SUBSET |
        openlcb::Defs::DATAGRAM | openlcb::Defs::MEMORY_CONFIGURATION |
        openlcb::Defs::EVENT_EXCHANGE | openlcb::Defs::SIMPLE_NODE_INFORMATION |
        openlcb::Defs::TRACTION_CONTROL | openlcb::Defs::TRACTION_FDI |
        Defs::ABBREVIATED_DEFAULT_CDI | Defs::CDI;

    void start_node() override;

    TrainService tractionService_{&ifCan_};
    /// The actual node.
    TrainNodeWithId trainNode_;
    FixedEventProducer<openlcb::TractionDefs::IS_TRAIN_EVENT>
        isTrainEventHandler{&trainNode_};
    ReadOnlyMemoryBlock fdiBlock_;
    /// Handles PIP requests.
    ProtocolIdentificationHandler pipHandler_{&trainNode_, PIP_RESPONSE};
    /// Handles SNIP requests.
    SNIPHandler snipHandler_{&ifCan_, &trainNode_, &infoFlow_};
};

} // namespace openlcb

#endif //  _OPENLCB_SIMPLESTACK_HXX_
