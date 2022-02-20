/** \copyright
 * Copyright (c) 2022, Mike Dunston
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
 * \file WiThrottle.hxx
 *
 * Main include file for the WiThrottle Server declarations.
 *
 * @author Mike Dunston
 * @date 8 Feb 2022
 */

/// @file WiThrottle server definitions

#ifndef WITHROTTLE_HXX_
#define WITHROTTLE_HXX_

#include <dcc/TrackIf.hxx>
#include <executor/Dispatcher.hxx>
#include <executor/Service.hxx>
#include <executor/StateFlow.hxx>
#include <openlcb/Node.hxx>
#include <openlcb/TractionThrottle.hxx>
#include <os/MDNS.hxx>
#include <TrainDbDefs.hxx>
#include <utils/constants.hxx>
#include <utils/format_utils.hxx>
#include <utils/Singleton.hxx>
#include <utils/socket_listener.hxx>
#include <utils/StringPrintf.hxx>

#ifdef CONFIG_IDF_TARGET
namespace openmrn_arduino
{
    class Esp32WiFiManager;
}
#endif

namespace withrottle
{
    /// This is the number of milliseconds to use for the socket send and
    /// receive timeouts.
    DECLARE_CONST(withrottle_socket_timeout_ms);

    /// FreeRTOS task stack size for the WiThrottle Executor.
    DECLARE_CONST(withrottle_server_stack_size);

    /// FreeRTOS task priority for the WiThrottle Executor.
    DECLARE_CONST(withrottle_server_priority);

    /// Max number of bytes to read in a single chunk when reading the
    /// WiThrottle requests.
    DECLARE_CONST(withrottle_read_size);

    /// This is the maximum wait time for receiving a single chunk of data from
    /// connected client.
    DECLARE_CONST(withrottle_client_timeout_ms);

    /// Maximum number of locomotives to allow per connected client.
    DECLARE_CONST(withrottle_max_client_locomotives);

    /// Maximum number of connections to allow on the WiThrottle server.
    DECLARE_CONST(withrottle_max_connections);

    static constexpr const char * const REQUEST_EOL_CHARACTER_CRNL = "\x0D\x0A";
    static constexpr const char * const REQUEST_EOL_CHARACTER_CR = "\x0D";
    static constexpr const char * const REQUEST_EOL_CHARACTER_NL = "\x0A";

    static constexpr const char * const COLLECTION_DELIMITER = "]\\[";
    static constexpr const char * const FIELD_DELIMITER = "}|{";

    static constexpr const char * const COMMAND_PAYLOAD_DELIMITER = "<;>";

    /// Forward declaration.
    class WiThrottleClientFlow;

    /// WiThrottle Server implementation.
    class WiThrottleServer : public Service, public Singleton<WiThrottleServer>
    {
    public:
        /// Constructor.
        ///
        /// @param node @ref Node which this server runs on.
        /// @param mdns @ref MDNS instance to use for publishing mDNS records
        /// when the server is active.
        /// @param track @ref TrackIf to send DCC packets to.
        /// @param port port to listen for WiThrottle requests on, default is
        /// 12090.
        /// @param name name to use for the executor, default is "withrottle".
        /// @param service_name mDNS service name to advertise when the server
        /// is active, default is _withrottle._tcp.
        WiThrottleServer(openlcb::Node *node,
                         MDNS *mdns,
                         dcc::TrackIf *track,
                         uint16_t port = 12090,
                         const std::string &name = "withrottle",
                         const std::string service_name = "_withrottle._tcp");

#ifdef CONFIG_IDF_TARGET
        /// Constructor.
        ///
        /// @param node @ref Node which this server runs on.
        /// @param track @ref TrackIf to send DCC packets to.
        /// @param port port to listen for WiThrottle requests on, default is
        /// 12090.
        /// @param service_name mDNS service name to advertise when the server
        /// is active, default is _withrottle._tcp.
        WiThrottleServer(openlcb::Node *node,
                         dcc::TrackIf *track,
                         uint16_t port = 12090,
                         const std::string service_name = "_withrottle._tcp");
#endif // CONFIG_IDF_TARGET

        /// Destructor.
        ~WiThrottleServer();

        /// Creates a new @ref WiThrottleFlow for the provided socket handle.
        ///
        /// @param fd is the socket handle.
        void new_connection(int fd);

        /// Configures the heartbeat period to use for each connected throttle.
        void configure_heartbeat(uint8_t period);

    private:
        /// Node upon which this server runs on.
        openlcb::Node *node_;

        /// Name to use for the @ref WiThrottleServer.
        const std::string name_;

        /// @ref MDNS instance to use for publishing mDNS records when the
        /// @ref WiThrottleServer server is active;
        MDNS *mdns_{nullptr};

        /// @ref TrackIf to send DCC packets to (if needed).
        dcc::TrackIf *track_;

        /// mDNS service name to advertise when the @ref WiThrottleServer is
        /// running.
        const std::string mdns_service_;

        /// @ref Executor that manages all @ref StateFlow for the
        /// @ref WiThrottleServer, note this is not necessarily used on all
        /// platforms.
        Executor<1> executor_;

        /// TCP/IP port to listen for requests on.
        uint16_t port_;

        /// @ref SocketListener that will accept() the socket connections and
        /// call the @ref WiThrottleServer when a new client is available.
        uninitialized<SocketListener> listener_;

        /// timeval to use for newly connected sockets for SO_RCVTIMEO and
        /// SO_SNDTIMEO.
        struct timeval socket_timeout_;

        /// Internal state flag for the listener_ being active.
        bool active_{false};

        /// Internal flag to indicate if this class owns the executor or if it is
        /// externally managed.
        bool externalExecutor_{false};

        /// Number of seconds to require throttle(s) to ping the server before
        /// disconnect.
        uint8_t throttleHeartbeat_{10};

        /// Number of active connections.
        std::atomic_uint_least8_t connectionCount_{0};

        /// Starts the socket listener.
        void start_listener();

        /// Stops the socket listener (if active).
        void stop_listener();

        /// Allow access from the @ref WiThrottleClientFlow class.
        friend class WiThrottleClientFlow;

        DISALLOW_COPY_AND_ASSIGN(WiThrottleServer);
    };

    /// WiThrottle Request parser that implements the @ref StateFlowBase
    /// interface.
    class WiThrottleClientFlow : public StateFlowBase
    {
    public:
        /// Constructor.
        ///
        /// @param server @ref WiThrottleServer server that created this instance.
        /// @param fd socket handle for the connected client.
        /// @param remote_ip Remote IP address of the client.
        /// @param heartbeat Heartbeat period to use for this client.
        WiThrottleClientFlow(WiThrottleServer *server,
                             int fd,
                             uint32_t remote_ip,
                             uint8_t heartbeat);

        /// Destructor.
        ~WiThrottleClientFlow();

    private:
        /// @ref StateFlowTimedSelectHelper which assists in reading/writing of
        /// the request data stream.
        StateFlowTimedSelectHelper helper_{this};

        /// Timeout value to use for reading data while processing the request.
        const long long timeout_{MSEC_TO_NSEC(config_withrottle_client_timeout_ms())};

        /// Maximum read size for a single read() call when processing a
        /// request.
        const size_t readSize_{(size_t)config_withrottle_read_size()};

        /// Remote client IP (if known).
        const uint32_t remoteIP_;

        /// Underlying socket handle for this request.
        const int fd_;

        /// @ref WiThrottleServer instance that owns this request.
        WiThrottleServer *server_;

        /// Device Unique ID.
        std::string udid_;

        /// Device name.
        std::string name_;

        typedef struct
        {
            openlcb::TractionThrottle * throttle;
            uint8_t key;
            uint32_t address;
        } throttle_t;

        /// Collection of throttles that have been assigned to this client.
        std::vector<throttle_t> throttles_;

        /// Iterator used for releasing locomotives from all @ref throttles_
        /// this client creates.
        std::vector<throttle_t>::iterator
            throttleIter_{throttles_.end()};

        enum WiThrottleCommands : uint8_t
        {
            /// Primary throttle command.
            PRIMARY    = 'T',

            /// Secondary throttle command.
            SECONDARY  = 'S',

            /// Multi-throttle command.
            MULTI      = 'M',

            /// Hex encoded DCC packet command.
            HEX_PACKET = 'D',

            /// Heartbeat command.
            HEARTBEAT  = '*',

            /// Device name command.
            SET_NAME   = 'N',

            /// Device unique ID command.
            SET_ID     = 'H',

            /// Panel command.
            PANEL      = 'P',

            /// Roster command.
            ROSTER     = 'R',

            /// Quit command.
            QUIT       = 'Q',

            TYPE_MASK  = 0xFF,
        };

        class ThrottleCommand
        {
        public:
            /** type of the dispatcher criteria */
            typedef WiThrottleCommands id_type;

            WiThrottleCommands command; /**< type of command */
            string payload; /**< the command payload */

            /** @returns the unique identifier of the reply message */
            id_type id()
            {
                return command;
            };
        };
        
        /// Custom dispatcher logic for routing various commands to the
        /// appropriate handlers.
        typedef DispatchFlow<Buffer<ThrottleCommand>, 1> CommandDispatchFlow;

        /// Instance of the custom dispatcher for routing requests.
        CommandDispatchFlow commandDispatcher_;

        Buffer<ThrottleCommand> *nextCommand_;

        /// Number of seconds to require this throttle to ping before shutdown
        /// and disconnect.
        uint8_t heartbeat_{10};

        /// Temporary buffer used for reading the WiThrottle request.
        std::vector<uint8_t> buf_;

        /// Temporary accumulator for the WiThrottle data before it is parsed.
        std::string reqData_;

        /// Base handler for all WiThrottle commands.
        class WiThrottleCommandBase :
            public StateFlow<Buffer<ThrottleCommand>, QList<1>>
        {
        public:
            /// Constructor.
            ///
            /// @param throttle @ref WiThrottleClientFlow that calls this
            /// handler.
            /// @param type @ref WiThrottleCommands command to register for
            /// this handler.
            WiThrottleCommandBase(WiThrottleClientFlow *throttle,
                WiThrottleCommands type);

            /// Destructor.
            ~WiThrottleCommandBase();

        protected:
            WiThrottleClientFlow *throttleFlow_;

            /// @ref StateFlowTimedSelectHelper which assists in writing of
            /// response data.
            StateFlowTimedSelectHelper helper_{this};

            /// Buffer to use for sending response data to the client.
            std::string sendBuf_;

            void register_command_type(WiThrottleCommands type);
            void deregister_command_type(WiThrottleCommands type);
        };

        class WiThrottleCommandLocomotiveLegacy : public WiThrottleCommandBase
        {
        public:
            /// Constructor.
            ///
            /// @param throttle @ref WiThrottleClientFlow that calls this
            /// handler.
            ///
            /// This handler will process @ref WiThrottleCommands::PRIMARY,
            /// and @ref WiThrottleCommands::SECONDARY.
            WiThrottleCommandLocomotiveLegacy(WiThrottleClientFlow *throttle);

            ~WiThrottleCommandLocomotiveLegacy();

        private:
            openlcb::TractionThrottle *throttle_;

            uint32_t address_;
            dcc::TrainAddressType addressType_;
            uint8_t throttleKey_;

            /// Entry point for processing a throttle command.
            StateFlowBase::Action entry() override;

            STATE_FLOW_STATE(loco_assigned);
            STATE_FLOW_STATE(loco_released);
            STATE_FLOW_STATE(loco_state_loaded);
            STATE_FLOW_STATE(send_function_labels);
            STATE_FLOW_STATE(send_function_states);
            STATE_FLOW_STATE(done);
            STATE_FLOW_STATE(command_not_recognized);
        };

        class WiThrottleCommandLocomotive : public WiThrottleCommandBase
        {
        public:
            /// Constructor.
            ///
            /// @param throttle @ref WiThrottleClientFlow that calls this
            /// handler.
            ///
            /// This handler will process @ref WiThrottleCommands::PRIMARY,
            /// @ref WiThrottleCommands::SECONDARY and
            /// @ref WiThrottleCommands::MULTI.
            WiThrottleCommandLocomotive(WiThrottleClientFlow *throttle);

            ~WiThrottleCommandLocomotive();

        private:
            openlcb::TractionThrottle *throttle_;

            uint32_t address_{0};
            dcc::TrainAddressType addressType_{
                dcc::TrainAddressType::UNSPECIFIED};
            uint8_t addressTypeChar_{'L'};
            commandstation::DccMode driveMode_{
                commandstation::DccMode::DCC_DEFAULT};
            uint8_t throttleKey_{0};
            bool multiAddress_{false};
            bool listenForUpdates_{false};

            /// Entry point for processing a throttle command.
            StateFlowBase::Action entry() override;

            bool lookup_roster_entry(std::string &entry_name);

            STATE_FLOW_STATE(parse_loco_command);
            STATE_FLOW_STATE(parse_loco_action);
            STATE_FLOW_STATE(loco_assigned);
            STATE_FLOW_STATE(loco_released);
            STATE_FLOW_STATE(release_next_loco);
            STATE_FLOW_STATE(loco_state_loaded);
            STATE_FLOW_STATE(send_function_labels);
            STATE_FLOW_STATE(send_function_states);
            STATE_FLOW_STATE(send_loco_speed);
            STATE_FLOW_STATE(send_loco_direction);
            STATE_FLOW_STATE(send_loco_stepmode);
            STATE_FLOW_STATE(done);
            STATE_FLOW_STATE(command_not_recognized);
            STATE_FLOW_STATE(command_not_supported);
        };

        class WiThrottleCommandRoster : public WiThrottleCommandBase
        {
        public:
            WiThrottleCommandRoster(WiThrottleClientFlow *throttle);

            ~WiThrottleCommandRoster();

        private:
            StateFlowBase::Action entry() override;
        };

        class WiThrottleCommandPanel : public WiThrottleCommandBase
        {
        public:
            WiThrottleCommandPanel(WiThrottleClientFlow *throttle);

            ~WiThrottleCommandPanel();

        private:
            StateFlowBase::Action entry() override;

            STATE_FLOW_STATE(done);
            STATE_FLOW_STATE(command_not_recognized);
            STATE_FLOW_STATE(command_not_supported);
        };

        class WiThrottleCommandRawPacket : public WiThrottleCommandBase
        {
        public:
            WiThrottleCommandRawPacket(WiThrottleClientFlow *throttle,
                                       dcc::TrackIf *track);

            ~WiThrottleCommandRawPacket();

        private:
            StateFlowBase::Action entry() override;
            dcc::TrackIf *track_;
        };

        friend class WiThrottleCommandBase;
        friend class WiThrottleCommandLocomotive;
        friend class WiThrottleCommandRoster;
        friend class WiThrottleCommandPanel;
        friend class WiThrottleCommandRawPacket;

        WiThrottleCommandLocomotive locoCmd_;
        WiThrottleCommandRoster rosterCmd_;
        WiThrottleCommandPanel panelCmd_;
        WiThrottleCommandRawPacket packetCmd_;

        STATE_FLOW_STATE(parse);
        STATE_FLOW_STATE(shutdown);
        STATE_FLOW_STATE(shutdown_throttles);
        STATE_FLOW_STATE(read_more_data);
        STATE_FLOW_STATE(send_cs_info_packets);
        STATE_FLOW_STATE(send_roster_list);
        STATE_FLOW_STATE(send_consist_list);
        STATE_FLOW_STATE(send_accessory_list);
        STATE_FLOW_STATE(send_power_status);
        STATE_FLOW_STATE(send_heartbeat_config);

        void trigger_shutdown();
        openlcb::TractionThrottle *get_throttle(uint8_t key, uint32_t *address);
        void release_throttle(openlcb::TractionThrottle *throttle);
    };

} // namespace withrottle

#endif // WITHROTTLE_HXX_