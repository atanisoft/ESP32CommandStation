/** \copyright
 * Copyright (c) 2013, Balazs Racz
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
 * \file GridConnectHub.cxx
 * Gridconnect parser/renderer pipe components.
 *
 * @author Balazs Racz
 * @date 20 May 2013
 */

//#define LOGLEVEL VERBOSE

#include "utils/GridConnectHub.hxx"

#include "executor/StateFlow.hxx"
#include "can_frame.h"
#include "nmranet_config.h"
#include "utils/Buffer.hxx"
#include "utils/BufferPort.hxx"
#include "utils/HubDevice.hxx"
#ifdef OPENMRN_FEATURE_EXECUTOR_SELECT
#include "utils/HubDeviceSelect.hxx"
#endif
#include "utils/Hub.hxx"
#include "utils/GcStreamParser.hxx"
#include "utils/gc_format.h"

/// Actual implementation for the gridconnect bridge between a string-typed Hub
/// and a CAN-frame-typed Hub.
class GCAdapter : public GCAdapterBase
{
public:
    /// Constructor.
    ///
    /// @param gc_side A hub of type string, the gridconnect side.
    /// @param can_side A hub of type struct can_frame, the binary side.
    /// @param double_bytes if true, upon rendering data each byte will be
    /// doubled. This is an anciant workaround.
    GCAdapter(HubFlow *gc_side, CanHubFlow *can_side, bool double_bytes)
        : parser_(can_side->service(), can_side, &formatter_)
        , formatter_(can_side->service(), gc_side, &parser_, double_bytes)
    {
        gc_side->register_port(&parser_);
        can_side->register_port(&formatter_);
        isRegistered_ = 1;
    }

    /// Constructor
    ///
    /// @param gc_side_read A hub of type string to read packets from. The read
    /// packets will be converted to binary and sent to can_side.
    /// @param gc_side_write A hub of type string to write incoming binary
    /// packets that were converted to
    /// @param can_side  A hub of type struct can_frame, the binary side.
    /// @param double_bytes  if true, upon rendering data each byte will be
    /// doubled. This is an anciant workaround.
    GCAdapter(HubFlow *gc_side_read, HubFlow *gc_side_write,
        CanHubFlow *can_side, bool double_bytes)
        : parser_(can_side->service(), can_side, &formatter_)
        , formatter_(can_side->service(), gc_side_write, &parser_, double_bytes)
    {
        gc_side_read->register_port(&parser_);
        can_side->register_port(&formatter_);
        isRegistered_ = 1;
    }

    virtual ~GCAdapter()
    {
        unregister();
    }

    void unregister()
    {
        if (isRegistered_)
        {
            parser_.destination()->unregister_port(&formatter_);
            /// @todo(balazs.racz) This is incorrect if the 3-pipe constructor
            /// is used.
            formatter_.destination()->unregister_port(&parser_);
            isRegistered_ = 0;
        }
    }

    bool shutdown() OVERRIDE
    {
        unregister();
        return formatter_.shutdown() && parser_.is_waiting() && formatter_.is_waiting();
    }

    /// HubPort (on a CAN-typed hub) that turns a binary CAN packet into a
    /// string-formatted CAN packet, and sends it off to the HubFlow (of type
    /// string).
    class BinaryToGCMember : public CanHubPort
    {
    public:
        /// Constructor.
        ///
        /// @param service which executor to run on
        /// @param destination string hub where to write gridconnecct data to.
        /// @param skip_member what to set the skipmember_ field of the outgoing
        /// packets to.
        /// @param double_bytes if true, upon rendering data each byte will be
        /// doubled. This is an anciant workaround.
        BinaryToGCMember(Service *service, HubFlow *destination,
            HubPort *skip_member, int double_bytes)
            : CanHubPort(service)
            , delayPort_(service, destination, config_gridconnect_buffer_size(),
                  USEC_TO_NSEC(config_gridconnect_buffer_delay_usec()))
            , destination_(destination)
            , skipMember_(skip_member)
            , double_bytes_(double_bytes)
        {
        }

        /// @return where to write the packets to.
        HubFlow *destination()
        {
            return destination_;
        }

        bool shutdown() {
            return delayPort_.shutdown();
        }
        
        Action entry() override
        {
            LOG(VERBOSE, "can packet arrived: %" PRIx32,
                GET_CAN_FRAME_ID_EFF(*message()->data()));
            char *end =
                gc_format_generate(message()->data(), dbuf_, double_bytes_);
            size_t size = (end - dbuf_);
            if (size)
            {
                Buffer<HubData> *target_buffer = nullptr;
                /// @todo(balazs.racz) switch to asynchronous allocation here.
                mainBufferPool->alloc(&target_buffer);
                target_buffer->data()->skipMember_ = skipMember_;
                /// @todo(balazs.racz) try to use an assign function for better
                /// performance.
                target_buffer->data()->resize(size);
                memcpy((char *)target_buffer->data()->data(), dbuf_, size);
                target_buffer->set_done(bn_.reset(this));
                delayPort_.send(target_buffer, 0);
                release();
                return wait_and_call(STATE(buffer_accepted));
            }
            else
            {
                LOG(INFO, "gc generate failed.");
            }
            return release_and_exit();
        }

        Action buffer_accepted()
        {
            return exit();
        }

    private:
        /// Helper class that assembles larger outgoing packets from the
        /// individual packets by delaying data a little bit.
        BufferPort delayPort_;
        /// Destination buffer (characters).
        char dbuf_[56];
        /// Pipe to send data to.
        HubFlow *destination_;
        /// The pipe member that should be sent as "source".
        HubPort *skipMember_;
        /// Non-zero if doubling was requested.
        int double_bytes_;
        /// Helper object
        BarrierNotifiable bn_;
    };

    /// HubPort (on a string hub) that turns a gridconnect-formatted CAN packet
    /// into a binary CAN packet, and sends them off to the HubFlow (of CAN
    /// frame).
    class GCToBinaryMember : public HubPort
    {
    public:
        /// Constructor.
        ///
        /// @param service defines the executor to run on.
        /// @param destination Where to write converted binary packets.
        /// @param skip_member what to set skipMember_ of the outgoing packets
        /// to.
        GCToBinaryMember(
            Service *service, CanHubFlow *destination, CanHubPort *skip_member)
            : HubPort(service)
            , destination_(destination)
            , skipMember_(skip_member)
        {
            int max_frames_to_parse =
                config_gridconnect_bridge_max_incoming_packets();
            if (max_frames_to_parse > 1) {
                frameAllocator_.reset(new FixedPool(
                    sizeof(CanHubFlow::buffer_type), max_frames_to_parse));
            }
        }

        /// @return the destination to write data to.
        CanHubFlow *destination()
        {
            return destination_;
        }

        /** Takes more characters from the pending incoming buffer. @return next state */
        Action entry() override
        {
            inBuf_ = message()->data()->data();
            inBufSize_ = message()->data()->size();
            return call_immediately(STATE(parse_more_data));
        }

        /// Matches the incoming characters to the pattern to form incoming
        /// frames. @return next state.
        Action parse_more_data()
        {
            while (inBufSize_--)
            {
                char c = *inBuf_++;
                if (streamSegmenter_.consume_byte(c))
                {
                    // End of frame. Allocate an output buffer and parse the
                    // frame.
                    return allocate_and_call(destination_, STATE(parse_to_output_frame), frameAllocator_.get());
                }
            }
            // Will notify the caller.
            return release_and_exit();
        }

        /** Takes the completed frame in cbuf_, parses it into the allocation
         * result (a can pipe buffer) and sends off frame. Then comes back to
         * process buffer. @return next state. */
        Action parse_to_output_frame()
        {
            auto* b = get_allocation_result(destination_);
            if (streamSegmenter_.parse_frame_to_output(b->data()))
            {
                b->data()->skipMember_ = skipMember_;
                destination_->send(b);
            }
            else
            {
                // Releases the buffer.
                b->unref();
            }
            return call_immediately(STATE(parse_more_data));
        }

    private:
        /// Holds the state of the incoming characters and the boundary.
        GcStreamParser streamSegmenter_;
        
        /// The incoming characters.
        const char *inBuf_;
        /// The remaining number of characters in inBuf_.
        size_t inBufSize_;

        // Allocator to get the frame from. If NULL, the target's default
        // buffer pool will be used.
        std::unique_ptr<FixedPool> frameAllocator_;
        
        // ==== static data ====

        /// Pipe to send data to.
        CanHubFlow *destination_;
        /// The pipe member that should be sent as "source".
        CanHubPortInterface *skipMember_;
    };

private:
    /// PipeMember doing the parsing.
    GCToBinaryMember parser_;
    /// PipeMember doing the formatting.
    BinaryToGCMember formatter_;
    /// 1 if the flows are registered.
    unsigned isRegistered_ : 1;
};

GCAdapterBase *GCAdapterBase::CreateGridConnectAdapter(HubFlow *gc_side,
                                                       CanHubFlow *can_side,
                                                       bool double_bytes)
{
    return new GCAdapter(gc_side, can_side, double_bytes);
}

GCAdapterBase *GCAdapterBase::CreateGridConnectAdapter(HubFlow *gc_side_read,
                                                       HubFlow *gc_side_write,
                                                       CanHubFlow *can_side,
                                                       bool double_bytes)
{
    return new GCAdapter(gc_side_read, gc_side_write, can_side, double_bytes);
}

/// Implementation for the gridconnect bridge. Owns all necessary structures,
/// and is responsible for the initialization, registering, unregistering and
/// destruction of these structures.
struct GcPacketPrinter::Impl : public CanHubPortInterface
{
    /// Constructor.
    ///
    /// @param can_hub Which hub's packets to write to stdout.
    /// @param timestamped Whether to put timestamps on the packets written.
    Impl(CanHubFlow *can_hub, bool timestamped)
        : canHub_(can_hub)
        , timestamped_(timestamped)
    {
        canHub_->register_port(this);
    }

    ~Impl()
    {
        canHub_->unregister_port(this);
    }

    /// Overridden entry method to send binary data to this hub.
    ///
    /// @param message CAN frame buffer.
    /// @param priority priority
    void send(Buffer<CanHubData> *message, unsigned priority) OVERRIDE
    {
        AutoReleaseBuffer<CanHubData> b(message);
        char str[40];
        char* p = gc_format_generate(message->data(), str, false);
        *p = 0;
        if (timestamped_)
        {
#if defined(__linux__) || defined(__MACH__)
            struct timeval tv;
            gettimeofday(&tv, nullptr);
            struct tm t;
            localtime_r(&tv.tv_sec, &t);
            printf("%04d-%02d-%02d %02d:%02d:%02d:%06ld [%p] ",
                t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min,
                t.tm_sec, (long)tv.tv_usec, message->data()->skipMember_);
#endif
        }
        printf("%s", str);
        if (config_gc_generate_newlines() != 1)
        {
            printf("\n");
        }
    }

    /// Which hun are we registered to.
    CanHubFlow* canHub_;
    /// Whether we are printing timestamps of the packets.
    bool timestamped_;
};

GcPacketPrinter::GcPacketPrinter(CanHubFlow *can_hub, bool timestamped) : impl_(new Impl(can_hub, timestamped))
{
}

GcPacketPrinter::~GcPacketPrinter()
{
}

/// Implementation class that adds a device to a CAN hub with dynamic
/// translation of the packets to/from GridConnect format.
///
/// Sends a notification to the application level when there is an error on the
/// device and the connection is closed.
struct GcHubPort : public Executable
{
    /// Constructor.
    ///
    /// @param can_hub Parent (binary) hub flow.
    /// @param fd device descriptor of open channel (device or socket)
    /// @param on_exit Notifiable that will be called when the descriptor
    /// experiences an error (typically upon device closed or connection lost).
    /// @param use_select true if fd can be used with select, false if threads
    /// are needed.
    GcHubPort(CanHubFlow *can_hub, int fd, Notifiable *on_exit, bool use_select)
        : gcHub_(can_hub->service())
        , bridge_(
              GCAdapterBase::CreateGridConnectAdapter(&gcHub_, can_hub, false))
        , onExit_(on_exit)
    {
        LOG(VERBOSE, "gchub port %p", (Executable *)this);
        if (use_select) {
#ifndef OPENMRN_FEATURE_EXECUTOR_SELECT
            DIE("select is not supported");
#else
            gcWrite_.reset(new HubDeviceSelect<HubFlow>(&gcHub_, fd, this));
#endif
        } else {
            gcWrite_.reset(new FdHubPort<HubFlow>(&gcHub_, fd, this));
        }
    }
    virtual ~GcHubPort()
    {
    }

    /** This hub sees the character-based representation of the packets. The
     * members of it are: the bridge and the physical device (fd).
     *
     * Destruction requirement: HubFlow should be empty. This means after the
     * disconnection of the bridge (write side) and the FdHubport (read side)
     * we need to wait for the executor until this flow drains. */
    HubFlow gcHub_;
    /** Translates packets between the can-hub of the device and the char-hub
     * of this port.
     *
     * Destruction requirement: Call shutdown() on the can-side executor (and
     * yield) until it returns true. */
    std::unique_ptr<GCAdapterBase> bridge_;
    /** Reads the characters from the char-hub and sends them to the
     * fd. Similarly, listens to the fd and sends the read charcters to the
     * char-hub. */
    std::unique_ptr<FdHubPortInterface> gcWrite_;
    /** If not null, this notifiable will be called when the device is
     * closed. */
    Notifiable* onExit_;

    /** Callback in case the connection is closed due to error. */
    void notify() OVERRIDE
    {
        /* We would like to delete *this but we cannot do that in this
         * callback, because we don't know what executor we are running
         * on. Deleting on the write executor would cause a deadlock for
         * example. */
        gcHub_.service()->executor()->add(this);
    }

    void run() OVERRIDE
    {
        if (!bridge_->shutdown() || !gcHub_.is_waiting())
        {
            // Yield.
            gcHub_.service()->executor()->add(this);
            return;
        }
        LOG(INFO, "GCHubPort: Shutting down gridconnect port %d. (%p)",
            gcWrite_->fd(), bridge_.get());
        if (onExit_) {
            onExit_->notify();
            onExit_ = nullptr;
        }
        /* We get this call when something is wrong with the FDs and we need to
         * close the connection. It is guaranteed that by the time we got this
         * call the device is unregistered from the char bridge, and the
         * service thread is ready to be stopped. */
        delete this;
    }
};

void create_gc_port_for_can_hub(
    CanHubFlow *can_hub, int fd, Notifiable *on_exit, bool use_select)
{
    new GcHubPort(can_hub, fd, on_exit, use_select);
}
