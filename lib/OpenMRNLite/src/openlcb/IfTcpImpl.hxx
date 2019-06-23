/** \copyright
 * Copyright (c) 2017, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * \file IfTcpImpl.hxx
 *
 * Internal implementation flows for the Tcp interface
 *
 * @author Balazs Racz
 * @date 16 Apr 2017
 */

#ifndef _OPENLCB_IFTCPIMPL_HXX_
#define _OPENLCB_IFTCPIMPL_HXX_

#include "openlcb/If.hxx"
#include "utils/Hub.hxx"
#include "utils/HubDeviceSelect.hxx"

namespace openlcb
{

/// Static class for constants and utilities related to the TCP transport
/// protocol.
class TcpDefs
{
public:
    /// Renders a TCP message into a single buffer, ready to transmit.
    /// @param msg is the OpenLCB message to render.
    /// @param gateway_node_id will be populated into the message header as the
    /// message source (last sending node ID).
    /// @param sequence is a 48-bit millisecond value that's monotonic.
    /// @param target is the buffer into which to render the message.
    static void render_tcp_message(const GenMessage &msg,
        NodeID gateway_node_id, long long sequence, string *tgt)
    {
        bool has_dst = Defs::get_mti_address(msg.mti);
        HASSERT(tgt);
        string &target = *tgt;
        target.assign(HDR_LEN + msg.payload.size() +
                (has_dst ? MSG_ADR_PAYLOAD_OFS : MSG_GLOBAL_PAYLOAD_OFS),
            '\0');
        uint16_t flags = FLAGS_OPENLCB_MSG;
        error_to_data(flags, &target[HDR_FLAG_OFS]);
        unsigned sz = target.size() - HDR_SIZE_END;
        target[HDR_SIZE_OFS] = (sz >> 16) & 0xff;
        target[HDR_SIZE_OFS + 1] = (sz >> 8) & 0xff;
        target[HDR_SIZE_OFS + 2] = sz & 0xff;
        node_id_to_data(gateway_node_id, &target[HDR_GATEWAY_OFS]);
        node_id_to_data(sequence, &target[HDR_TIMESTAMP_OFS]);
        error_to_data(msg.mti, &target[HDR_LEN + MSG_MTI_OFS]);
        node_id_to_data(msg.src.id, &target[HDR_LEN + MSG_SRC_OFS]);
        if (has_dst)
        {
            node_id_to_data(msg.dst.id, &target[HDR_LEN + MSG_DST_OFS]);
            memcpy(&target[HDR_LEN + MSG_ADR_PAYLOAD_OFS], msg.payload.data(),
                msg.payload.size());
        }
        else
        {
            memcpy(&target[HDR_LEN + MSG_GLOBAL_PAYLOAD_OFS],
                msg.payload.data(), msg.payload.size());
        }
    }

    /// Guesses the length of a tcp message from looking at the prefix of the
    /// payload.
    ///
    /// @param data is the prefix of the incoming stream.
    /// @param len is the number of available bytes in this prefix.
    /// @return -1 if there is not enough bytes in the prefix yet to know the
    /// length. Otherwise the total number of bytes that constitute this
    /// packet.
    static int get_tcp_message_len(const void *data, size_t len)
    {
        if (len < HDR_SIZE_END)
        {
            return -1;
        }
        const uint8_t *src = static_cast<const uint8_t *>(data);
        uint32_t sz = src[HDR_SIZE_OFS];
        sz <<= 8;
        sz |= src[HDR_SIZE_OFS + 1];
        sz <<= 8;
        sz |= src[HDR_SIZE_OFS + 2];
        return sz + HDR_SIZE_END;
    }

    /// Parses a TCP message format (from binary payload) into a general
    /// OpenLCB message.
    /// @param src the rendered TCP message.
    /// @param tgt the output generic message
    /// @return false if the message is not well formatted or
    /// it is not an OpenLCB message.
    static bool parse_tcp_message(const string &src, GenMessage *tgt)
    {
        int expected_size = get_tcp_message_len(src.data(), src.size());
        if (expected_size > (int)src.size() || expected_size < MIN_MESSAGE_SIZE)
        {
            LOG(WARNING, "Incomplete or incorrectly formatted TCP message.");
            return false;
        }
        uint16_t flags = data_to_error(&src[0]);
        if ((flags & FLAGS_OPENLCB_MSG) == 0)
        {
            return false;
        }
        HASSERT((flags & FLAGS_CHAINING) == 0);
        tgt->flagsDst = 0;
        tgt->flagsSrc = 0;
        if (flags & FLAGS_FRAGMENT_NOT_FIRST)
        {
            tgt->set_flag_dst(GenMessage::DSTFLAG_NOT_FIRST_MESSAGE);
        }
        if (flags & FLAGS_FRAGMENT_NOT_LAST)
        {
            tgt->set_flag_dst(GenMessage::DSTFLAG_NOT_LAST_MESSAGE);
        }
        const char *msg = &src[HDR_LEN];
        // We have already checked the size to be long enough for a global
        // message with 0 bytes payload. So source address and MTI is ok to
        // parse now.
        tgt->mti = (Defs::MTI)data_to_error(msg + MSG_MTI_OFS);
        tgt->src.clear();
        tgt->dst.clear();
        tgt->src.id = data_to_node_id(msg + MSG_SRC_OFS);
        // now contains the length of the message after the header.
        int payload_bytes = expected_size - HDR_LEN;
        int payload_ofs;
        if (Defs::get_mti_address(tgt->mti))
        {
            payload_ofs = MSG_ADR_PAYLOAD_OFS;
            tgt->dst.id = data_to_node_id(msg + MSG_DST_OFS);
        }
        else
        {
            payload_ofs = MSG_GLOBAL_PAYLOAD_OFS;
        }
        payload_bytes -= payload_ofs;
        if (payload_bytes < 0)
        {
            LOG(WARNING, "TCP message to short.");
            return false;
        }
        tgt->payload.assign(msg + payload_ofs, payload_bytes);
        return true;
    }

    /// @param tcp_payload is a message holding a TCP protocol frame.
    /// @return the OpenLCB priority in range 0..3 or uint_max if the message
    /// is broken.
    static unsigned guess_priority(const string &tcp_payload)
    {
        if (tcp_payload.size() < ABS_MTI_OFS + 2)
        {
            return UINT_MAX;
        }
        auto mti = (Defs::MTI)data_to_error(tcp_payload.data() + ABS_MTI_OFS);
        return Defs::mti_priority(mti);
    }

    enum
    {
        /// Bit in the uint16 flags field in the header that signals frames that
        /// carry an OpenLCB message.
        FLAGS_OPENLCB_MSG = 0x8000,
        /// Chaining bit in the uint16 flags field. Means (according to the old
        /// draft) that there will be anopther header embedded inside this.
        FLAGS_CHAINING = 0x4000,
        /// Reserved bits in the flags field. Check as zero.
        FLAGS_RESVD1_ZERO_CHECK = 0x3000,
        /// "not first" fragmenting bit in the flags field.
        FLAGS_FRAGMENT_NOT_FIRST = 0x0800,
        /// "not last" fragmenting bit in the flags field.
        FLAGS_FRAGMENT_NOT_LAST = 0x0400,
        /// Reserved bits in the flags field. Ignore (by the standard).
        FLAGS_RESVD2_IGNORED = 0x03FF,

        /// Offset in the header of the flags field.
        HDR_FLAG_OFS = 0,
        /// Offset in the header of the size field.
        HDR_SIZE_OFS = 2,
        /// Offset in the header of the source gateway field.
        HDR_GATEWAY_OFS = 2 + 3,
        /// Offset in the header of the end of the size field.
        HDR_SIZE_END = HDR_GATEWAY_OFS,
        /// Offset in the header of the timestamp (seq no) field.
        HDR_TIMESTAMP_OFS = 2 + 3 + 6,
        /// Total length of the fixed header.
        HDR_LEN = 2 + 3 + 6 + 6,

        /// Offset in the message (=after the header) of the MTI field.
        MSG_MTI_OFS = 0,
        /// Offset in the message (=after the header) of the src node ID field.
        MSG_SRC_OFS = 2,
        /// Offset in the message (=after the header) of the dst node ID field
        /// (for addressed MTI only).
        MSG_DST_OFS = 2 + 6,
        /// Offset in the message (=after the header) of the message payload
        /// (for addressed MTI only).
        MSG_ADR_PAYLOAD_OFS = 2 + 6 + 6,

        /// Offset in the message (=after the header) of the message payload
        /// (for global MTI only).
        MSG_GLOBAL_PAYLOAD_OFS = MSG_DST_OFS,

        /// Minimum length of a valid message.
        MIN_MESSAGE_SIZE = MSG_GLOBAL_PAYLOAD_OFS + HDR_LEN,
        /// Minimum length of a valid message that has an addressed MTI.
        MIN_ADR_MESSAGE_SIZE = MSG_ADR_PAYLOAD_OFS + HDR_LEN,

        /// Offset from the header of the MTI field in the message. Assumes no
        /// chaining.
        ABS_MTI_OFS = HDR_LEN + MSG_MTI_OFS,
    };

private:
    /// No usable constructor; this is a static-only class.
    TcpDefs();
};

///   Virtual clock interface. Implementations are not required to be
///   thread-safe.
class SequenceNumberGenerator : public Destructable
{
public:
    /// Returns the next strictly monotonic sequence number.
    virtual long long get_sequence_number() = 0;
};

/// Implementation of sequence number generator that uses the real clock. Not
/// thread-safe.
class ClockBaseSequenceNumberGenerator : public SequenceNumberGenerator
{
public:
    /// @return next sequence number based on clock time.
    long long get_sequence_number() override
    {
        long long current_time_ms = os_get_time_monotonic() / 1000000;
        if (current_time_ms > sequence_)
        {
            sequence_ = current_time_ms;
        }
        else
        {
            ++sequence_;
        }
        return sequence_;
    }

private:
    /// Sequence number of last sent message.
    long long sequence_ = 0;
};

/// This flow renders outgoing OpenLCB messages to their TCP stream
/// representation.
class TcpSendFlow : public MessageStateFlowBase
{
public:
    /// Constructor.
    ///
    /// @param service the owning TCP interface.
    /// @param gateway_node_id our own node ID that will be put into the gateway
    /// node ID field of outgoing TCP messages.
    /// @param send_target where to send rendered TCP messages (this is usually
    /// either the device FD flow or a hub with the binary rendered packets).
    /// @param skip_member outgoing packets will get their skipMember_ set to
    /// this value. Usually the matching read flow of the interface to avoid
    /// undesired echo.
    /// @param sequence how to generate sequence numbers for the outgoing
    /// packets.
    TcpSendFlow(If *service, NodeID gateway_node_id,
        HubPortInterface *send_target, HubPortInterface *skip_member,
        SequenceNumberGenerator *sequence)
        : MessageStateFlowBase(service)
        , sendTarget_(send_target)
        , skipMember_(skip_member)
        , gatewayId_(gateway_node_id)
        , sequenceNumberGenerator_(sequence)
    {
    }

private:
    /// Handler where dequeueing of messages to be sent starts.
    /// @return next state
    Action entry() override
    {
        auto id = nmsg()->dst.id;
        if (id)
        {
            auto *n = iface()->lookup_local_node(id);
            if (n)
            {
                // Addressed message loopback.
                auto *b = transfer_message();
                b->data()->dstNode = n;
                iface()->dispatcher()->send(b, priority());
                return release_and_exit();
            }
        }
        return allocate_and_call(sendTarget_, STATE(render_src_message));
    }

    /// Callback state after allocation succeeded. Renders the message to binary
    /// payload into the allocated buffer, send the message and exits the
    /// processing.
    /// @return back to the base state.
    Action render_src_message()
    {
        auto b = get_buffer_deleter(get_allocation_result(sendTarget_));
        TcpDefs::render_tcp_message(*nmsg(), gatewayId_,
            sequenceNumberGenerator_->get_sequence_number(), b->data());
        b->data()->skipMember_ = skipMember_;
        sendTarget_->send(b.release(), nmsg()->priority());

        // Checks and performs global loopback.
        if (!nmsg()->dst.id)
        {
            iface()->dispatcher()->send(transfer_message(), priority());
        }
        return release_and_exit();
    }

    /// @return the abstract message we are trying to send.
    GenMessage *nmsg()
    {
        return message()->data();
    }

    /// @return owning interface object.
    If *iface()
    {
        return static_cast<If *>(service());
    }

    /// Where to send the rendered messages to.
    HubPortInterface *sendTarget_;
    /// This value will be populated to the skipMember_ field.
    HubPortInterface *skipMember_;
    /// Populated into the source gateway field of the outgoing messages.
    NodeID gatewayId_;
    /// Responsible for generating the sequence numbers of the outgoing
    /// messages.
    SequenceNumberGenerator *sequenceNumberGenerator_;
};

/// This flow is listening to data from a TCP connection, segments the incoming
/// data into TcpMessages based on the incoming size, and forwards packets
/// containing the TCP message as string payload.
class FdToTcpParser : public StateFlowBase
{
public:
    /// Constructor.
    /// @param s the parent (owning) service that holds the different flows
    /// together.
    /// @param dst where to forward the segmented packets.
    /// @param skipMember all forwarded messages will have their skipMember set
    /// to this value. Usually the output port.
    FdToTcpParser(FdHubPortService *s, HubPortInterface *dst,
        HubPortInterface *skipMember)
        : StateFlowBase(s)
        , dst_(dst)
        , skipMember_(skipMember)
    {
        HASSERT(s->fd() >= 0);
        bufEnd_ = 0;
        bufOfs_ = 0;
        start_flow(STATE(start_msg));
    }

    /// Stops listening and terminates the flow.
    void shutdown()
    {
        auto *e = this->service()->executor();
        if (e->is_selected(&helper_))
        {
            e->unselect(&helper_);
            helper_.remaining_ = 0;
        }
        set_terminated();
        notify_barrier();
    }

private:
    /// @return the typed service
    FdHubPortService *device()
    {
        return static_cast<FdHubPortService *>(service());
    }

    /// In this state the internal buffer (bufOfs_) points at the beginning of a
    /// message.
    /// @return next state.
    Action start_msg()
    {
        msg_.clear();
        expectedLen_ = -1;
        return parse_bytes();
    }

    /// Intermediate state where we are
    /// - figuring out what is the length of the message we are parsing
    /// - allocating the output buffer
    /// - copying bytes of the message from the internal buffer into the output
    ///   buffer
    /// @return next state
    Action parse_bytes()
    {
        if (bufEnd_ <= bufOfs_)
        {
            return call_immediately(STATE(read_more_bytes));
        }
        int available = bufEnd_ - bufOfs_;
        if (expectedLen_ < 0)
        {
            // We have bytes in the read buffer but don't know the expected
            // size yet, so none in the assembly buffer. Let's guess at the
            // desired size of the assembly buffer.
            expectedLen_ =
                TcpDefs::get_tcp_message_len(buffer_ + bufOfs_, available);
            if (expectedLen_ < 0)
            {
                // failed. Not enough bytes. Maybe move existing bytes to the
                // beginning of the buffer and try to read more bytes.
                if (bufOfs_ > (READ_BUFFER_SIZE / 2))
                {
                    memmove(buffer_, buffer_ + bufOfs_, available);
                    bufEnd_ -= bufOfs_;
                    bufOfs_ = 0;
                }
                return call_immediately(STATE(read_more_bytes));
            }
        }
        // now: we have an expected length.
        DASSERT(expectedLen_ > 0);
        if (msg_.empty())
        {
            msg_.reserve(expectedLen_);
        }
        // Copy some bytes to the assembly buffer.
        int needed = expectedLen_ - msg_.size();
        if (needed > available)
        {
            needed = available;
        }
        msg_.append((const char *)(buffer_ + bufOfs_), needed);
        bufOfs_ += needed;
        if (msg_.size() >= (unsigned)expectedLen_)
        {
            // we're done.
            return send_entry();
        }
        else
        {
            HASSERT(bufOfs_ >= bufEnd_);
            return call_immediately(STATE(read_more_bytes));
        }
    }

    /// Helper state to call the stateflow kernel to read bytes from the fd into
    /// the internal buffer.
    /// @return next state
    Action read_more_bytes()
    {
        if (bufEnd_ <= bufOfs_)
        {
            bufOfs_ = 0;
            bufEnd_ = 0;
        }
        return read_single(&helper_, device()->fd(), buffer_ + bufEnd_,
            READ_BUFFER_SIZE - bufEnd_, STATE(read_done), READ_PRIO);
    }

    /// Callback state when the kernel read is completed.
    /// @return next state
    Action read_done()
    {
        if (helper_.hasError_)
        {
            notify_barrier();
            set_terminated();
            device()->report_read_error();
            return exit();
        }
        bufEnd_ = READ_BUFFER_SIZE - helper_.remaining_;
        return parse_bytes();
    }

    /// Sends an assembled message (1) to the destination flow.
    /// @return next state.
    Action send_entry()
    {
        return allocate_and_call(dst_, STATE(have_alloc_msg));
    }

    /// Sends an assembled message (2) to the destination flow.
    /// @return next state.
    Action have_alloc_msg()
    {
        auto *b = get_allocation_result(dst_);
        auto prio = TcpDefs::guess_priority(msg_);
        b->data()->assign(std::move(msg_));
        b->data()->skipMember_ = skipMember_;
        /// @todo (balazs.racz): there should be some form of throttling here,
        /// ot not read more bytes from the tcp socket than how much RAM we
        /// have available.
        dst_->send(b, prio);
        return call_immediately(STATE(start_msg));
    }

    /// Calls into the parent flow's barrier notify, but makes sure to
    /// only do this once in the lifetime of *this.
    void notify_barrier()
    {
        if (barrierOwned_)
        {
            barrierOwned_ = false;
            device()->barrier_.notify();
        }
    }

    /// We attempt to read this many bytes in one go from the FD.
    static constexpr unsigned READ_BUFFER_SIZE = 300;
    /// If we have to guess at the size of the packet, we start by allocating
    /// this many bytes.
    static constexpr unsigned DEFAULT_PACKET_SIZE =
        TcpDefs::MIN_ADR_MESSAGE_SIZE + 16;
    static constexpr unsigned MIN_SIZE_GUESS = TcpDefs::HDR_SIZE_END;
    /// What priority to use for reads from fds.
    static constexpr unsigned READ_PRIO = Selectable::MAX_PRIO;

    /// Temporary buffer to read into from the FD.
    uint8_t buffer_[READ_BUFFER_SIZE];
    /// true iff pending parent->barrier_.notify()
    uint8_t barrierOwned_{1};
    /// Offset of the end in the read buffer.
    uint16_t bufEnd_;
    /// First active byte (offset of the beginning) in the read buffer.
    uint16_t bufOfs_;
    /// How many bytes we think the current message will be. -1 if we don't
    /// know yet.
    int expectedLen_;
    /// Assembly buffer. Holds the captured message so far.
    string msg_;
    /// Where to send parsed messages to.
    HubPortInterface *dst_;
    /// Parsed messages will be initialized to this skipMember_.
    HubPortInterface *skipMember_;
    StateFlowSelectHelper helper_{this};
};

using TcpHubDeviceSelect = HubDeviceSelect<HubFlow, FdToTcpParser>;

/// Simple stateless translator for incoming TCP messages from binary format
/// into the structured format. Drops everything to the floor that is not a
/// valid TCP message. Performs synchronous allocation and keeps the done
/// callback passed along.
class TcpRecvFlow : public HubPortInterface, public Destructable
{
public:
    /// @param target is where to send the parsed messages. Usually the
    /// interface's dispatcher flow.
    TcpRecvFlow(MessageHandler *target)
        : target_(target)
    {
    }

    /// Entry point for the incoming (binary) data. These are already segmented
    /// correctly to openlcb-TCP packet boundaries.
    /// @param data buffer
    /// @param prio priority
    void send(Buffer<HubData> *data, unsigned prio) override
    {
        auto src = get_buffer_deleter(data);
        auto dst = get_buffer_deleter(target_->alloc());
        dst->set_done(data->new_child());
        if (TcpDefs::parse_tcp_message(*src->data(), dst->data()))
        {
            target_->send(dst.release(), prio);
        }
    }

private:
    /// Flow(interface) where to pass on the parsed GenMessage.
    MessageHandler *target_;
};

} // namespace openlcb

#endif // _OPENLCB_IFTCPIMPL_HXX_
