/** \copyright
 * Copyright (c) 2014, Balazs Racz
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
 * \file DatagramCan.cxx
 *
 * CANbus datagram parser and renderer flows.
 *
 * @author Balazs Racz
 * @date 27 Jan 2013
 */

#include "openlcb/DatagramCan.hxx"

#include "openlcb/DatagramDefs.hxx"
#include "openlcb/IfCanImpl.hxx"

namespace openlcb
{

/// Defines how long the datagram client flow should wait for the datagram
/// ack/nack response message.
long long DATAGRAM_RESPONSE_TIMEOUT_NSEC = SEC_TO_NSEC(3);

/// Datagram client implementation for CANbus-based datagram protocol.
///
/// This flow is responsible for the outgoing CAN datagram framing, and listens
/// for incoming datagram response messages.
///
/// The base class of AddressedCanMessageWriteFlow is responsible for the
/// discovery and address resolution of the destination node.
class CanDatagramClient : public DatagramClient,
                          public AddressedCanMessageWriteFlow,
                          public LinkedObject<CanDatagramClient>
{
public:
    CanDatagramClient(IfCan *iface)
        : AddressedCanMessageWriteFlow(iface)
        , listener_(this)
        , isSleeping_(0)
        , sendPending_(0)
    {
        /** This flow does not use the incoming queue that we inherited from
         * AddressedCanMessageWriteFlow. We skip the wait state.
         *
         * @TODO(balazs.racz) consider skipping this flow into two parts: one
         * client that waits for the responses and one flow that just renders
         * the outgoing frames. */
        set_terminated();
    }

    void write_datagram(Buffer<GenMessage> *b, unsigned priority) OVERRIDE
    {
        if (!b->data()->mti)
        {
            b->data()->mti = Defs::MTI_DATAGRAM;
        }
        HASSERT(b->data()->mti == Defs::MTI_DATAGRAM);
        result_ = OPERATION_PENDING;
        reset_message(b, priority);
        start_flow(STATE(acquire_srcdst_lock));
    }

    Action acquire_srcdst_lock()
    {
        // First check if there is another datagram client sending a datagram
        // to the same target node.
        {
            AtomicHolder h(LinkedObject<CanDatagramClient>::head_mu());
            for (CanDatagramClient* c = LinkedObject<CanDatagramClient>::head_;
                 c;
                 c = c->LinkedObject<CanDatagramClient>::link_next()) {
                // this will catch c == this.
                if (!c->sendPending_) continue;
                if (c->nmsg()->src.id != nmsg()->src.id) continue; 
                if (!async_if()->matching_node(c->nmsg()->dst, nmsg()->dst))
                    continue;
                // Now: there is another datagram client sending a datagram to
                // this destination. We need to wait for that transaction to
                // complete.
                c->waitingClients_.push_front(this);
                return wait();
            }
        }
        register_handlers();
        /// @TODO(balazs.racz) this will not work for loopback messages because
        /// it calls transfer_message().
        return call_immediately(STATE(addressed_entry));
    }
    
    Action send_to_local_node() OVERRIDE
    {
        return allocate_and_call(async_if()->dispatcher(),
                                 STATE(local_copy_allocated));
    }

    Action local_copy_allocated()
    {
        auto *b = get_allocation_result(async_if()->dispatcher());
        b->data()->reset(nmsg()->mti, nmsg()->src.id, nmsg()->dst, Payload());
        b->data()->payload.swap(nmsg()->payload);
        b->data()->dstNode = nmsg()->dstNode;
        async_if()->dispatcher()->send(b);
        return call_immediately(STATE(send_finished));
    }

    /** Requests cancelling the datagram send operation. Will notify the done
     * callback when the canceling is completed. */
    void cancel() OVERRIDE
    {
        DIE("Canceling datagram send operation is not yet implemented.");
    }

private:
    enum
    {
        MTI_1a = Defs::MTI_TERMINATE_DUE_TO_ERROR,
        MTI_1b = Defs::MTI_OPTIONAL_INTERACTION_REJECTED,
        MASK_1 = ~(MTI_1a ^ MTI_1b),
        MTI_1 = MTI_1a,
        MTI_2a = Defs::MTI_DATAGRAM_OK,
        MTI_2b = Defs::MTI_DATAGRAM_REJECTED,
        MASK_2 = ~(MTI_2a ^ MTI_2b),
        MTI_2 = MTI_2a,
        MTI_3 = Defs::MTI_INITIALIZATION_COMPLETE,
        MASK_3 = Defs::MTI_EXACT,
    };

    void register_handlers()
    {
        hasResponse_ = 0;
        isSleeping_ = 0;
        sendPending_ = 1;
        if_can()->dispatcher()->register_handler(&listener_, MTI_1, MASK_1);
        if_can()->dispatcher()->register_handler(&listener_, MTI_2, MASK_2);
        if_can()->dispatcher()->register_handler(&listener_, MTI_3, MASK_3);
    }

    Action fill_can_frame_buffer() OVERRIDE
    {
        LOG(VERBOSE, "fill can frame buffer");
        auto *b = get_allocation_result(if_can()->frame_write_flow());
        struct can_frame *f = b->data()->mutable_frame();
        HASSERT(nmsg()->mti == Defs::MTI_DATAGRAM);

        // Sets the CAN id.
        uint32_t can_id = 0x1A000000;
        CanDefs::set_src(&can_id, srcAlias_);
        LOG(VERBOSE, "dst alias %x", dstAlias_);
        CanDefs::set_dst(&can_id, dstAlias_);

        bool need_more_frames = false;
        unsigned len = nmsg()->payload.size() - dataOffset_;
        if (len > 8)
        {
            len = 8;
            // This is not the last frame.
            need_more_frames = true;
            if (dataOffset_)
            {
                CanDefs::set_can_frame_type(&can_id,
                                            CanDefs::DATAGRAM_MIDDLE_FRAME);
            }
            else
            {
                CanDefs::set_can_frame_type(&can_id,
                                            CanDefs::DATAGRAM_FIRST_FRAME);
            }
        }
        else
        {
            // No more data after this frame.
            if (dataOffset_)
            {
                CanDefs::set_can_frame_type(&can_id,
                                            CanDefs::DATAGRAM_FINAL_FRAME);
            }
            else
            {
                CanDefs::set_can_frame_type(&can_id,
                                            CanDefs::DATAGRAM_ONE_FRAME);
            }
        }

        memcpy(f->data, &nmsg()->payload[dataOffset_], len);
        dataOffset_ += len;
        f->can_dlc = len;

        SET_CAN_FRAME_ID_EFF(*f, can_id);
        if_can()->frame_write_flow()->send(b);

        if (need_more_frames)
        {
            return call_immediately(STATE(get_can_frame_buffer));
        }
        else
        {
            return call_immediately(STATE(send_finished));
        }
    }

    Action send_finished() OVERRIDE
    {
        isSleeping_ = 1;
        return sleep_and_call(&timer_, DATAGRAM_RESPONSE_TIMEOUT_NSEC,
                              STATE(timeout_waiting_for_dg_response));
    }

    Action timeout_looking_for_dst() OVERRIDE
    {
        result_ |= PERMANENT_ERROR | DST_NOT_FOUND;
        unregister_response_handler();
        return call_immediately(STATE(datagram_finalize));
    }

    /// @todo( balazs.racz): why is this virtual?
    virtual Action timeout_waiting_for_dg_response()
    {
        LOG(INFO, "CanDatagramWriteFlow: No datagram response arrived from "
                  "destination %012" PRIx64 ".",
            nmsg()->dst.id);
        isSleeping_ = 0;
        unregister_response_handler();
        result_ |= PERMANENT_ERROR | TIMEOUT;
        return call_immediately(STATE(datagram_finalize));
    }

    void unregister_response_handler()
    {
        if_can()->dispatcher()->unregister_handler(&listener_, MTI_1, MASK_1);
        if_can()->dispatcher()->unregister_handler(&listener_, MTI_2, MASK_2);
        if_can()->dispatcher()->unregister_handler(&listener_, MTI_3, MASK_3);
        sendPending_ = 0;
        if (!waitingClients_.empty()) {
            CanDatagramClient* c = static_cast<CanDatagramClient*>(waitingClients_.pop_front());
            // Hands off all waiting clients to c.
            HASSERT(c->waitingClients_.empty());
            std::swap(waitingClients_, c->waitingClients_);
            c->notify();
        }
    }

    Action datagram_finalize()
    {
        HASSERT(!sendPending_);
        HASSERT(result_ & OPERATION_PENDING);
        result_ &= ~OPERATION_PENDING;
        release();
        return set_terminated();
    }

    /** This object is registered to receive response messages at the interface
     * level. Then it forwards the call to the parent CanDatagramClient. */
    class ReplyListener : public MessageHandler
    {
    public:
        ReplyListener(CanDatagramClient *parent) : parent_(parent)
        {
        }

        void send(message_type *buffer, unsigned priority = UINT_MAX) OVERRIDE
        {
            parent_->handle_response(buffer->data());
            buffer->unref();
        }

    private:
        CanDatagramClient *parent_;
    };

    /// Callback when a matching response comes in on the bus.
    void handle_response(GenMessage *message)
    {
        //LOG(INFO, "%p: Incoming response to datagram: mti %x from %x", this,
        //    (int)message->mti, (int)message->src.alias);

        // Check for reboot (unaddressed message) first.
        if (message->mti == Defs::MTI_INITIALIZATION_COMPLETE) {
            if (message->payload.size() != 6) {
                // Malformed message inbound.
                return;
            }
            NodeHandle rebooted(message->src);
            rebooted.id = buffer_to_node_id(message->payload);
            if (if_can()->matching_node(nmsg()->dst, rebooted))
            {
                // Destination node has rebooted. Kill datagram flow.
                result_ |= DST_REBOOT;
                return stop_waiting_for_response();
            }
            return; // everything else below is for addressed message
        }

        // First we check that the response is for this source node.
        if (message->dst.id)
        {
            if (message->dst.id != nmsg()->src.id)
            {
                LOG(VERBOSE, "wrong dst");
                return;
            }
        }
        else if (message->dst.alias != srcAlias_)
        {
            LOG(VERBOSE, "wrong dst alias");
            /* Here we hope that the source alias was not released by the time
             * the response comes in. */
            return;
        }
        // We also check that the source of the response is our destination.
        if (message->src.id && nmsg()->dst.id)
        {
            if (message->src.id != nmsg()->dst.id)
            {
                LOG(VERBOSE, "wrong src");
                return;
            }
        }
        else if (message->src.alias)
        {
            // We hope the dstAlias_ has not changed yet.
            if (message->src.alias != dstAlias_)
            {
                LOG(VERBOSE, "wrong src alias %x %x", (int)message->src.alias,
                    (int)dstAlias_);
                return;
            }
        }
        else
        {
            /// @TODO(balazs.racz): we should initiate an alias lookup here.
            HASSERT(0); // Don't know how to match the response source.
        }

        uint16_t error_code = 0;
        uint8_t payload_length = 0;
        const uint8_t *payload = nullptr;
        if (!message->payload.empty())
        {
            payload =
                reinterpret_cast<const uint8_t *>(message->payload.data());
            payload_length = message->payload.size();
        }
        if (payload_length >= 2)
        {
            error_code = (((uint16_t)payload[0]) << 8) | payload[1];
        }

        switch (message->mti)
        {
            case Defs::MTI_TERMINATE_DUE_TO_ERROR:
            case Defs::MTI_OPTIONAL_INTERACTION_REJECTED:
            {
                if (payload_length >= 4)
                {
                    uint16_t return_mti = payload[2];
                    return_mti <<= 8;
                    return_mti |= payload[3];
                    if (return_mti != Defs::MTI_DATAGRAM)
                    {
                        // This must be a rejection of some other
                        // message. Ignore.
                        LOG(VERBOSE, "wrong rejection mti");
                        return;
                    }
                }
            } // fall through
            case Defs::MTI_DATAGRAM_REJECTED:
            {
                result_ &= ~0xffff;
                result_ |= error_code;
                // Ensures that an error response is visible in the flags.
                if (!(result_ & (PERMANENT_ERROR | RESEND_OK)))
                {
                    result_ |= PERMANENT_ERROR;
                }
                break;
            }
            case Defs::MTI_DATAGRAM_OK:
            {
                if (payload_length)
                {
                    result_ &= ~(0xff << RESPONSE_FLAGS_SHIFT);
                    result_ |= payload[0] << RESPONSE_FLAGS_SHIFT;
                }
                result_ |= OPERATION_SUCCESS;
                break;
            }
            default:
                // Ignore message.
                LOG(VERBOSE, "unknown mti");
                return;
        } // switch response MTI
        stop_waiting_for_response();
    } // handle_message

    /// To be called from the handler. Wakes up main flow and terminates it
    /// (with whatever is in the result_ code right now).
    void stop_waiting_for_response()
    {
        // Avoids duplicate wakeups on the timer.
        unregister_response_handler();
        hasResponse_ = 1;
        if (isSleeping_) {
            // Stops waiting for response and notifies the current flow.
            timer_.trigger();
            isSleeping_ = 0;
        }
        /// @TODO(balazs.racz) Here we might want to decide whether to start a
        /// retry.
        LOG(VERBOSE, "restarting at datagram finalize");
        reset_flow(STATE(datagram_finalize));
    }

    ReplyListener listener_;
    /// List of other datagram clients that are trying to send to the same
    /// target node. We need to wake up one of this list when we are done
    /// sending.
    TypedQueue<Executable> waitingClients_;
    /// 1 when we are in the sleep call waiting for the datagram Ack or Reject
    /// message.
    unsigned isSleeping_ : 1;
    unsigned hasResponse_ : 1;
    /// 1 when we have the handlers registered. During this time we have
    /// exclusive lock on the specific src/dst node pair.
    unsigned sendPending_ : 1;
};

/** Frame handler that assembles incoming datagram fragments into a single
 * datagram message. (That is, datagrams addressed to local nodes.) */
class CanDatagramParser : public CanFrameStateFlow
{
public:
    enum
    {
        CAN_FILTER = CanMessageData::CAN_EXT_FRAME_FILTER |
            (CanDefs::NMRANET_MSG << CanDefs::FRAME_TYPE_SHIFT) |
            (CanDefs::NORMAL_PRIORITY << CanDefs::PRIORITY_SHIFT),
        CAN_MASK = CanMessageData::CAN_EXT_FRAME_MASK |
            CanDefs::FRAME_TYPE_MASK | CanDefs::PRIORITY_MASK |
            CanDefs::CAN_FRAME_TYPE_MASK,
    };

    CanDatagramParser(IfCan *iface);
    ~CanDatagramParser();

    /// Handler callback for incoming frames.
    Action entry() override
    {
        errorCode_ = 0;
        const struct can_frame *f = &message()->data()->frame();

        uint32_t id = GET_CAN_FRAME_ID_EFF(*f);
        unsigned can_frame_type = (id & CanDefs::CAN_FRAME_TYPE_MASK) >>
                                  CanDefs::CAN_FRAME_TYPE_SHIFT;

        if (can_frame_type < 2 || can_frame_type > 5)
        {
            // Not datagram frame.
            return release_and_exit();
        }

        srcAlias_ = (id & CanDefs::SRC_MASK) >> CanDefs::SRC_SHIFT;

        uint64_t buffer_key = id & (CanDefs::DST_MASK | CanDefs::SRC_MASK);

        dst_.alias = buffer_key >> (CanDefs::DST_SHIFT);
        dstNode_ = nullptr;
        dst_.id = if_can()->local_aliases()->lookup(NodeAlias(dst_.alias));
        if (dst_.id)
        {
            dstNode_ = if_can()->lookup_local_node(dst_.id);
        }
        if (!dstNode_)
        {
            // Destination not local node.
            return release_and_exit();
        }

        DatagramPayload *buf = nullptr;
        bool last_frame = true;

        switch (can_frame_type)
        {
            case 2:
                // Single-frame datagram. Let's allocate one small buffer for
                // it.
                localBuffer_.clear();
                localBuffer_.reserve(f->can_dlc);
                buf = &localBuffer_;
                break;
            case 3:
            {
                // Datagram first frame
                auto it = pendingBuffers_.find(buffer_key);
                if (it != pendingBuffers_.end())
                {
                    pendingBuffers_.erase(it);
                    /** Frames came out of order or more than one datagram is
                     * being sent to the same dst. */
                    errorCode_ = DatagramClient::RESEND_OK |
                                 DatagramClient::OUT_OF_ORDER;
                    break;
                }

                buf = &pendingBuffers_[buffer_key];
                buf->clear();

                // Datagram first frame. Get a full buffer.
                buf->reserve(72);
                last_frame = false;
                break;
            }
            case 4:
                // Datagram middle frame
                last_frame = false;
            // Fall through
            case 5:
            {
                // Datagram last frame
                auto it = pendingBuffers_.find(buffer_key);
                if (it != pendingBuffers_.end())
                {
                    buf = &it->second;
                    if (last_frame)
                    {
                        localBuffer_.clear();
                        // Moves ownership of the allocated data to the local
                        // buffer.
                        localBuffer_.swap(*buf);
                        buf = &localBuffer_;
                        pendingBuffers_.erase(it);
                    }
                }
                break;
            }
            default:
                // Not datagram frame.
                return release_and_exit();
        }

        if (!buf)
        {
            errorCode_ =
                DatagramClient::RESEND_OK | DatagramClient::OUT_OF_ORDER;
        }
        else if (buf->size() + f->can_dlc > DatagramDefs::MAX_SIZE)
        {
            // Too long datagram arrived.
            LOG(WARNING, "AsyncDatagramCan: too long incoming datagram arrived."
                         " Size: %d",
                (int)(buf->size() + f->can_dlc));
            errorCode_ = DatagramClient::PERMANENT_ERROR;
            // Since we reject the datagram, let's not keep the buffer
            // around. This call should not crash if the buffer was already
            // deleted.
            pendingBuffers_.erase(buffer_key);
        }

        if (errorCode_)
        {
            release();
            // Gets the send flow to send rejection.
            return allocate_and_call(if_can()->addressed_message_write_flow(),
                                     STATE(send_rejection));
        }

        // Copies new data into buf.
        buf->append(reinterpret_cast<const char *>(&f->data[0]), f->can_dlc);
        release();
        if (last_frame)
        {
            HASSERT(buf == &localBuffer_);
            // Datagram is complete; let's send it to higher level If.
            return allocate_and_call(if_can()->dispatcher(),
                                     STATE(datagram_complete));
        }
        else
        {
            return exit();
        }
    }

    /** Sends a datagram rejection. The lock_ is held and must be
     * released. entry is an If::addressed write flow. errorCode_ != 0. */
    Action send_rejection()
    {
        HASSERT(errorCode_);
        HASSERT(dstNode_);
        auto *f =
            get_allocation_result(if_can()->addressed_message_write_flow());
        f->data()->reset(Defs::MTI_DATAGRAM_REJECTED, dst_.id, {0, srcAlias_},
                         error_to_buffer(errorCode_));
        if_can()->addressed_message_write_flow()->send(f);
        return exit();
    }

    /** Requests the datagram in buf_, dstNode_ etc... to be sent to the
     * AsyncIf for processing. The lock_ is held and must be released. entry is
     * the dispatcher. */
    Action datagram_complete()
    {
        HASSERT(!errorCode_);
        auto *f = get_allocation_result(if_can()->dispatcher());
        GenMessage *m = f->data();
        m->mti = Defs::MTI_DATAGRAM;
        m->payload.swap(localBuffer_);
        m->dst = dst_;
        m->dstNode = dstNode_;
        m->src.alias = srcAlias_;
        // This will be zero if the alias is not known.
        m->src.id =
            m->src.alias ? if_can()->remote_aliases()->lookup(m->src.alias) : 0;
        if (!m->src.id && m->src.alias)
        {
            // It's unlikely to have a datagram coming in on the interface with
            // a local alias and still framed into CAN frames. But we still
            // handle it.
            m->src.id = if_can()->local_aliases()->lookup(m->src.alias);
        }
        if_can()->dispatcher()->send(f);
        return exit();
    }

private:
    /// A local buffer that owns the datagram payload bytes after we took the
    /// entry from the pending buffers map.
    DatagramPayload localBuffer_;

    Node *dstNode_;
    NodeHandle dst_;
    unsigned short srcAlias_ : 12;
    /// If non-zero, contains a Rejection error code and the datagram should not
    /// be forwarded to the upper layer in this case.
    uint16_t errorCode_;

    /** Open datagram buffers. Keyed by (dstid | srcid), value is a datagram
     * payload. When a payload is finished, it should be moved into the final
     * datagram message using swap() to avoid memory copies.
     * @TODO(balazs.racz) we need some kind of timeout-based release mechanism
     * in here. */
    StlMap<uint64_t, DatagramPayload> pendingBuffers_;
};
CanDatagramService::CanDatagramService(IfCan *iface,
                                       int num_registry_entries,
                                       int num_clients)
    : DatagramService(iface, num_registry_entries)
{
    if_can()->add_owned_flow(new CanDatagramParser(if_can()));
    for (int i = 0; i < num_clients; ++i)
    {
        auto *client_flow = new CanDatagramClient(if_can());
        if_can()->add_owned_flow(client_flow);
        client_allocator()->insert(static_cast<DatagramClient *>(client_flow));
    }
}

Executable *TEST_CreateCanDatagramParser(IfCan *if_can)
{
    return new CanDatagramParser(if_can);
}

CanDatagramService::~CanDatagramService()
{
}

CanDatagramParser::CanDatagramParser(IfCan *iface)
    : CanFrameStateFlow(iface)
{
    if_can()->frame_dispatcher()->register_handler(this,
        CAN_FILTER |
            (CanDefs::DATAGRAM_ONE_FRAME << CanDefs::CAN_FRAME_TYPE_SHIFT),
        CAN_MASK &
            ~((CanDefs::DATAGRAM_ONE_FRAME ^ CanDefs::DATAGRAM_FIRST_FRAME)
                << CanDefs::CAN_FRAME_TYPE_SHIFT));
    if_can()->frame_dispatcher()->register_handler(this,
        CAN_FILTER |
            (CanDefs::DATAGRAM_MIDDLE_FRAME << CanDefs::CAN_FRAME_TYPE_SHIFT),
        CAN_MASK &
            ~((CanDefs::DATAGRAM_MIDDLE_FRAME ^ CanDefs::DATAGRAM_FINAL_FRAME)
                << CanDefs::CAN_FRAME_TYPE_SHIFT));
}

CanDatagramParser::~CanDatagramParser()
{
    if_can()->frame_dispatcher()->unregister_handler_all(this);
}

} // namespace openlcb
