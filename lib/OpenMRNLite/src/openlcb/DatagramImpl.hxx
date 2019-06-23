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

#include "openlcb/Datagram.hxx"
#include "openlcb/DatagramDefs.hxx"

namespace openlcb
{

/// Defines how long the datagram client flow should wait for the datagram
/// ack/nack response message.
extern long long DATAGRAM_RESPONSE_TIMEOUT_NSEC;

/// Datagram client implementation for CANbus-based datagram protocol.
///
/// This flow is responsible for the outgoing CAN datagram framing, and listens
/// for incoming datagram response messages.
///
/// The base class of AddressedCanMessageWriteFlow is responsible for the
/// discovery and address resolution of the destination node.
class DatagramClientImpl : public DatagramClient,
                           public StateFlowBase,
                           public LinkedObject<DatagramClientImpl>
{
public:
    /// Constructor.
    /// @param iface is the service on which to run this flow
    /// @param send_flow can receive an (addressed) Datagram message and send
    /// it to the appropriate destination -- takes care of fragmenting etc.
    DatagramClientImpl(If *iface, MessageHandler *send_flow)
        : StateFlowBase(iface)
        , sendFlow_(send_flow)
        , listener_(this)
        , isSleeping_(0)
        , sendPending_(0)
    {
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
        start_flow(STATE(start_send));
    }

    /** Requests cancelling the datagram send operation. Will notify the done
     * callback when the canceling is completed. */
    void cancel() OVERRIDE
    {
        DIE("Canceling datagram send operation is not yet implemented.");
    }

private:
    /// Equivalent to enqueuing a new datagram to send.
    /// @param b datagram to send.
    /// @param priority executor priority.
    void reset_message(Buffer<GenMessage> *b, unsigned priority)
    {
        set_priority(priority);
        message_ = b;
    }

    /// Entry point to the flow processing.
    /// @return next state
    Action start_send()
    {
        iface()->canonicalize_handle(&message_->data()->src);
        iface()->canonicalize_handle(&message_->data()->dst);
        src_ = message_->data()->src;
        dst_ = message_->data()->dst;
        return acquire_srcdst_lock();
    }

    /// Ensures that there is no other datagram client with the same src:dst
    /// pair. This is required by the standard.
    /// @return next state
    Action acquire_srcdst_lock()
    {
        // First check if there is another datagram client sending a datagram
        // to the same target node.
        {
            AtomicHolder h(LinkedObject<DatagramClientImpl>::head_mu());
            for (DatagramClientImpl *c =
                     LinkedObject<DatagramClientImpl>::head_;
                 c; c = c->LinkedObject<DatagramClientImpl>::link_next())
            {
                // this will catch c == this.
                if (!c->sendPending_) continue;
                if (c->src_.id != src_.id) continue; 
                if (!iface()->matching_node(c->dst_, dst_))
                    continue;
                // Now: there is another datagram client sending a datagram to
                // this destination. We need to wait for that transaction to
                // complete.
                c->waitingClients_.push_front(this);
                return wait();
            }
        }

        return do_send();
    }

    /// Hands off the datagram to the send flow.
    /// @return next state.
    Action do_send()
    {
        auto *b = message_;
        message_ = nullptr;
        // These two statements transfer the barrier's ownership from the
        // BufferBase to our pointer variable.
        done_ = b->new_child();
        b->set_done(nullptr);

        register_handlers();
        // Transfers ownership.
        sendFlow_->send(b, priority_);

        isSleeping_ = 1;
        return sleep_and_call(&timer_, DATAGRAM_RESPONSE_TIMEOUT_NSEC,
            STATE(timeout_waiting_for_dg_response));
    }

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
        iface()->dispatcher()->register_handler(&listener_, MTI_1, MASK_1);
        iface()->dispatcher()->register_handler(&listener_, MTI_2, MASK_2);
        iface()->dispatcher()->register_handler(&listener_, MTI_3, MASK_3);
    }

    /// @todo In IfCanImpl.hxx there is a timeout_looking_for_dst action. It
    /// should trigger a 'terminate due to error' response message, and when
    /// that arrives in the handle_response() function below, this is the code
    /// that we need to trigger.
    Action timeout_looking_for_dst()
    {
        result_ |= PERMANENT_ERROR | DST_NOT_FOUND;
        unregister_response_handler();
        return call_immediately(STATE(datagram_finalize));
    }

    Action timeout_waiting_for_dg_response()
    {
        LOG(INFO,
            "CanDatagramWriteFlow: No datagram response arrived from "
            "destination %012" PRIx64 ".",
            dst_.id);
        isSleeping_ = 0;
        unregister_response_handler();
        result_ |= PERMANENT_ERROR | TIMEOUT;
        return call_immediately(STATE(datagram_finalize));
    }

    void unregister_response_handler()
    {
        iface()->dispatcher()->unregister_handler(&listener_, MTI_1, MASK_1);
        iface()->dispatcher()->unregister_handler(&listener_, MTI_2, MASK_2);
        iface()->dispatcher()->unregister_handler(&listener_, MTI_3, MASK_3);
        sendPending_ = 0;
        if (!waitingClients_.empty())
        {
            DatagramClientImpl *c =
                static_cast<DatagramClientImpl *>(waitingClients_.pop_front());
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
        if (done_)
        {
            done_->notify();
            done_ = nullptr;
        }
        return set_terminated();
    }

    /** This object is registered to receive response messages at the interface
     * level. Then it forwards the call to the parent DatagramClientImpl. */
    class ReplyListener : public MessageHandler
    {
    public:
        ReplyListener(DatagramClientImpl *parent)
            : parent_(parent)
        {
        }

        void send(message_type *buffer, unsigned priority = UINT_MAX) OVERRIDE
        {
            parent_->handle_response(buffer->data());
            buffer->unref();
        }

    private:
        DatagramClientImpl *parent_;
    };

    /// Callback when a matching response comes in on the bus.
    /// @param message is the incoming generic message (from the response
    /// buffer).
    void handle_response(GenMessage *message)
    {
        // LOG(INFO, "%p: Incoming response to datagram: mti %x from %x", this,
        //    (int)message->mti, (int)message->src.alias);

        // Check for reboot (unaddressed message) first.
        if (message->mti == Defs::MTI_INITIALIZATION_COMPLETE)
        {
            if (message->payload.size() != 6)
            {
                // Malformed message inbound.
                return;
            }
            NodeHandle rebooted(message->src);
            rebooted.id = buffer_to_node_id(message->payload);
            if (iface()->matching_node(dst_, rebooted))
            {
                // Destination node has rebooted. Kill datagram flow.
                result_ |= DST_REBOOT;
                return stop_waiting_for_response();
            }
            return; // everything else below is for addressed message
        }

        // First we check that the response is for this source node.
        if (!iface()->matching_node(message->dst, src_))
        {
            LOG(VERBOSE, "wrong dst");
            return;
        }
        // We also check that the source of the response is our destination.
        if (!iface()->matching_node(message->src, dst_))
        {
            LOG(VERBOSE, "wrong src");
            return;
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
        if (isSleeping_)
        {
            // Stops waiting for response and notifies the current flow.
            timer_.trigger();
            isSleeping_ = 0;
        }
        /// @TODO(balazs.racz) Here we might want to decide whether to start a
        /// retry.
        LOG(VERBOSE, "restarting at datagram finalize");
        reset_flow(STATE(datagram_finalize));
    }

    /// Overrides the default notify implementation to make sure we obey the
    /// priority values.
    void notify() override
    {
        service()->executor()->add(this, priority_);
    }

    /// Sets the stateflow priority.
    /// @param p the stateflow's priority on the executor.
    void set_priority(unsigned p)
    {
        priority_ = std::min((unsigned)MAX_PRIORITY, p);
    }

    /// @return the interface service we are running on.
    If *iface()
    {
        return static_cast<If *>(service());
    }

    /// Datagram message we are trying to send now. We own it.
    Buffer<GenMessage> *message_ {nullptr};
    /// This notifiable is saved from the datagram buffer. Will be notified
    /// when the entire interaction is completed, but the buffer itself is
    /// transferred to the send flow.
    BarrierNotifiable *done_ {nullptr};
    /// Source of the datagram we are currently sending.
    NodeHandle src_;
    /// Destination of the datagram we are currently sending.
    NodeHandle dst_;
    /// Addressed datagram send flow from the interface. Externally owned.
    MessageHandler *sendFlow_;
    /// Instance of the listener object.
    ReplyListener listener_;
    /// Helper object for sleep.
    StateFlowTimer timer_ {this};
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
    /// Priority in the executor.
    unsigned priority_ : 24;
    /// Constant used to clamp the incoming priority value to something that
    /// first in priority_ bit field.
    static constexpr unsigned MAX_PRIORITY = (1 << 24) - 1;
}; // class DatagramClientImpl

} // namespace openlcb
