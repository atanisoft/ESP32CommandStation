/** \copyright
 * Copyright (c) 2016, Balazs Racz
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
 * \file RoutingLogic.hxx
 *
 * Data structure for gateways and routers of OpenLCB, keeping track of the
 * necessary information to make routing decisions.
 *
 * @author Balazs Racz
 * @date 23 May 2016
 */

#ifndef _OPENLCB_CANROUTNGHUB_HXX_
#define _OPENLCB_CANROUTNGHUB_HXX_

#include "openlcb/RoutingLogic.hxx"
#include "openlcb/CanDefs.hxx"
#include "openlcb/Defs.hxx"
#include "openlcb/If.hxx"
#include "utils/Hub.hxx"
#include "utils/GcStreamParser.hxx"
#include "utils/gc_format.h"

namespace openlcb
{

/**
   A hub flow that accepts string HUB ports sending CAN frames via the
   GridConnect protocol, performs routing decisions on the frames and sends out
   to the appropriate ports.

   TODO: need to process consumer and producer identified messages.
   TODO: need to exclude CHECK ID frames from the source address learning.
 */
class GcCanRoutingHub : public HubPortInterface
{
public:
    typedef HubData value_type;
    typedef Buffer<value_type> buffer_type;
    typedef FlowInterface<buffer_type> port_type;

    GcCanRoutingHub(Service *s)
        : deliveryFlow_(s, this)
    {
    }

    void send(Buffer<HubData> *b, unsigned priority = UINT_MAX) override
    {
        OSMutexLock l(&lock_);
        void *port = b->data()->skipMember_;
        auto it = ports_.find(port);
        if (it == ports_.end())
        {
            LOG(INFO, "Arrived packet to routing hub without recognized source "
                      "designation (%p). Dropped packet.",
                b->data()->skipMember_);
            b->unref();
            return;
        }
        const string &p = *b->data();
        for (unsigned i = 0; i < p.size(); ++i)
        {
            if (it->second.segmenter_.consume_byte(p[i]))
            {
                // We have a frame.
                string ret;
                it->second.segmenter_.frame_buffer(&ret);
                LOG(VERBOSE, "sending frame: %s", ret.c_str());
                auto *cb = deliveryFlow_.alloc();
                HASSERT(it->second.segmenter_.parse_frame_to_output(
                    cb->data()->mutable_frame()));
                cb->data()->skipMember_ = reinterpret_cast<
                    FlowInterface<Buffer<HubContainer<CanFrameContainer>>> *>(
                    b->data()->skipMember_);
                deliveryFlow_.send(
                    cb, reprioritize_frame(cb->data()->frame(), priority));
            }
        }
    }

    CanHubPortInterface *can_hub()
    {
        /// @TODO: need a priority wrapper:
        // deliveryFlow_.send(b, reprioritize_frame(b->data()->frame(),
        // priority));
        return &deliveryFlow_;
    }

    void register_port(HubPortInterface *port)
    {
        OSMutexLock l(&lock_);
        HASSERT(port);
        ports_[port].hubPort_ = port;
    }

    void unregister_port(HubPortInterface *port)
    {
        OSMutexLock l(&lock_);
        auto it = ports_.find(port);
        if (it == ports_.end())
        {
            LOG(INFO, "Trying to remove a nonexistant port: %p", port);
            return;
        }
        it->second.inactive_ = true;
        pendingRemove_.push_back(port);
    }

private:
    class PortParser;
    typedef std::map<void *, PortParser> PortsMap;
    /**
       Computes the desired priority of a CAN frame.

       @param frame is the CAN frame (at the input side).
       @param old_priority is the incoming priority of the frame, as it arrived
       from the previous flow.
       @return the desired priority of the frame, in the range of 0..4.
     */
    unsigned reprioritize_frame(
        const struct can_frame &frame, unsigned old_priority)
    {
        // @TODO(balazs.racz): check actual priority.
        return old_priority;
    }

    /// Flow responsible for queuing outgoing CAN frames as well as sending out
    /// the actual frames to the recipients.
    class DeliveryFlow : public StateFlow<Buffer<CanHubData>, QList<5>>
    {
    public:
        DeliveryFlow(Service *s, GcCanRoutingHub *parent)
            : StateFlow<Buffer<CanHubData>, QList<5>>(s)
            , parent_(parent)
        {
        }

    private:
        Action entry() override
        {
            OSMutexLock l(&parent_->lock_);
            // First we apply any pending removes.
            for (void *p : parent_->pendingRemove_)
            {
                parent_->ports_.erase(p);
            }
            parent_->pendingRemove_.clear();

            // Classifies the packet.
            srcAddress_ = 0;
            dstAddress_ = 0;
            const struct can_frame &frame = message()->data()->frame();
            if (IS_CAN_FRAME_ERR(frame) || IS_CAN_FRAME_RTR(frame))
            {
                return release_and_exit();
            }
            classify_frame(frame);

            if (srcAddress_ != 0)
            {
                parent_->routingTable_.add_node_id_to_route(
                    message()->data()->skipMember_, srcAddress_);
            }

            gcBuf_ = nullptr;

            if (forwardType_ == ADDRESSED && dstAddress_ != 0)
            {
                void *port =
                    parent_->routingTable_.lookup_port_for_address(dstAddress_);
                nextIt_ = parent_->ports_.find(port);
                if (nextIt_ != parent_->ports_.end())
                {
                    // We found the desired port in the routing table.
                    return call_immediately(STATE(forward_addressed));
                }
                else
                {
                    forwardType_ = FORWARD_ALL;
                }
            }

            nextIt_ = parent_->ports_.begin();

            return call_immediately(STATE(try_next_entry));
        }

        /**
           Classifies the incoming frame and sets the class variables
           determining what to do with it.
         */
        void classify_frame(const can_frame &frame)
        {
            if (!IS_CAN_FRAME_EFF(frame))
            {
                forwardType_ = FORWARD_ALL;
                return;
            }
            uint32_t can_id = GET_CAN_FRAME_ID_EFF(frame);
            // At this point: all frames belong to openlcb protocols thus the
            // last 12 bits are the source alias.
            srcAddress_ = CanDefs::get_src(can_id);
            if (CanDefs::get_frame_type(can_id) == CanDefs::CONTROL_MSG)
            {
                // control frame
                forwardType_ = FORWARD_ALL;
                if (CanDefs::is_cid_frame(can_id))
                {
                    // We do not record source address of CHECK_ID frames,
                    // because they could be in conflict. We only record the ID
                    // at the reserve alias frame 200 msec later.
                    srcAddress_ = 0;
                }
                return;
            }
            // At this point: OpenLCB message.
            if (CanDefs::get_can_frame_type(can_id) == 6 ||
                CanDefs::get_can_frame_type(can_id) == 0)
            {
                // unknown can frame type
                forwardType_ = FORWARD_ALL;
                return;
            }
            // At this point: openlcb message with a known frame type (1, 2..5,
            // 7)
            if (CanDefs::get_can_frame_type(can_id) !=
                CanDefs::GLOBAL_ADDRESSED)
            {
                // Datagram and stream frames.
                forwardType_ = ADDRESSED;
                dstAddress_ = CanDefs::get_dst(can_id);
                return;
            }
            // At this point: global or addressed message
            Defs::MTI mti = static_cast<Defs::MTI>(CanDefs::get_mti(can_id));
            if (Defs::get_mti_address(mti) && frame.can_dlc >= 2)
            {
                // address present (really).
                dstAddress_ = frame.data[0] & 0xf;
                dstAddress_ <<= 8;
                dstAddress_ |= frame.data[1];
                forwardType_ = ADDRESSED;
                return;
            }
            bool has_event = false;
            if (Defs::get_mti_event(mti) && frame.can_dlc == 8)
            {
                event_ = data_to_eventid(frame.data);
                has_event = true;
            }
            if (mti == Defs::MTI_EVENT_REPORT && has_event)
            {
                forwardType_ = EVENT;
                return;
            }
            if (has_event)
            {
                switch (mti & ~Defs::MTI_MODIFIER_MASK)
                {
                    case Defs::MTI_CONSUMER_IDENTIFIED_VALID &
                        ~Defs::MTI_MODIFIER_MASK:
                        parent_->routingTable_.register_consumer(
                            message()->data()->skipMember_, event_);
                        break;
                    case Defs::MTI_PRODUCER_IDENTIFIED_VALID &
                        ~Defs::MTI_MODIFIER_MASK:
                        parent_->routingTable_.register_producer(
                            message()->data()->skipMember_, event_);
                        break;
                    default:
                        break;
                }
                switch (mti)
                {
                    case Defs::MTI_PRODUCER_IDENTIFIED_RANGE:
                        parent_->routingTable_.register_producer_range(
                            message()->data()->skipMember_, event_);
                        break;
                    case Defs::MTI_CONSUMER_IDENTIFIED_RANGE:
                        parent_->routingTable_.register_consumer_range(
                            message()->data()->skipMember_, event_);
                        break;
                    default:
                        break;
                }
            }
            // Now: we have a non-event global message or a message with an
            // invalid format.
            forwardType_ = FORWARD_ALL;
        }

        Action try_next_entry()
        {
            OSMutexLock l(&parent_->lock_);
            if (nextIt_ == parent_->ports_.end())
            {
                return done_processing();
            }

            if (forwardType_ == EVENT)
            {
                if (parent_->routingTable_.check_pcer(
                        static_cast<CanHubPortInterface *>(nextIt_->first),
                        event_))
                {
                    forward_to_port();
                }
            }
            else
            {
                // forward all
                forward_to_port();
            }

            nextIt_++;
            return again();
        }

        Action forward_addressed()
        {
            OSMutexLock l(&parent_->lock_);
            forward_to_port();
            return done_processing();
        }

        Action done_processing()
        {
            if (gcBuf_)
            {
                gcBuf_->unref();
                gcBuf_ = nullptr;
            }
            return release_and_exit();
        }

        void forward_to_port()
        {
            if (nextIt_->second.inactive_)
                return;
            if (nextIt_->second.canPort_)
            {
                if (nextIt_->second.canPort_ == message()->data()->skipMember_)
                    return;
                nextIt_->second.canPort_->send(message()->ref(), priority());
            }
            else
            {
                HASSERT(nextIt_->second.hubPort_);
                CanHubPortInterface *hpi =
                    reinterpret_cast<CanHubPortInterface *>(
                        nextIt_->second.hubPort_);
                if (hpi == message()->data()->skipMember_)
                    return;
                ensure_gc_buf_available();
                nextIt_->second.hubPort_->send(gcBuf_->ref());
            }
        }

        void ensure_gc_buf_available()
        {
            if (gcBuf_ != nullptr)
                return;
            mainBufferPool->alloc(&gcBuf_);
            char buf[29];
            char *end = gc_format_generate(&message()->data()->frame(), buf, 0);
            gcBuf_->data()->assign(buf, end - buf);
            gcBuf_->data()->skipMember_ = reinterpret_cast<HubPortInterface *>(
                message()->data()->skipMember_);
        }

        enum ForwardType
        {
            /// Broadcast packet that needs to go out to all ports,
            /// unfiltered.
            FORWARD_ALL,
            /// Addressed packet that needs to check the routing table.
            ADDRESSED,
            /// Event report packet that needs to check the routing table.
            EVENT
        };

        ForwardType forwardType_;   //< what to do with this frame
        NodeAlias srcAddress_;      //< for all OpenLCB frames
        NodeAlias dstAddress_;      //< for addressed frames
        EventId event_;             //< for PCER messages
        PortsMap::iterator nextIt_; //< which port to consider next
        GcCanRoutingHub *parent_;
        /// Gridconnect-rendered frame.
        Buffer<HubData> *gcBuf_;
    };

    DeliveryFlow deliveryFlow_;

    friend class DeliveryFlow;

    /// Data and objects we keep for each port.
    struct PortParser
    {
        /// If true, we must not send any data to this target, because it has
        /// been unregistered.
        bool inactive_{false};
        GcStreamParser segmenter_;
        CanHubPortInterface *canPort_{nullptr};
        HubPortInterface *hubPort_{nullptr};
    };
    /// Keyed by the skipMember_ value of the incoming data from a given port.
    std::map<void *, PortParser> ports_;
    OSMutex lock_;
    /** Due to race conditions involving iteration and add/remove calls, we
     * delay applying unregister requests until the next packet is being
     * sent. */
    std::vector<void *> pendingRemove_;

    RoutingLogic<CanHubPortInterface, NodeAlias> routingTable_;
};

} // namespace openlcb

#endif // _OPENLCB_CANROUTNGHUB_HXX_
