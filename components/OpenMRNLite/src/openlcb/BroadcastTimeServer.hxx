/** @copyright
 * Copyright (c) 2018, Stuart W. Baker
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
 * @file BroadcastTimeServer.hxx
 *
 * Implementation of a Broadcast Time Protocol Server.
 *
 * @author Stuart W. Baker
 * @date 4 November 2018
 */

#ifndef _OPENLCB_BROADCASTTIMESERVER_HXX_
#define _OPENLCB_BROADCASTTIMESERVER_HXX_

#include "openlcb/BroadcastTime.hxx"
#include "openlcb/BroadcastTimeAlarm.hxx"

namespace openlcb
{

class BroadcastTimeServerTime;
class BroadcastTimeServerSync;
class BroadcastTimeServerSet;
class BroadcastTimeServerAlarm;

/// Implementation of a Broadcast Time Protocol client.
class BroadcastTimeServer : public BroadcastTime
{
public:
    /// Constructor.
    /// @param node the virtual node that will be listening for events and
    ///             responding to Identify messages.
    /// @param clock_id 48-bit unique identifier for the clock instance
    BroadcastTimeServer(Node *node, NodeID clock_id);

    /// Destructor.
    ~BroadcastTimeServer();

#if defined(GTEST)
    void shutdown();

    bool is_shutdown();
#endif

private:
    /// Handle requested identification message.
    /// @param entry registry entry for the event range
    /// @param event information about the incoming message
    /// @param done used to notify we are finished
    void handle_identify_global(const EventRegistryEntry &entry,
                                EventReport *event,
                                BarrierNotifiable *done) override
    {
        AutoNotify an(done);
        if (event->dst_node && event->dst_node != node_)
        {
            // not for us
            return;
        }

        if (is_terminated())
        {
            start_flow(STATE(query_response));
        }

        event->event_write_helper<1>()->WriteAsync(
            node_, Defs::MTI_PRODUCER_IDENTIFIED_RANGE, WriteHelper::global(),
            eventid_to_buffer(EncodeRange(entry.event, 0x1 << 16)),
            done->new_child());

        // we can configure ourselves
        event->event_write_helper<2>()->WriteAsync(
            node_, Defs::MTI_CONSUMER_IDENTIFIED_RANGE, WriteHelper::global(),
            eventid_to_buffer(EncodeRange(entry.event + 0x8000, 0x1 << 15)),
            done->new_child());
    }

    /// Handle requested identification message.
    /// @param entry registry entry for the event range
    /// @param event information about the incoming message
    /// @param done used to notify we are finished
    void handle_identify_consumer(const EventRegistryEntry &entry,
                                  EventReport *event,
                                  BarrierNotifiable *done) override
    {
        AutoNotify an(done);

        if (event->event >= (eventBase_ + 0x8000))
        {
            // we can configure ourselves
            event->event_write_helper<1>()->WriteAsync(
                node_, Defs::MTI_CONSUMER_IDENTIFIED_RANGE,
                WriteHelper::global(),
                eventid_to_buffer(EncodeRange(entry.event + 0x8000, 0x1 << 15)),
                done->new_child());
        }
    }

    /// Handle requested identification message.
    /// @param entry registry entry for the event range
    /// @param event information about the incoming message
    /// @param done used to notify we are finished
    void handle_identify_producer(const EventRegistryEntry &entry,
                                  EventReport *event,
                                  BarrierNotifiable *done) override
    {
        AutoNotify an(done);
        event->event_write_helper<1>()->WriteAsync(
            node_, Defs::MTI_PRODUCER_IDENTIFIED_RANGE, WriteHelper::global(),
            eventid_to_buffer(EncodeRange(entry.event, 0x1 << 16)),
            done->new_child());
    }

    /// Handle an incoming consumer identified.
    /// @param entry registry entry for the event range
    /// @param event information about the incoming message
    /// @param done used to notify we are finished
    void handle_consumer_identified(const EventRegistryEntry &entry,
                                    EventReport *event,
                                    BarrierNotifiable *done) override;

    /// Handle an incoming event report.
    /// @param entry registry entry for the event range
    /// @param event information about the incoming message
    /// @param done used to notify we are finished
    void handle_event_report(const EventRegistryEntry &entry,
                             EventReport *event,
                             BarrierNotifiable *done) override;

    /// Try the possible set event shortcut.
    /// @param event event that we would be "setting"
    virtual void set_shortcut(uint64_t event) override;

    /// Entry to state machine.
    /// @return query_response after timeout
    Action entry()
    {
        return sleep_and_call(&timer_, MSEC_TO_NSEC(300),
                              STATE(query_response));
    }

    /// Respond to a query by scheduling a sync.
    /// @return exit()
    Action query_response();

    //BroadcastTimeAlarmDate alarmDate_; ///< date rollover alarm
    time_t secondsRequested_; ///< pending clock time in seconds
    uint16_t updateRequested_ : 1; ///< clock settings have change
#if defined(GTEST)
    uint16_t shutdown_ : 1;
#endif

    BroadcastTimeServerTime *time_;
    BroadcastTimeServerSync *sync_;
    BroadcastTimeServerSet *set_;
    BroadcastTimeServerAlarm *alarm_;

    friend class BroadcastTimeServerTime;
    friend class BroadcastTimeServerSync;
    friend class BroadcastTimeServerSet;
    friend class BroadcastTimeServerAlarm;


    DISALLOW_COPY_AND_ASSIGN(BroadcastTimeServer);
};

} // namespace openlcb

#endif // _OPENLCB_BROADCASTTIMESERVER_HXX_
