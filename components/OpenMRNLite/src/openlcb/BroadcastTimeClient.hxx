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
 * @file BroadcastTimeClient.hxx
 *
 * Implementation of a Broadcast Time Protocol client.
 *
 * @author Stuart W. Baker
 * @date 14 October 2018
 */

#ifndef _OPENLCB_BROADCASTTIMECLIENT_HXX_
#define _OPENLCB_BROADCASTTIMECLIENT_HXX_

#include "openlcb/BroadcastTime.hxx"
#include "openlcb/BroadcastTimeAlarm.hxx"

namespace openlcb
{

/// Implementation of a Broadcast Time Protocol client.
class BroadcastTimeClient : public BroadcastTime
{
public:
    /// Constructor.
    /// @param node the virtual node that will be listening for events and
    ///             responding to Identify messages.
    /// @param clock_id 48-bit unique identifier for the clock instance
    /// @param configure_agent if true, can configure clock server, if false,
    ///                        ignores all set_time/data/etc calls.
    BroadcastTimeClient(Node *node, NodeID clock_id,
                        bool configure_agent = false)
        : BroadcastTime(node, clock_id)
        , configureAgent_(configure_agent)
        , immediateUpdate_(false)
        , immediatePending_(false)
        , sleeping_(false)
        , waiting_(false)
        , rolloverPending_(false)
        , rolloverPendingDate_(false)
        , rolloverPendingYear_(false)
    {
        EventRegistry::instance()->register_handler(
            EventRegistryEntry(this, eventBase_), 16);
    }

    /// Destructor.
    ~BroadcastTimeClient()
    {
        EventRegistry::instance()->unregister_handler(this);
    }

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
            start_flow(STATE(initialize));
        }

        event->event_write_helper<1>()->WriteAsync(
            node_, Defs::MTI_CONSUMER_IDENTIFIED_RANGE, WriteHelper::global(),
            eventid_to_buffer(EncodeRange(eventBase_, 0x1 << 16)),
            done->new_child());
        if (configureAgent_)
        {
            // we can configure our complementary time server
            event->event_write_helper<2>()->WriteAsync(
                node_, Defs::MTI_PRODUCER_IDENTIFIED_RANGE,
                WriteHelper::global(),
                eventid_to_buffer(
                    EncodeRange(entry.event |
                                    BroadcastTimeDefs::EVENT_SET_SUFFIX_MASK,
                                0x1 << 15)),
                done->new_child());
        }
        else
        {
            // we cannot configure our complementary time server
            event->event_write_helper<2>()->WriteAsync(
                node_, Defs::MTI_PRODUCER_IDENTIFIED_UNKNOWN,
                WriteHelper::global(),
                eventid_to_buffer(entry.event |
                                  BroadcastTimeDefs::QUERY_EVENT_SUFFIX),
                done->new_child());
        }
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
        if (event->event < (eventBase_ | 0x5000))
        {
            event->event_write_helper<1>()->WriteAsync(
                node_, Defs::MTI_CONSUMER_IDENTIFIED_UNKNOWN,
                WriteHelper::global(), eventid_to_buffer(event->event),
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
        if ((configureAgent_ &&
             (entry.event & BroadcastTimeDefs::EVENT_SET_SUFFIX_MASK)) ||
            event->event == (entry.event |
                             BroadcastTimeDefs::QUERY_EVENT_SUFFIX))
        {
            // if we cannot configure our complementary time server,
            // we only produce one event.
            event->event_write_helper<1>()->WriteAsync(
                node_, Defs::MTI_PRODUCER_IDENTIFIED_UNKNOWN,
                WriteHelper::global(),
                eventid_to_buffer(entry.event |
                                  BroadcastTimeDefs::QUERY_EVENT_SUFFIX),
                done->new_child());
        }
    }

    /// Handle an incoming producer identified.
    /// @param entry registry entry for the event range
    /// @param event information about the incoming message
    /// @param done used to notify we are finished
    void handle_producer_identified(const EventRegistryEntry &entry,
                                    EventReport *event,
                                    BarrierNotifiable *done) override
    {
        done->notify();

        if (event->state == EventState::VALID)
        {
            // We only care about valid event state.
            // Producer identified only happens when a clock synchronization
            // is taking place.  This voids previous date rollover events.
            rolloverPending_ = false;
            handle_updates(event, false);
        }

    }

    /// Handle an incoming event report.
    /// @param entry registry entry for the event range
    /// @param event information about the incoming message
    /// @param done used to notify we are finished
    void handle_event_report(const EventRegistryEntry &entry,
                             EventReport *event,
                             BarrierNotifiable *done) override
    {
        done->notify();

        handle_updates(event, true);
    }

private:
    /// Handle an incoming time update.
    /// @param entry registry entry for the event range
    /// @report true of this an event report, false if a Producer Identified
    void handle_updates(EventReport *event, bool report);

    /// Perform a bit of logic that is required whenever the clock's running
    /// state is changed.
    /// @param started true if the clock is started, else false
    void start_stop_logic(bool started)
    {
        bool notify = false;
        {
            AtomicHolder h(this);
            if (started_ != started)
            {
                if (!started)
                {
                    seconds_ = time();
                }
                timestamp_ = OSTime::get_monotonic();
                if (!started)
                {
                    ::gmtime_r(&seconds_, &tm_);
                }
                started_ = started;
                notify = true;
            }
            // release AtomicHolder
        }
        if (notify)
        {
            service_callbacks();
        }
    }

    /// Initialize client by sending a time query.
    /// @return next state is initialize_done
    Action initialize()
    {
        rolloverPending_ = false;
        rolloverPendingDate_ = false;
        rolloverPendingYear_ = false;

        writer_.WriteAsync(node_, Defs::MTI_EVENT_REPORT, WriteHelper::global(),
            eventid_to_buffer(eventBase_ |
                              BroadcastTimeDefs::QUERY_EVENT_SUFFIX),
            this);
        return wait_and_call(STATE(initialize_done));
    }

    /// Initialize finished
    /// @return next state client_update.
    Action initialize_done()
    {
        waiting();
        return wait_and_call(STATE(client_update));
    }

    /// Notification arrived that we should update our state.
    /// @return next state client_update_commit if an update is to be made,
    ///         else next state client_update_wait if we are expecting more
    ///         incoming clock set events or producer identifieds
    Action client_update()
    {
        if (immediateUpdate_ || rolloverPending_)
        {
            return call_immediately(STATE(client_update_commit));
        }
        else
        {
            // because the set time sometimes comes in bursts, we will wait
            // a little while for them to finish coming in.  This avoids
            // extra redundant processing.
            sleeping();
            return sleep_and_call(&timer_, MSEC_TO_NSEC(100),
                                  STATE(client_update_maybe));
        }
    }

    /// Test if we are ready to update the client, or if we may want to wait
    /// for more messages (in the form of a burst) to come in first.
    /// @return next state client_updeate_commit if the burst is interrupted
    ///         else next state client_update if the burtst is still in
    ///         progress
    Action client_update_maybe()
    {
        if (timer_.is_triggered())
        {
            return call_immediately(STATE(client_update));
        }
        else
        {
            sleeping_ = false;
            return call_immediately(STATE(client_update_commit));
        }
    }

    /// commit the client update.
    /// @return next state rollover_pending if a date rollover is in progress,
    ///         else next state client_update
    Action client_update_commit()
    {
        if (rolloverPendingDate_ || rolloverPendingYear_)
        {
            // initiate timeout, in case the time server doesn't complete the
            // sequence successfully.
            sleeping();
            return sleep_and_call(&timer_, SEC_TO_NSEC(6),
                                  STATE(rollover_pending));
        }

        /// @todo As of 14 October 2018, newlib for armgcc uses 32-bit time_t.
        /// It seems that 64-bit time_t may become the default soon.

        {
            bool notify = false;
            {
                AtomicHolder h(this);
                time_t old_seconds;
                if (rate_ != rateRequested_ ||
                    (immediateUpdate_ && immediatePending_))
                {
                    // rate changed or an immediate update was pending.
                    notify = true;
                }
                else
                {
                    // we will need to check for jitter later
                    old_seconds = time();
                }

                // we are about to commit an updated time, reset the flags
                immediateUpdate_ = false;
                immediatePending_ = false;

                rate_ = rateRequested_;
                seconds_ = ::mktime(&tm_);
                timestamp_ = OSTime::get_monotonic();
                if (rolloverPending_)
                {
                    // roll forward/back the 3 second delay for the year/date
                    // report
                    seconds_ += rate_ >= 0 ? 3 : -3;
                    rolloverPending_ = false;
                }

                if (notify == false)
                {
                    if (std::abs(compare_realtime(seconds_, old_seconds)) > 3)
                    {
                        // We hadd a drift of more than 3 real seconds.  3 real
                        // seconds is chosen because it is within the magnitude
                        // of normal network jitter.
                        notify = true;
                    }
                }
                // release AtomicHolder
            }
            if (notify)
            {
                service_callbacks();
            }
        }

        waiting();
        return wait_and_call(STATE(client_update));
    }

    /// Wait on the completion of a date rollover sequence.
    /// @return next state client_update_commit if timer triggered, else
    ///         next state initialize
    Action rollover_pending()
    {
        if (timer_.is_triggered())
        {
            return call_immediately(STATE(client_update_commit));
        }
        else
        {
            sleeping_ = false;
            // A date rollover sequence failed.  Query the clock in order to
            // resync.  In the meantime, continue using our internal time.
            return call_immediately(STATE(initialize));
        }
    }

    /// Setup the state flags to reflect that we are in a wait_and_call().
    void waiting()
    {
        waiting_ = true;
        sleeping_ = false;
    }

    /// Setup the state flags to reflect that we are in a sleep_and_call().
    void sleeping()
    {
        waiting_ = false;
        sleeping_ = true;
    }

    /// wakeup the state machine.
    void wakeup()
    {
        if (waiting_)
        {
            notify();
            waiting_ = false;
        }
        if (sleeping_)
        {
            timer_.trigger();
            sleeping_ = false;
        }
    }

    uint16_t configureAgent_   : 1; ///< instance can be used to configure clock
    uint16_t immediateUpdate_  : 1; ///< true if the update should be immediate
    uint16_t immediatePending_ : 1; /// future immediate upate expected
    uint16_t sleeping_         : 1; ///< true if stateflow is waiting on timer
    uint16_t waiting_          : 1; ///< true if stateflow is waiting
    uint16_t rolloverPending_  : 1; ///< a day rollover is about to occur
    uint16_t rolloverPendingDate_ : 1; ///< a day rollover is about to occur
    uint16_t rolloverPendingYear_ : 1; ///< a day rollover is about to occur


    DISALLOW_COPY_AND_ASSIGN(BroadcastTimeClient);
};

} // namespace openlcb

#endif // _OPENLCB_BROADCASTTIMECLIENT_HXX_

