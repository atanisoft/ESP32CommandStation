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
 * @file BroadcastTimeServer.cxx
 *
 * Implementation of a Broadcast Time Protocol Server.
 *
 * @author Stuart W. Baker
 * @date 17 November 2018
 */

#include "openlcb/BroadcastTimeServer.hxx"

#include "executor/CallableFlow.hxx"

namespace openlcb
{

/// Request structure used to send requests to the
/// BroadcastTimeServerDateRolloverFinish object.
struct BroadcastTimeServerDateRolloverFinishInput
    : public CallableFlowRequestBase
{
    /// Setup "null" input.
    void reset()
    {
    }
};

/// State machine for sending the date rollover finish (year and date events)
/// sequence.
class BroadcastTimeServerDateRolloverFinish
    : public CallableFlow<BroadcastTimeServerDateRolloverFinishInput>
{
public:
    /// Constuctor.
    /// @param server reference to our parent
    BroadcastTimeServerDateRolloverFinish(BroadcastTimeServer *server)
        : CallableFlow<BroadcastTimeServerDateRolloverFinishInput>(
              server->node()->iface())
        , server_(server)
        , writer_()
        , timer_(this)
        , abortCnt_(0)
    {
    }

    /// Destructor.
    ~BroadcastTimeServerDateRolloverFinish()
    {
    }

    /// Request a date rollover finish.
    void request_finish()
    {
        // Because these requests are very timely, we use the abort count in
        // order to flush out any previously requested date report finishes.
        //
        // Because this can only be called from the same executor as the state
        // flow, we don't need an atomic lock.

        // terminate early so that we can invalidate the active request(s)

        if (abortCnt_ != UINT8_MAX)
        {
            ++abortCnt_;

            timer_.ensure_triggered();

            invoke_subflow_and_ignore_result(this);
        }
    }

private:
    /// Wait the obligatory 3 seconds before sending the year/date report.
    /// @return normally wait_and_call(STATE(send_year_report)), else
    ///         return_ok() on abort
    Action entry() override
    {
        // see if we need to flush any stale requests
        if (--abortCnt_ != 0)
        {
            return return_ok();
        }

        return sleep_and_call(&timer_, SEC_TO_NSEC(3), STATE(send_year_report));
    }

    /// Send the Event Report message appropriate for the year event ID.
    /// @return normally wait_and_call(STATE(send_date_report)), else
    ///         return_ok() on trigger (abort)
    Action send_year_report()
    {
        if (timer_.is_triggered())
        {
            // abort
            return return_ok();
        }

        const struct tm *tm = server_->gmtime_recalculate();

        int year = tm->tm_year + 1900;
        if (year < 0)
        {
            year = 0;
        }
        if (year > 4095)
        {
            year = 4095;
        }

        uint64_t event_id =
            BroadcastTimeDefs::year_to_event(server_->event_base(), year);

        writer_.WriteAsync(server_->node(), Defs::MTI_EVENT_REPORT,
            WriteHelper::global(), eventid_to_buffer(event_id), this);

        return wait_and_call(STATE(send_date_report));
    }

    /// Send the Event Report message appropriate for the date event ID.
    /// @return wait_and_return_ok()
    Action send_date_report()
    {
        const struct tm *tm = server_->gmtime_get();

        uint64_t event_id = BroadcastTimeDefs::date_to_event(
            server_->event_base(), tm->tm_mon + 1, tm->tm_mday);

        writer_.WriteAsync(server_->node(), Defs::MTI_EVENT_REPORT,
            WriteHelper::global(), eventid_to_buffer(event_id), this);

        return wait_and_return_ok();
    }

    BroadcastTimeServer *server_; ///< reference to our parent
    WriteHelper writer_; ///< helper for sending event messages
    StateFlowTimer timer_; ///< timer helper
    uint8_t abortCnt_; ///< request that the current processing be aborted

    DISALLOW_COPY_AND_ASSIGN(BroadcastTimeServerDateRolloverFinish);
};


/// Request structure used to send requests to the BroadcastTimeServerTime
/// object.
struct BroadcastTimeServerTimeInput : public CallableFlowRequestBase
{
    /// Setup "null" input.
    void reset()
    {
    }
};

/// State machine for sending the clock time events.
class BroadcastTimeServerTime
    : public CallableFlow<BroadcastTimeServerTimeInput>
{
public:
    /// Constuctor.
    /// @param server reference to our parent
    BroadcastTimeServerTime(BroadcastTimeServer *server)
        : CallableFlow<BroadcastTimeServerTimeInput>(server->node()->iface())
        , finish_(server)
        , server_(server)
        , writer_()
        , timeRequired_(false)
        , dateRollover_(false)
    {
    }

    /// Destructor.
    ~BroadcastTimeServerTime()
    {
    }

    /// Request a time event.
    void request_time()
    {
        if (!timeRequired_)
        {
            // no time pending, make it pending
            timeRequired_ = true;
            invoke_subflow_and_ignore_result(this);
        }
    }

private:
    /// Send the date rollover event if appropriate.
    /// @return if running send_time_report(), else return_ok()
    Action entry() override
    {
        if (!server_->is_running())
        {
            // we shouldn't get here, but it means the clock is not running
            return return_ok();
        }

        const struct tm *tm = server_->gmtime_recalculate();

        if ((server_->get_rate_quarters() > 0 && tm->tm_hour ==  0 &&
             tm->tm_min ==  0) ||
            (server_->get_rate_quarters() < 0 && tm->tm_hour == 23 &&
             tm->tm_min == 59))
        {
            // date rollover
            dateRollover_ = true;

            uint64_t event_id = server_->event_base();
            event_id += BroadcastTimeDefs::DATE_ROLLOVER_EVENT_SUFFIX;

            writer_.WriteAsync(server_->node(), Defs::MTI_EVENT_REPORT,
                WriteHelper::global(), eventid_to_buffer(event_id), this);

            return wait_and_call(STATE(send_time_report));
        }
        else
        {
            return call_immediately(STATE(send_time_report));
        }
    }

    /// Send the appropriate time report event.
    /// @return wait_and_return_ok()
    Action send_time_report()
    {
        timeRequired_ = false;
        const struct tm *tm = server_->gmtime_get();

        uint64_t event_id = BroadcastTimeDefs::time_to_event(
            server_->event_base(), tm->tm_hour, tm->tm_min);
        writer_.WriteAsync(server_->node(), Defs::MTI_EVENT_REPORT,
            WriteHelper::global(), eventid_to_buffer(event_id), this);

        if (dateRollover_)
        {
            dateRollover_ = false;
            finish_.request_finish();
        }

        return wait_and_return_ok();        
    }

    BroadcastTimeServerDateRolloverFinish finish_; ///< finsh the date rollover
    BroadcastTimeServer *server_; ///< reference to our parent
    WriteHelper writer_; ///< helper for sending event messages
    uint8_t timeRequired_ : 1; ///< flag to keep track of multiple tme requests
    uint8_t dateRollover_ : 1; ///< processing a date rollover

    DISALLOW_COPY_AND_ASSIGN(BroadcastTimeServerTime);
};

/// Request structure used to send requests to the BroadcastTimeServerSync
/// object.
struct BroadcastTimeServerSyncInput : public CallableFlowRequestBase
{
    /// Setup "null" input.
    void reset()
    {
    }
};

/// State machine for sending the clock sync sequence.
class BroadcastTimeServerSync
    : public CallableFlow<BroadcastTimeServerSyncInput>
{
public:
    /// Constuctor.
    /// @param server reference to our parent
    BroadcastTimeServerSync(BroadcastTimeServer *server)
        : CallableFlow<BroadcastTimeServerSyncInput>(server->node()->iface())
        , server_(server)
        , writer_()
        , timer_(this)
        , syncRequired_(false)
#if defined(GTEST)
        , shutdown_(false)
#endif
    {
    }

    /// Destructor.
    ~BroadcastTimeServerSync()
    {
    }

    /// Request a sync.
    void request_sync()
    {
        if (!syncRequired_)
        {
            // no sync pending, make it pending
            syncRequired_ = true;

            // abort the current sync if sleeping
            timer_.ensure_triggered();

            invoke_subflow_and_ignore_result(this);
        }
    }

#if defined(GTEST)
    void shutdown()
    {
        shutdown_ = true;
        request_sync();
    }

    bool is_shutdown()
    {
        return is_terminated();
    }
#endif

private:
    /// Send the Producer Identified message appropriate for the start/stop
    /// event ID.
    /// @return wait_and_call(STATE(send_rate_report))
    Action entry() override
    {
#if defined(GTEST)
        if (shutdown_)
        {
            return StateFlowBase::exit();
        }
#endif
        server_->gmtime_recalculate();

        uint64_t event_id = server_->event_base();
        event_id += server_->is_started() ?
            BroadcastTimeDefs::START_EVENT_SUFFIX :
            BroadcastTimeDefs::STOP_EVENT_SUFFIX;

        syncRequired_ = false;
        writer_.WriteAsync(server_->node(), Defs::MTI_PRODUCER_IDENTIFIED_VALID,
            WriteHelper::global(), eventid_to_buffer(event_id), this);

        return wait_and_call(STATE(send_rate_report));
    }

    /// Send the Producer Identified message appropriate for the rate event ID.
    /// @return wait_and_call(STATE(send_year_report))
    Action send_rate_report()
    {
        uint64_t event_id = BroadcastTimeDefs::rate_to_event(
            server_->event_base(), server_->get_rate_quarters());

        writer_.WriteAsync(server_->node(), Defs::MTI_PRODUCER_IDENTIFIED_VALID,
            WriteHelper::global(), eventid_to_buffer(event_id), this);

        return wait_and_call(STATE(send_year_report));
    }

    /// Send the Producer Identified message appropriate for the year event ID.
    /// @return wait_and_call(STATE(send_date_report))
    Action send_year_report()
    {
        const struct tm *tm = server_->gmtime_get();

        int year = tm->tm_year + 1900;
        if (year < 0)
        {
            year = 0;
        }
        if (year > 4095)
        {
            year = 4095;
        }

        uint64_t event_id =
            BroadcastTimeDefs::year_to_event(server_->event_base(), year);

        writer_.WriteAsync(server_->node(), Defs::MTI_PRODUCER_IDENTIFIED_VALID,
            WriteHelper::global(), eventid_to_buffer(event_id), this);

        return wait_and_call(STATE(send_date_report));
    }

    /// Send the Producer Identified message appropriate for the date event ID.
    /// @return wait_and_call(STATE(send_time_report))
    Action send_date_report()
    {
        const struct tm *tm = server_->gmtime_get();

        uint64_t event_id = BroadcastTimeDefs::date_to_event(
            server_->event_base(), tm->tm_mon + 1, tm->tm_mday);

        writer_.WriteAsync(server_->node(), Defs::MTI_PRODUCER_IDENTIFIED_VALID,
            WriteHelper::global(), eventid_to_buffer(event_id), this);

        return wait_and_call(STATE(send_time_report));
    }

    /// Send the Producer Identified message appropriate for the time event ID.
    /// @return send_time_report_done()
    Action send_time_report()
    {
        const struct tm *tm = server_->gmtime_recalculate();

        uint64_t event_id = BroadcastTimeDefs::time_to_event(
            server_->event_base(), tm->tm_hour, tm->tm_min);
        writer_.WriteAsync(server_->node(), Defs::MTI_PRODUCER_IDENTIFIED_VALID,
            WriteHelper::global(), eventid_to_buffer(event_id), this);

        return wait_and_call(STATE(send_time_report_done));
    }

    /// If clock is running, sleep until next time report.
    /// @return send_time_report_next() if running, return_ok()
    Action send_time_report_done()
    {
        if (server_->is_running())
        {
            const struct tm *tm = server_->gmtime_get();
            //const struct tm *tm = server_->gmtime_recalculate();

            // setup to send the next time report
            time_t expires = server_->time();
            if (server_->get_rate_quarters() > 0)
            {
                expires += 60 - tm->tm_sec;
            }
            else
            {
                // Note, we are adding 2 in order to account for some jitter
                // in the RTOS timing. The result is that the next time report
                // event might be sent at tm_sec == 57 to 59.
                expires -= tm->tm_sec ? (tm->tm_sec + 2) : 2;
            }

            long long real_expires = 0;
            bool result =
                server_->real_nsec_until_fast_time_abs(expires, &real_expires);
            HASSERT(result);
            return sleep_and_call(&timer_, real_expires,
                                  STATE(send_time_report_next));
        }
        return return_ok();
    }

    /// Send the Event Report message appropriate for the time event ID.
    /// @return return_ok() if triggered early, else wait_and_return_ok()
    Action send_time_report_next()
    {
        if (!timer_.is_triggered())
        {
            server_->time_->request_time();
        }

        return return_ok();
    }

    BroadcastTimeServer *server_; ///< reference to our parent
    WriteHelper writer_; ///< helper for sending event messages
    StateFlowTimer timer_; ///< timer helper
    uint8_t syncRequired_ : 1; ///< flag to keep track of multiple sync requests
#if defined(GTEST)
    uint8_t shutdown_ : 1;
#endif

    DISALLOW_COPY_AND_ASSIGN(BroadcastTimeServerSync);
};

/// Request structure used to send requests to the BroadcastTimeServerSet
/// object.
struct BroadcastTimeServerSetInput : public CallableFlowRequestBase
{
    /// Setup "null" input.
    /// @param suffix the clock set event suffix
    void reset(uint16_t suffix)
    {
        suffix_ = suffix;
    }
    
    uint16_t suffix_; ///< clock set event suffix
};

/// State machine for sending the clock set sequence.
class BroadcastTimeServerSet
    : public CallableFlow<BroadcastTimeServerSetInput>
{
public:
    /// Constuctor.
    /// @param server reference to our parent
    BroadcastTimeServerSet(BroadcastTimeServer *server)
        : CallableFlow<BroadcastTimeServerSetInput>(
              server->node()->iface())
        , server_(server)
        , writer_()
        , timer_(this)
        , requestCount_(0)
    {
    }

    /// Destructor.
    ~BroadcastTimeServerSet()
    {
    }

    /// Request a time parameter set.
    /// @param suffix the clock set event suffix
    void request_set(uint16_t suffix)
    {
        if (requestCount_ != UINT8_MAX)
        {
            ++requestCount_;

            // waiting for a sync sequence to start, abort
            timer_.ensure_triggered();

            invoke_subflow_and_ignore_result(this, suffix);
        }
    }

private:
    /// Set a clock attribute and send the appropriate event report.
    /// @return write_done()
    Action entry() override
    {
        bool start_or_stop = false;
        struct tm tm;
        server_->gmtime_r(&tm);

        uint16_t suffix = message()->data()->suffix_;

        switch(BroadcastTimeDefs::get_event_type(suffix))
        {
            case BroadcastTimeDefs::START:
                if (!server_->started_)
                server_->started_ = true;
                start_or_stop = true;
                break;
            case BroadcastTimeDefs::STOP:
                server_->started_ = false;
                start_or_stop = true;
                break;
            case BroadcastTimeDefs::SET_TIME:
            {
                tm.tm_hour = BroadcastTimeDefs::event_to_hour(suffix);
                tm.tm_min = BroadcastTimeDefs::event_to_min(suffix);
                break;
            }
            case BroadcastTimeDefs::SET_DATE:
            {
                tm.tm_mday = BroadcastTimeDefs::event_to_day(suffix);
                tm.tm_mon = BroadcastTimeDefs::event_to_month(suffix) - 1;
                break;
            }
            case BroadcastTimeDefs::SET_YEAR:
            {
                tm.tm_year = BroadcastTimeDefs::event_to_year(suffix) - 1900;
                break;
            }
            case BroadcastTimeDefs::SET_RATE:
            {
                    server_->rate_ = BroadcastTimeDefs::event_to_rate(suffix);
                break;
            }
            default:
                // should never get there
                --requestCount_;
                return return_ok();
        }

        {
            AtomicHolder h(server_);
            server_->seconds_ = mktime(&tm);
            server_->timestamp_ = OSTime::get_monotonic();
        }

        server_->service_callbacks();

        if (start_or_stop)
        {
            // start or stop events do not produce events of their own
            return call_immediately(STATE(write_done));
        }

        uint64_t event_id = server_->event_base() + suffix - 0x8000;

        writer_.WriteAsync(server_->node(), Defs::MTI_EVENT_REPORT,
            WriteHelper::global(), eventid_to_buffer(event_id), this);

        return wait_and_call(STATE(write_done));
    }

    /// The previous event write has completed.
    /// @return next state send_sync after 3 second delay if no more pening
    ///         requests, else if requests pending, return_ok()
    Action write_done()
    {
        if (--requestCount_ == 0)
        {
            return sleep_and_call(&timer_, SEC_TO_NSEC(3), STATE(send_sync));
        }
        else
        {
            return return_ok();
        }
    }

    /// Request the sync sequence if timer has not been triggered early.
    /// @return return_ok()
    Action send_sync()
    {
        if (!timer_.is_triggered())
        {
            // we have not received any more set events for 3 seconds, sync
            server_->sync_->request_sync();
        }
        return return_ok();
    }

    BroadcastTimeServer *server_; ///< reference to our parent
    WriteHelper writer_; ///< helper for sending event messages
    StateFlowTimer timer_; ///< timer helper
    uint8_t requestCount_; ///< counter to know when there are no more requests

    DISALLOW_COPY_AND_ASSIGN(BroadcastTimeServerSet);
};

/// Specialization of the BroacastTimeAlarm to expire on the necessary clock
/// minutes that must be produced.
class BroadcastTimeServerAlarm : public BroadcastTimeAlarm
{
public:
   /// Constructor.
    /// @param clock clock that our alarm is based off of
    BroadcastTimeServerAlarm(BroadcastTimeServer *server)
        : BroadcastTimeAlarm(
              server->node(), server,
              std::bind(&BroadcastTimeServerAlarm::expired_callback, this))
        , server_(server)
    {
        memset(activeMinutes_, 0, sizeof(activeMinutes_));
    }

    /// Destructor.
    ~BroadcastTimeServerAlarm()
    {
    }

    /// Add a time event subscriber.
    /// @param hour hour to subscribe to
    /// @param min minute to subscribe to
    void subscribe(int hour, int min)
    {
        if (hour <= 23 && hour >= 0 && min <= 59 && min >= 0)
        {
            AtomicHolder h(this);
            if ((activeMinutes_[hour] & (0x1ULL << min)) == 0)
            {
                activeMinutes_[hour] |= 0x1ULL << min;
                update_notify();
            }
        }
    }

private:
    /// Entry point of the state machine.
    /// @return BroadcastTimeAlarm::entry();
    Action entry() override
    {
        update_notify();

        return BroadcastTimeAlarm::entry();
    }

    /// callback for when the alarm expires.
    void expired_callback()
    {
        server_->time_->request_time();
        update_notify();
    }

    /// Called when the clock time has changed.
    void update_notify() override
    {
        if (clock_->is_running())
        {
            set(next_active_minute(clock_->time(),
                                   clock_->gmtime_recalculate()));
            BroadcastTimeAlarm::update_notify();
        }
    }

    /// Get the next active minute that we will produce a time event on.
    /// @param seconds current time in seconds
    /// @param tm current time in struct tm format
    /// @return the next time in rate seconds that we will expire
    time_t next_active_minute(time_t seconds, const struct tm *tm)
    {
        int hour = tm->tm_hour;
        int min = tm->tm_min;

        // we will target to produce a time event every four real minutes.
        int rate_min_per_4_real_min = std::abs(clock_->get_rate_quarters());

        // get the time_t value for the next whole minute
        if (clock_->get_rate_quarters() > 0)
        {
            seconds += 60 - tm->tm_sec;
        }
        else
        {
             seconds -= tm->tm_sec ? tm->tm_sec : 60;
        }

        do
        {
            if (next_minute(&hour, &min))
            {
                // date rollover, always produce the date rollover event
                break;
            }
            if (activeMinutes_[hour] & (0x1ULL << min))
            {
                break;
            }
            seconds += clock_->get_rate_quarters() > 0 ? 60 : -60;
        }
        while ((hour != tm->tm_hour || min != tm->tm_min) &&
               --rate_min_per_4_real_min > 0);

        return seconds;
    }

    /// Advance to the next hour/min based on rate setting.
    /// @param hour input is current hour, output is hour for the next minute
    /// @paran min input is the current minute, output is the next minute
    /// @return true of we have a date rollover, else false
    bool next_minute(int *hour, int *min)
    {
        bool result = false;
        if (clock_->get_rate_quarters() > 0)
        {
            if (++(*min) == 60)
            {
                *min = 0;
                if (++(*hour) == 24)
                {
                    *hour = 0;
                    result = true;
                }
            }
        }
        else if (clock_->get_rate_quarters() < 0)
        {
            if (--(*min) < 0)
            {
                *min = 59;
                if (--(*hour) < 0)
                {
                    *hour = 23;
                    result = true;
                }
            }
        }
        return result;
    }

    BroadcastTimeServer *server_; ///< reference to our parent
    uint64_t activeMinutes_[24]; ///< active minutes to produce events on

    DISALLOW_COPY_AND_ASSIGN(BroadcastTimeServerAlarm);
};



//
// BroadcastTimeServer::BroadcastTimeServer()
//
BroadcastTimeServer::BroadcastTimeServer(Node *node, NodeID clock_id)
    : BroadcastTime(node, clock_id)
    , secondsRequested_(0)
    , updateRequested_(false)
#if defined (GTEST)
    , shutdown_(false)
#endif
    , time_(new BroadcastTimeServerTime(this))
    , sync_(new BroadcastTimeServerSync(this))
    , set_(new BroadcastTimeServerSet(this))
    , alarm_(new BroadcastTimeServerAlarm(this))
{
    EventRegistry::instance()->register_handler(
        EventRegistryEntry(this, eventBase_), 16);
}

//
// BroadcastTimeServer::~BroadcastTimeServer()
//
BroadcastTimeServer::~BroadcastTimeServer()
{
    EventRegistry::instance()->unregister_handler(this);
    delete alarm_;
    delete set_;
    delete sync_;
    delete time_;
}

#if defined(GTEST)
void BroadcastTimeServer::shutdown()
{
    shutdown_ = true;
    sync_->shutdown();
    alarm_->shutdown();
}

bool BroadcastTimeServer::is_shutdown()
{
    return is_terminated() && alarm_->is_shutdown() && sync_->is_shutdown();
}
#endif

//
// BroadcastTimeServer::handle_consumer_identified()
//
void BroadcastTimeServer::handle_consumer_identified(
    const EventRegistryEntry &entry, EventReport *event,
    BarrierNotifiable *done)
{
    done->notify();

    if (BroadcastTimeDefs::get_event_type(event->event) ==
        BroadcastTimeDefs::REPORT_TIME)
    {
        int min = BroadcastTimeDefs::event_to_min(event->event);
        int hour = BroadcastTimeDefs::event_to_hour(event->event);
        if (hour != -1 && min != -1)
        {
            alarm_->subscribe(hour, min);
        }
    }
}

//
// BroadcastTimeServer::handle_event_report()
//
void BroadcastTimeServer::handle_event_report(const EventRegistryEntry &entry,
                                              EventReport *event,
                                              BarrierNotifiable *done)
{
    AutoNotify an(done);

    set_shortcut(event->event);
}

//
// BroadcastTimeServer::set_shortcut
//
void BroadcastTimeServer::set_shortcut(uint64_t event)
{
    switch(BroadcastTimeDefs::get_event_type(event))
    {
        case BroadcastTimeDefs::SET_TIME:
        case BroadcastTimeDefs::SET_DATE:
        case BroadcastTimeDefs::SET_YEAR:
        case BroadcastTimeDefs::SET_RATE:
        case BroadcastTimeDefs::START:
        case BroadcastTimeDefs::STOP:
            set_->request_set(event);
            break;
        case BroadcastTimeDefs::QUERY:
            if (is_terminated())
            {
                start_flow(STATE(entry));
            }
            break;
        default:
            break;
    }
}

//
// BroadcastTimeServer::query_response()
//
StateFlowBase::Action BroadcastTimeServer::query_response()
{
    sync_->request_sync();
    return exit();
}

} // namespace openlcb
