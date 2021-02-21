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
 * @file BroadcastTime.hxx
 *
 * Implementation of Broadcast Time Protocol.
 *
 * @author Stuart W. Baker
 * @date 4 November 2018
 */

#ifndef _OPENLCB_BROADCASTTIME_HXX_
#define _OPENLCB_BROADCASTTIME_HXX_

#include <functional>

#include "openlcb/BroadcastTimeDefs.hxx"
#include "openlcb/EventHandlerTemplates.hxx"

namespace openlcb
{

/// Implementation of Broadcast Time Protocol.
class BroadcastTime : public SimpleEventHandler
                    , public StateFlowBase
                    , protected Atomic
{
public:
    typedef std::vector<std::function<void()>>::size_type UpdateSubscribeHandle;

    /// Set the time in seconds since the system Epoch. The new time does not
    /// become valid until the update callbacks are called.
    /// @param hour hour (0 to 23)
    /// @param minutes minutes (0 to 59)
    void set_time(int hours, int minutes)
    {
        new SetFlow(this, SetFlow::Command::SET_TIME, hours, minutes);
    }

    /// Set the time in seconds since the system Epoch. The new date does not
    /// become valid until the update callbacks are called.
    /// @param month month (1 to 12)
    /// @param day day of month (1 to 31)
    void set_date(int month, int day)
    {
        new SetFlow(this, SetFlow::Command::SET_DATE, month, day);
    }

    /// Set the time in seconds since the system Epoch. The new year does not
    /// become valid until the update callbacks are called.
    /// @param year (0AD to 4095AD)
    void set_year(int year)
    {
        new SetFlow(this, SetFlow::Command::SET_YEAR, year);
    }

    /// Set Rate. The new rate does not become valid until the update callbacks
    /// are called.
    /// @param rate clock rate ratio as 12 bit sign extended fixed point
    ///             rrrrrrrrrr.rr
    void set_rate_quarters(int16_t rate)
    {
        new SetFlow(this, SetFlow::Command::SET_RATE, rate);
    }

    /// Start clock
    void start()
    {
        new SetFlow(this, SetFlow::Command::START);
    }

    /// Stop clock
    void stop()
    {
        new SetFlow(this, SetFlow::Command::STOP);
    }

    /// Get the time as a value of seconds relative to the system epoch.  At the
    /// same time get an atomic matching pair of the rate
    /// @return pair<time in seconds relative to the system epoch, rate>
    std::pair<time_t, int16_t> time_and_rate_quarters()
    {
        AtomicHolder h(this);
        return std::make_pair(time(), rate_);
    }

    /// Get the difference in time scaled to real time.
    /// @param t1 time 1 to compare
    /// @param t2 time 2 to compare
    /// @return (t1 - t2) scaled to real time.
    time_t compare_realtime(time_t t1, time_t t2)
    {
        int rate = rate_;
        if (rate == 0)
        {
            // avoid divid by zero error
            rate = 4;
        }
        return ((t1 - t2) * 4) / rate;
    }

    /// Get the time as a value of seconds relative to the system epoch.
    /// @return time in seconds relative to the system epoch
    time_t time()
    {
        AtomicHolder h(this);
        if (started_)
        {
            long long elapsed = OSTime::get_monotonic() - timestamp_;
            elapsed = ((elapsed * std::abs(rate_)) + 2) / 4;

            time_t diff = (time_t)NSEC_TO_SEC_ROUNDED(elapsed);
            return (rate_ < 0) ? seconds_ - diff : seconds_ + diff;
        }
        else
        {
            // clock is stopped, time is not progressing
            return seconds_;
        }
    }

    /// Get the time as a standard struct tm.
    /// @param result a pointer to the structure that will hold the result
    /// @return pointer to the passed in result on success, nullptr on failure
    struct tm *gmtime_r(struct tm *result)
    {
        time_t now = time();
        return ::gmtime_r(&now, result);
    }

    /// Get the day of the week.
    /// @returns day of the week (0 - 6, Sunday - Saturday) upon success,
    ///          else -1 on failure
    int day_of_week()
    {
        struct tm tm;
        if (gmtime_r(&tm) == nullptr)
        {
            return -1;
        }
        return tm.tm_wday;
    }

    /// Get the day of the year.
    /// @returns day of the year (0 - 365) upon success, else -1 on failure
    int day_of_year()
    {
        struct tm tm;
        if (gmtime_r(&tm) == nullptr)
        {
            return -1;
        }
        return tm.tm_yday;
    }

    /// Report the clock rate as a 12-bit fixed point number
    /// (-512.00 to 511.75).
    /// @return clock rate 
    int16_t get_rate_quarters()
    {
        return rate_;
    }

    /// Test of the clock is running. Running backwards (negative rate) also
    /// qualifies as running.
    /// @return true if running, else false
    bool is_running()
    {
        AtomicHolder h(this);
        return rate_ != 0 && started_;
    }

    /// Test of the clock is started (rate could still be 0).
    /// @return true if started, else false
    bool is_started()
    {
        return started_;
    }

    /// Convert fast clock period to a period in real nsec. Result will
    /// preserve sign.
    /// @param rate rate to use in conversion
    /// @param fast_sec period in seconds in fast clock time
    /// @param real_nsec period in nsec
    /// @return true if successfull, else false if clock is not running
    bool fast_sec_to_real_nsec_period(int16_t rate, time_t fast_sec,
                                      long long *real_nsec)
    {
        if (rate != 0 && rate >= -2048 && rate <= 2047)
        {
            *real_nsec = ((SEC_TO_NSEC(std::abs(fast_sec)) * 4) +
                          (std::abs(rate) / 2)) / rate;
            if (fast_sec < 0)
            {
                *real_nsec = -(*real_nsec);
            }
            return true;
        }
        else
        {
            return false;
        }
    }

    /// Convert period in real nsec to a fast clock period. Result will
    /// preserve sign.
    /// @param rate rate to use in conversion
    /// @param real_nsec period in nsec
    /// @param fast_sec period in seconds in fast clock time
    /// @return true if successfull, else false if clock is not running
    bool real_nsec_to_fast_sec_period(int16_t rate, long long real_nsec,
                                      time_t *fast_sec)
    {
        if (rate != 0 && rate >= -2048 && rate <= 2047)
        {
            *fast_sec = (std::abs(NSEC_TO_SEC(real_nsec * rate)) + 2) / 4;
            if ((real_nsec < 0 && rate > 0) || (real_nsec >= 0 && rate < 0))
            {
                *fast_sec = -(*fast_sec);
            }
            return true;
        }
        else
        {
            return false;
        }
    }

    /// Convert a fast time to absolute nsec until it will occur.
    ///
    /// Caution!!! Not an atomic operation if called from a different thread or
    /// executor than the thread or executor being used by this object.
    ///
    /// @param fast_sec seconds in rate time
    /// @param real_nsec period in nsec until it will occure
    /// @return true if successfull, else false if clock is not running
    bool real_nsec_until_fast_time_abs(time_t fast_sec, long long *real_nsec)
    {
        if (fast_sec_to_real_nsec_period_abs(fast_sec - seconds_, real_nsec))
        {
            *real_nsec += timestamp_;
            long long monotonic = OSTime::get_monotonic();
            *real_nsec -= monotonic;
            return true;
        }
        else
        {
            return false;
        }
    }

    /// Convert fast clock absolute (negative or positive) period to a positive
    /// (absolute) period in real nsec.
    ///
    /// Caution!!! Not an atomic operation if called from a different thread or
    /// executor than the thread or executor being used by this object.
    ///
    /// @param sec period in seconds in fast clock time
    /// @param real_nsec period in nsec
    /// @return true if successfull, else false if clock is not running
    bool fast_sec_to_real_nsec_period_abs(time_t fast_sec, long long *real_nsec)
    {
        if (fast_sec_to_real_nsec_period(rate_, fast_sec, real_nsec))
        {
            *real_nsec = std::abs(*real_nsec);
            return true;
        }
        else
        {
            return false;
        }
    }

    /// Register a callback for when the time synchronization is updated. The
    /// context of the caller will be from a state flow on the Node Interface
    /// executor.
    /// @param callback function callback to be called.
    /// @return handle to entry that can be used in update_unsubscribe
    UpdateSubscribeHandle update_subscribe_add(std::function<void()> callback)
    {
        AtomicHolder h(this);
        for (size_t i = 0; i < callbacks_.size(); ++i)
        {
            // atempt to garbage collect unused entries
            if (callbacks_[i] == nullptr)
            {
                callbacks_[i] = callback;
                return i;
            }
        }
        callbacks_.emplace_back(callback);
        return callbacks_.size() - 1;
    }

    /// Unregister a callback for when the time synchronization is updated.
    /// @param handle returned from corresponding update_subscribe
    void update_subscribe_remove(UpdateSubscribeHandle handle)
    {
        AtomicHolder h(this);
        callbacks_[handle] = nullptr;
    }

    /// Accessor method to get the Node reference
    /// @return Node reference
    Node *node()
    {
        return node_;
    }

    /// Accessor method to get the (48-bit) Clock ID.
    /// @return clock ID
    NodeID clock_id()
    {
        return eventBase_ >> 16;
    }

    /// Access method to get the (64-bit) Event ID base.
    EventId event_base()
    {
        return eventBase_;
    }

    /// Recalculate the struct tm representation of time.
    /// @return last calculated time in the form of a struct tm
    const struct tm *gmtime_recalculate()
    {
        gmtime_r(&tm_);
        return &tm_;
    }

    /// Get the last calculated reprentation of time.
    /// @return last calculated time in the form of a struct tm
    const struct tm *gmtime_get()
    {
        return &tm_;
    }

protected:
    class SetFlow : public StateFlowBase
    {
    public:
        /// Supported operations.
        enum Command
        {
            SET_TIME, ///< set time request
            SET_DATE, ///< set date request
            SET_YEAR, ///< set year reauest
            SET_RATE, ///< set rate request
            START, ///< stop request
            STOP, ///< start request
        };

        /// Constructor.
        /// @param clock the parent clock instance
        /// @param command operation to perform
        /// @param data1 first data argument
        /// @param data2 second data argument
        SetFlow(BroadcastTime *clock, Command command,
               int data1 = 0, int data2 = 0)
            : StateFlowBase(clock->node()->iface())
            , clock_(clock)
            , command_(command)
            , data1_(data1)
            , data2_(data2)
        {
            start_flow(STATE(send_event));
        }

    private:

        /// Send the necessary event.
        /// @return delete_this()
        Action send_event()
        {
            uint64_t event_id;
            switch (command_)
            {
                case SET_TIME:
                    event_id = BroadcastTimeDefs::EVENT_SET_SUFFIX_MASK |
                        BroadcastTimeDefs::time_to_event(clock_->event_base(),
                                                         data1_, data2_);
                    break;
                case SET_DATE:
                    event_id = BroadcastTimeDefs::EVENT_SET_SUFFIX_MASK |
                        BroadcastTimeDefs::date_to_event(clock_->event_base(),
                                                         data1_, data2_);
                    break;
                case SET_YEAR:
                    event_id = BroadcastTimeDefs::EVENT_SET_SUFFIX_MASK |
                        BroadcastTimeDefs::year_to_event(clock_->event_base(),
                                                         data1_);
                    break;
                case SET_RATE:
                    event_id = BroadcastTimeDefs::EVENT_SET_SUFFIX_MASK |
                        BroadcastTimeDefs::rate_to_event(clock_->event_base(),
                                                         data1_);
                    break;
                case START:
                    event_id = clock_->event_base() |
                               BroadcastTimeDefs::START_EVENT_SUFFIX;
                    break;
                case STOP:
                    event_id = clock_->event_base() |
                               BroadcastTimeDefs::STOP_EVENT_SUFFIX;
                    break;
                default:
                    // should never get here.
                    LOG_ERROR("Unhanded SetFlow command");
                    return delete_this();
            }

            if (!clock_->node()->is_initialized())
            {
                // since the node is not yet initialized, events get thrown on
                // the floor, we will try a shortcut instead
                clock_->set_shortcut(event_id);
                return delete_this();
            }
            else
            {
                writer_.WriteAsync(clock_->node(), Defs::MTI_EVENT_REPORT,
                    WriteHelper::global(), eventid_to_buffer(event_id), this);

                return wait_and_call(STATE(delete_this));
            }
        }

        WriteHelper writer_; ///< helper for sending event messages
        BroadcastTime *clock_; ///< the parent clock instance
        Command command_; ///< operation to perform;
        int data1_; ///< first data argument
        int data2_; ///< second data argument
    };

    /// Constructor.
    /// @param node the virtual node that will be listening for events and
    ///             responding to Identify messages.
    /// @param clock_id 48-bit unique identifier for the clock instance
    BroadcastTime(Node *node, NodeID clock_id)
        : StateFlowBase(node->iface())
        , node_(node)
        , eventBase_((uint64_t)clock_id << 16)
        , writer_()
        , timer_(this)
        , callbacks_()
        , timestamp_(OSTime::get_monotonic())
        , seconds_(0)
        , rate_(0)
        , rateRequested_(0)
        , started_(false)
    {
        // use a process-local timezone
        clear_timezone();

        time_t time = 0;
        ::gmtime_r(&time, &tm_);
        tm_.tm_isdst = 0;
    }

    virtual ~BroadcastTime()
    {
    }

    /// Try the possible set event shortcut. This is typically a bypass of the
    /// OpenLCB loopback.
    /// @param event event that we would be "setting"
    virtual void set_shortcut(uint64_t event)
    {
    }

    /// Service all of the attached update subscribers. These are called when
    /// there are jumps in time or if the clock is stopped or started.
    void service_callbacks()
    {
        AtomicHolder h(this);
        for (auto n : callbacks_)
        {
            if (n)
            {
                n();
            }
        }
    }

    struct tm tm_; ///< the time we are last set to as a struct tm

    Node *node_; ///< OpenLCB node to export the consumer on
    EventId eventBase_; ///< 48-bit unique identifier for the clock instance
    WriteHelper writer_; ///< helper for sending event messages
    StateFlowTimer timer_; ///< timer helper

    /// update subscribers
    std::vector<std::function<void()>> callbacks_;

    long long timestamp_; ///< monotonic timestamp from last server update
    time_t seconds_; ///< clock time in seconds from last server update
    int16_t rate_; ///< effective clock rate
    int16_t rateRequested_; ///< pending clock rate

    uint16_t started_ : 1; ///< true if clock is started

private:
    /// Reset our process local timezone environment to GMT0.
    void clear_timezone();

    DISALLOW_COPY_AND_ASSIGN(BroadcastTime);
};

} // namespace openlcb

#endif // _OPENLCB_BROADCASTTIME_HXX_
