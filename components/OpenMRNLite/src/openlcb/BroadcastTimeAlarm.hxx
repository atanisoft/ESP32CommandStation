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
 * @file BroadcastTimeAlarm.hxx
 *
 * Implementation of Broadcast Time Protocol alarms.
 *
 * @author Stuart W. Baker
 * @date 29 October 2018
 */

#ifndef _OPENLCB_BROADCASTTIMEALARM_HXX_
#define _OPENLCB_BROADCASTTIMEALARM_HXX_

#include "openlcb/BroadcastTime.hxx"

namespace openlcb
{

/// Basic alarm type that all other alarms are based off of.
class BroadcastTimeAlarm : public StateFlowBase, protected Atomic
{
public:
    /// Constructor.
    /// @param node the virtual node that our StateFlowBase service will be
    ///             derived from
    /// @param clock clock that our alarm is based off of
    /// @param callback callback for when alarm expires
    BroadcastTimeAlarm(Node *node, BroadcastTime *clock,
        std::function<void(BarrierNotifiable *)> callback)
        : StateFlowBase(node->iface())
        , clock_(clock)
        , wakeup_(this)
        , callback_(callback)
        , timer_(this)
        , bn_()
        , bnPtr_(nullptr)
        , expires_(0)
        , running_(false)
        , set_(false)
#if defined(GTEST)
        , shutdown_(false)
#endif
        , updateSubscribeHandle_(clock->update_subscribe_add(
              std::bind(&BroadcastTimeAlarm::update_notify, this)))
    {
        // By ensuring that the alarm runs in the same thread context as the
        // clock which it uses, we can have much simpler logic for avoiding
        // race conditions in this implementation.
        HASSERT(service()->executor() == clock_->service()->executor());
        start_flow(STATE(entry));
    }

    /// Destructor.
    ~BroadcastTimeAlarm()
    {
        clock_->update_subscribe_remove(updateSubscribeHandle_);
    }

    /// Start the alarm to expire at the given period from now.
    /// @param period in fast seconds from now to expire. @ref period is a
    ///        a signed value. If the fast time rate is negative, the @ref
    ///        period passed in should also be negative for an expiration in
    ///        the future.
    void set_period(time_t period)
    {
        set(clock_->time() + period);
    }

    /// Start the alarm to expire at the given fast time.
    /// @param time in seconds since epoch to expire
    void set(time_t time)
    {
        bool need_wakeup = false;
        {
            AtomicHolder h(this);
            expires_ = time;
            running_ = true;
            if (!set_)
            {
                need_wakeup = true;
                set_ = true;
            }
        }
        if (need_wakeup)
        {
            wakeup_.trigger();
        }
    }

    /// Inactivate the alarm.
    void clear()
    {
        {
            AtomicHolder h(this);
            running_ = false;
            set_ = false;
        }
        // rather than waking up the state flow, just let it expire naturally.
    }

#if defined(GTEST)
    void shutdown()
    {
        shutdown_ = true;
        wakeup_.trigger();
    }

    bool is_shutdown()
    {
        return is_terminated();
    }
#endif

protected:
    /// Entry point to state flow.
    /// @return next state setup()
    virtual Action entry()
    {
        return call_immediately(STATE(setup));
    }

    /// Called by the clock when time, rate, or running state has changed.
    virtual void update_notify()
    {
        bool need_wakeup = false;
        {
            AtomicHolder h(this);
            if (running_ && !set_)
            {
                set_ = true;
                need_wakeup = true;
            }
        }
        if (need_wakeup)
        {
            wakeup_.trigger();
        }
    }

    BroadcastTime *clock_; ///< clock that our alarm is based off of

private:
    // Wakeup helper
    class Wakeup : public Executable
    {
    public:
        /// Constructor.
        /// @param alarm our parent alarm that we will awaken
        Wakeup(BroadcastTimeAlarm *alarm)
            : alarm_(alarm)
        {
        }

        /// Trigger the wakeup to run.
        void trigger()
        {
            alarm_->service()->executor()->add(this);
        }

    private:
        /// Entry point. This funciton will be called when *this gets scheduled
        /// on the CPU.
        void run() override
        {
            alarm_->wakeup();
        }

        BroadcastTimeAlarm *alarm_; ///< our parent alarm we will wakeup
    };

    /// Setup, or wait to setup alarm.
    /// @return expired() if alarm is set and already expired,
    ///         timeout() if alarm will expire in the future,
    ///         setup() if clock and/or alarm is not currently active
    Action setup()
    {
#if defined(GTEST)
        if (shutdown_)
        {
            return exit();
        }
#endif

        AtomicHolder h(this);
        if (set_)
        {
            set_ = false;
            time_t now = clock_->time();

            if ((now >= expires_ && clock_->get_rate_quarters() > 0) ||
                (now <= expires_ && clock_->get_rate_quarters() < 0))
            {
                // have already met the alarm conditions,
                // typically won't get here
                return call_immediately(STATE(expired));
            }
            else if (clock_->is_running())
            {
                long long real_expires;
                bool result =
                    clock_->real_nsec_until_fast_time_abs(expires_,
                                                          &real_expires);
                HASSERT(result);
                return sleep_and_call(&timer_, real_expires, STATE(timeout));
            }
        }

        bnPtr_ = bn_.reset(this);
        return wait_and_call(STATE(setup));
    }

    /// Wait for timeout or early trigger.
    /// @return setup() if the timer is triggered prematurely, else
    ///         expired() if the timer has expired
    Action timeout()
    {
        if (timer_.is_triggered())
        {
            // this is a wakeup, not a timeout
            return call_immediately(STATE(setup));
        }
        else
        {
            // timeout
            return call_immediately(STATE(expired));
        }
    }

    /// Handle action on timer expiration.
    /// @return setup()
    Action expired()
    {
        bnPtr_ = bn_.reset(this);
        if (running_ && clock_->is_running() && callback_)
        {
            running_ = false;
            callback_(bnPtr_->new_child());
        }

        return wait_and_call(STATE(setup));
    }

    /// Wakeup the state machine. Must be called from this service's executor.
    void wakeup()
    {
        timer_.ensure_triggered();
        if (bnPtr_)
        {
            bnPtr_ = nullptr;
            bn_.notify();
        }
    }

    Wakeup wakeup_; ///< wakeup helper for scheduling alarms
    /// callback for when alarm expires
    std::function<void(BarrierNotifiable *)> callback_;
    StateFlowTimer timer_; ///< timer helper
    BarrierNotifiable bn_; ///< notifiable for callback callee
    BarrierNotifiable *bnPtr_; ///< not null we have an outstanding notification
    time_t expires_; ///< time at which the alarm expires
    uint8_t running_  : 1; ///< true if running (alarm armed), else false
    uint8_t set_      : 1; ///< true if a start request is pending
#if defined(GTEST)
    uint8_t shutdown_ : 1; ///< true if test has requested shutdown
#endif
    /// handle to the update subscrition used for unsubcribing in the destructor
    BroadcastTime::UpdateSubscribeHandle updateSubscribeHandle_;

    /// make our wakeup agent a friend
    friend class BroadcastTimeAlarm::Wakeup;

    DISALLOW_COPY_AND_ASSIGN(BroadcastTimeAlarm);
};

/// Specialization of BroadcastTimeAlarm meant to expire at each date rollover.
class BroadcastTimeAlarmDate : public BroadcastTimeAlarm
{
public:
    /// Constructor.
    /// @param node the virtual node that our StateFlowBase service will be
    ///             derived from
    /// @param clock clock that our alarm is based off of
    /// @param callback callback for when alarm expires
    BroadcastTimeAlarmDate(Node *node, BroadcastTime *clock,
        std::function<void(BarrierNotifiable *)> callback)
        : BroadcastTimeAlarm(
              node, clock, std::bind(&BroadcastTimeAlarmDate::expired_callback,
                                     this, std::placeholders::_1))
        , callbackUser_(callback)
    {
    }

    /// Destructor.
    ~BroadcastTimeAlarmDate()
    {
    }

private:
    /// Entry point of the state machine.
    /// @return BroadcastTimeAlarm::entry();
    Action entry() override
    {
        update_notify();

        return BroadcastTimeAlarm::entry();
    }

    /// Reset the expired time based on what time it is now.
    void reset_expired_time()
    {
        const struct tm *tm = clock_->gmtime_recalculate();
        time_t seconds = clock_->time();

        if (clock_->get_rate_quarters() > 0)
        {
            set(seconds + (60 - tm->tm_sec) +
                          (60 * (59 - tm->tm_min)) +
                          (60 * 60 * (23 - tm->tm_hour)));
        }
        else if (clock_->get_rate_quarters() < 0)
        {
            set(seconds - ((tm->tm_sec + 1) +
                           (60 * (tm->tm_min)) +
                           (60 * 60 * tm->tm_hour)));
        }
    }

    /// callback for when the alarm expires
    /// @param done used to notify we are finished
    void expired_callback(BarrierNotifiable *done)
    {
        reset_expired_time();
        callbackUser_ ? callbackUser_(done) : done->notify();
    }

    /// Called when the clock time has changed.
    void update_notify() override
    {
        reset_expired_time();
        BroadcastTimeAlarm::update_notify();
    }

    /// callback for when alarm expires
    std::function<void(BarrierNotifiable *)> callbackUser_;

    DISALLOW_COPY_AND_ASSIGN(BroadcastTimeAlarmDate);
};

/// Specialization of BroadcastTimeAlarm meant to expire at each minute.
class BroadcastTimeAlarmMinute : public BroadcastTimeAlarm
{
public:
    /// Constructor.
    /// @param node the virtual node that our StateFlowBase service will be
    ///             derived from
    /// @param clock clock that our alarm is based off of
    /// @param callback callback for when alarm expires
    BroadcastTimeAlarmMinute(Node *node, BroadcastTime *clock,
        std::function<void(BarrierNotifiable *)> callback)
        : BroadcastTimeAlarm(
              node, clock,
              std::bind(&BroadcastTimeAlarmMinute::expired_callback, this,
                        std::placeholders::_1))
        , callbackUser_(callback)
    {
    }

    /// Destructor.
    ~BroadcastTimeAlarmMinute()
    {
    }

private:
    /// Entry point of the state machine.
    /// @return BroadcastTimeAlarm::entry();
    Action entry() override
    {
        update_notify();

        return BroadcastTimeAlarm::entry();
    }

    /// Reset the expired time based on what time it is now.
    /// @param force_on_match true to force an expiration if on a minute
    ///                       rollover boundary
    void reset_expired_time(bool force_on_match = false)
    {
        const struct tm *tm = clock_->gmtime_recalculate();
        time_t seconds = clock_->time();

        if (force_on_match)
        {
            if ((clock_->get_rate_quarters() > 0 && tm->tm_sec == 0) ||
                (clock_->get_rate_quarters() < 0 && tm->tm_sec == 59))
            {
                set(seconds);
                return;
            }
        }
        if (clock_->get_rate_quarters() > 0)
        {
            set(seconds + (60 - tm->tm_sec));
        }
        else if (clock_->get_rate_quarters() < 0)
        {
            set(seconds - (tm->tm_sec + 1));
        }
    }

    /// callback for when the alarm expires
    /// @param done used to notify we are finished
    void expired_callback(BarrierNotifiable *done)
    {
        reset_expired_time();
        callbackUser_ ? callbackUser_(done) : done->notify();
    }

    /// Called when the clock time has changed.
    void update_notify() override
    {
        reset_expired_time(true);
        BroadcastTimeAlarm::update_notify();
    }

    /// callback for when alarm expires
    std::function<void(BarrierNotifiable *)> callbackUser_;

    DISALLOW_COPY_AND_ASSIGN(BroadcastTimeAlarmMinute);
};

} // namespace openlcb

#endif // _OPENLCB_BROADCASTTIMEALARM_HXX_