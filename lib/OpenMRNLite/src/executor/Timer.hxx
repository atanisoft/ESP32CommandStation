/** \copyright
 * Copyright (c) 2014, Stuart W Baker and Balazs Racz
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
 * \file Timer.hxx
 *
 * Class for managing timers and timeouts in the executor.
 *
 * @author Stuart W Baker and Balazs Racz
 * @date 16 Mar 2014
 */

#ifndef _EXECUTOR_TIMER_HXX_
#define _EXECUTOR_TIMER_HXX_

#include "executor/Notifiable.hxx"
#include "utils/Buffer.hxx"
#include "utils/QMember.hxx"
#include "os/OS.hxx"

class Timer;
class ExecutorBase;

/** Class that manages the list of active timers. The Executor uses this class
 * tightly in its sleep-execute loop. */
class ActiveTimers : public Executable
{
public:
    /// Constructor.
    ///
    /// @param executor parent that will use this instance.
    ActiveTimers(ExecutorBase *executor)
        : executor_(executor)
        , isPending_(0)
    {
    }

    ~ActiveTimers();

    /** Tell when the first timer will expire. If there are no active timers,
     * returns a large number.
     * @returns the timer in nanoseconds to sleep until the next timer to wake
     * up. Can return 0 if there is an expired timer. */
    long long get_next_timeout();

    /** @return true if there are no timers waiting. */
    bool empty();
    
    /** Adds a new timer to the active timer list. It is OK to schedule a timer
     * that is already expired, which will then wake up the executor.
     *
     * @param timer is the timer to schedule. It must not be already
     * scheduled. */
    void schedule_timer(::Timer *timer);

    /** Updates the expiration time of an already scheduled timer. This call is
     * somewhat expensive, because it needs to walk the entire queue of active
     * timers. May wake up the executor.
     *
     * @param timer is the timer whose next execution time has been updated. It
     * must already be scheduled. */
    void update_timer(::Timer *timer);

    /** Deletes an already scheduled but not yet expired timer. This call is
     * somewhat expensive, because it needs to walk the entire queue of active
     * timers. Asserts that the timer is in fact not yet expired.
     *
     * @param timer is the timer to delete. */
    void remove_timer(::Timer *timer);

    /** @returns the executor on which the timers will be scheduled. */
    ExecutorBase *executor()
    {
        return executor_;
    }

    /** Notification callback from the timer. Schedules *this on the
     * executor. */
    void notify() override;

    /** Callback from the executor. Puts all expired timers on the executor. */
    void run() override;

private:
    /** Removes a timer from the active list. Assert fails if it is not
     * there. Caller must hold the lock. 
     * @param timer what to remove from the active list. */
    void remove_locked(::Timer *timer);

    /** Inserts a timer into the active list. Caller must hold the lock.
     * @param timer what to insert into the active list. */
    void insert_locked(::Timer *timer);

    /// Parent.
    ExecutorBase *executor_;
    /// Protects the timer list.
    OSMutex lock_;
    /// List of timers that are scheduled.
    QMember activeTimers_;
    /// 1 if we in the executor's queue.
    unsigned isPending_ : 1;

    friend class TimerTest;

    DISALLOW_COPY_AND_ASSIGN(ActiveTimers);
};

/** A timer that can schedule itself to run on an executor at specified times
 * in the future.
 */
class Timer : public Executable
{
public:
    /** Constructor.
     * @param timers list of active timers from the executor.
     */
    Timer(ActiveTimers *timers)
        : activeTimers_(timers)
        , priority_(UINT_MAX)
        , when_(0)
        , period_(0)
        , isActive_(0)
        , isExpired_(0)
        , isCancelled_(0)
        , tcRequestStop_(0)
    {
    }

    /** Destructor. */
    ~Timer();

    /** Callback from the executor when this timer is scheduled. It will call
     * the virtual method timeout(). */
    void run() override;

    /** Special return values from the timeout function. */
    enum
    {
        NONE = 0,    /**< Do not restart the timer */
        RESTART = 1, /**< Restart the timer with existing period */
        DELETE = -1, /**< delete the timer, use with extreme caution */
    };

    /** Clients of timer should override this function. It will be called on
     * the executor of the timer.
     * @returns the new timer period, or one of the above special values. */
    virtual long long timeout() = 0;

    /** Starts a timer. The timer must not be active, and neither expired at
     * the time of call.
     * @param period period in nanoseconds before expiration. If not specified,
     * the timer will expire immediately.
     */
    void start(long long period = -1)
    {
        HASSERT(!isActive_);
        HASSERT(!isExpired_);
        isActive_ = 1;
        isCancelled_ = 0;
        when_ = OSTime::get_monotonic() + period;
        period_ = period;
        activeTimers_->schedule_timer(this);
    }

    /** Starts the timer with an absolute deadline. The timer must not be
     * active, and neither expired at the time of call.
     * @param expiry_time_nsec absolute time when the timer should expire in nanoseconds. If less than the current time, the timer will expire immediately. */
    void start_absolute(long long expiry_time_nsec)
    {
        HASSERT(!isActive_);
        HASSERT(!isExpired_);
        isActive_ = 1;
        isCancelled_ = 0;
        when_ = expiry_time_nsec;
        period_ = when_ - OSTime::get_monotonic();
        activeTimers_->schedule_timer(this);
    }

    /** Restart a timer with the existing period but from the current
     * time. This function must be called on the timer executor only. Calling
     * restart on an expired timer has no effect and is ignored.
     */
    void restart()
    {
        /// @todo(balazs.racz) assert here that we are on the given executor.
        if (isExpired_)
            return;
        when_ = OSTime::get_monotonic() + period_;
        isCancelled_ = 0;
        if (isActive_)
        {
            activeTimers_->update_timer(this);
        }
        else
        {
            isActive_ = 1;
            activeTimers_->schedule_timer(this);
        }
    }

    /** This will wakeup the timer prematurely, immediately.  The timer must be
     * active or expired at the time of call.  This function must be called on
     * the timer executor only.  The timer period remains unchanged in case it
     * is restarted.
     */
    void trigger()
    {
        /// @todo(balazs.racz) assert here that we are on the given executor.
        if (isExpired_)
            return;
        HASSERT(isActive_);
        isCancelled_ = 1;
        when_ = 2; // in the past
        activeTimers_->update_timer(this);
    }

    /** Triggers the timer if it is not expired yet. */
    void ensure_triggered()
    {
        if (isActive_) trigger();
    }

    /** Dangerous, do not call. Contains a race condition. Production users
     * should use the trigger() method to cancel an active timer.
     * 
     * Cancels an active timer. Crashes if the timer is expired. Use in
     * unittests only. Does nothing if the timer is not active. Can be called
     * from outside the main executor. */
    void cancel()
    {
        when_ = INT64_MAX;  // This will ensure the we don't get from active to
                            // expired from now on.
        HASSERT(!isExpired_);
        isCancelled_ = 1;
        if (isActive_) {
            activeTimers_->remove_timer(this);
        }
    }

    /** @returns true if the timer was triggered due to cancel() or trigger();
     * false if it was timed out. */
    bool is_triggered()
    {
        return isCancelled_;
    }

    /** Sets the timer as if it was woken up by a trigger(), even if it was
     * never started. This is helpful for boundary conditions. */
    void set_triggered()
    {
        isCancelled_ = 1;
    }


private:
    friend class ActiveTimers;  // for scheduling an expiring timers
    friend class CountingTimer; // for testing

    /** Points to the executor's timer structure. Not owned. */
    ActiveTimers *activeTimers_;
    /** what priority to schedule this timer at */
    unsigned priority_;
    /** when in nanoseconds timer should expire */
    long long when_;
    /** period in nanoseconds for timer */
    long long period_;
    /** true when the timer is in the active timers list */
    unsigned isActive_ : 1;
    /** True when the timer is in the pending executables list of the
     * Executor. */
    unsigned isExpired_ : 1;
    /** Was the timer cancelled or did the timer expire regularly? 1 if
     * cancelled or triggered. */
    unsigned isCancelled_ : 1;
    /** For children: 1 if a repeated timer should stop sending wakeups. */
    unsigned tcRequestStop_ : 1;

    DISALLOW_COPY_AND_ASSIGN(Timer);
};

/** Class usable by synchronous code to utilize a timeout.
 *
 * usage:
 * SyncTimeout t;
 * t.start(MSEC_TO_NSEC(100));
 * t.wait_for_notification();
 * if (t.is_triggered())
 * {  call was success -- response arrived } 
 * else 
 * {  failure -- timeout }
 * */
class SyncTimeout : public ::Timer {
public:
    /// @param timers should come from the executor on which we're waiting.
    SyncTimeout(ActiveTimers *timers) : Timer(timers)
    {
    }

    /** Blocks the current thread's execution until the timeout is expired or
     * triggered. */
    void wait_for_notification() {
        n_.wait_for_notification();
    }

private:
    /** Clients of timer should override this function. It will be called on
     * the executor of the timer.
     * @returns the new timer period, or one of the above special values. */
    long long timeout() OVERRIDE {
        n_.notify();
        return NONE;
    }

    /// Blocks the calling thread until triggered or timeout expired.
    SyncNotifiable n_;
};

#endif // _EXECUTOR_TIMER_HXX_
