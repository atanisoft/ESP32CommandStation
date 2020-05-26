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
 * \file Timer.cxx
 *
 * Class for managing timers and timeouts in the executor.
 *
 * @author Stuart W Baker and Balazs Racz
 * @date 16 Mar 2014
 */

#include "executor/Timer.hxx"
#include "executor/Executor.hxx"
#include "os/os.h"

Timer::~Timer()
{
    HASSERT(!isActive_);
    HASSERT(!isExpired_);
}

void Timer::run()
{
    isExpired_ = 0;
    long long new_period = timeout();
    if (new_period == RESTART)
    {
        when_ += period_;
        isActive_ = 1;
        activeTimers_->schedule_timer(this);
    }
    else if (new_period == DELETE)
    {
        delete this;
    }
    else if (new_period > 0)
    {
        start(new_period);
    }
}

ActiveTimers::~ActiveTimers()
{
}

void ActiveTimers::notify()
{
    if (isPending_.exchange(1) == 0)
    {
        executor_->add(this);
    }
}

void ActiveTimers::run()
{
    isPending_ = 0;
    // Do nothing; the work of scheduling users is done in the get_next_timeout
    // call.
}

long long ActiveTimers::get_next_timeout()
{
    OSMutexLock l(&lock_);

    QMember **last = &activeTimers_.next;
    Timer *current_timer = static_cast<Timer *>(*last);
    long long now = OSTime::get_monotonic();
    bool found_timer = false;
    while (current_timer && current_timer->when_ <= now)
    {
        // Deques next timer.
        found_timer = true;
        *last = current_timer->next;
        current_timer->next = nullptr;

        current_timer->isActive_ = 0;
        current_timer->isExpired_ = 1;
        // Puts it on the executor.
        executor_->add(current_timer, current_timer->priority_);
        // Takes the next timer.
        current_timer = static_cast<Timer *>(*last);
    }

    if (found_timer)
    {
        return 0;
    }
    else if (current_timer)
    {
        long long ret = current_timer->when_ - now;
        return ret;
    }
    else
    {
        // Wakes up the timer service every now and then. It won't make any
        // difference.
        return SEC_TO_NSEC(3600);
    }
}

bool ActiveTimers::empty() {
    OSMutexLock l(&lock_);

    QMember **last = &activeTimers_.next;
    Timer *current_timer = static_cast<Timer *>(*last);
    return (current_timer == nullptr);
}

void ActiveTimers::schedule_timer(Timer *timer)
{
    OSMutexLock l(&lock_);
    insert_locked(timer);
}

void ActiveTimers::insert_locked(Timer *timer)
{
    HASSERT(timer);
    HASSERT(timer->next == nullptr);

    QMember **last = &activeTimers_.next;
    Timer *current_timer = static_cast<Timer *>(*last);
    while (current_timer && current_timer->when_ <= timer->when_)
    {
        last = &current_timer->next;
        current_timer = static_cast<Timer *>(*last);
    }
    // Inserts into the queue.
    timer->next = current_timer;
    *last = timer;

    // This will wake up the executor, which will schedule all expired timers
    // and recompute sleep length.
    notify();
}

void ActiveTimers::remove_locked(Timer *timer)
{
    HASSERT(timer);
    // Removes the timer from the queue.
    QMember **last = &activeTimers_.next;
    while (*last && *last != timer)
    {
        last = &((*last)->next);
    }
    HASSERT(*last == timer);
    *last = timer->next;
    timer->next = nullptr;
}

void ActiveTimers::update_timer(Timer *timer)
{
    HASSERT(timer);
    OSMutexLock l(&lock_);
    remove_locked(timer);
    insert_locked(timer);
}

void ActiveTimers::remove_timer(Timer *timer)
{
    HASSERT(timer);
    OSMutexLock l(&lock_);
    remove_locked(timer);
    timer->isActive_ = 0;
}
