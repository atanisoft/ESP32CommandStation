/** \copyright
 * Copyright (c) 2020, Balazs Racz
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
 * \file FakeClock.hxx
 *
 * Helper class for unit tests that want to control the advancement of time by
 * hand.
 *
 * @author Balazs Racz
 * @date 28 Nov 2020
 */

#ifndef _OS_FAKECLOCK_HXX_
#define _OS_FAKECLOCK_HXX_

#include "executor/Executor.hxx"
#include "os/os.h"
#include "utils/Singleton.hxx"

/// Stores the private variables of a fake clock.
struct FakeClockContent
{
protected:
    /// @param t the starting timestamp for the fake clock.
    FakeClockContent(long long t)
        : lastTime_(t)
    {
    }

    long long lastTime_;
};

/// Class that injects a fake progression of time for unit tests. When this
/// class is created, the time as returned by os_get_time_monotonic()
/// freezes. From that point on time only moves forward when advance() is
/// called.
///
/// There can be at most one instance of this class at any time.
class FakeClock : private FakeClockContent, public Singleton<FakeClock>
{
public:
    FakeClock()
        : FakeClockContent(os_get_time_monotonic())
    {
    }

    /// Advances the time returned by os_get_time_monotonic().
    /// @param nsec how much the time should jump forward (relative to now).
    void advance(long long nsec)
    {
        lastTime_ += nsec;
        // Wakes up all executors. This will cause them to evaluate if their
        // timers have something expired.
        ExecutorBase *current = ExecutorBase::link_head();
        while (current)
        {
            current->add(new CallbackExecutable([]() {}));
            current = current->link_next();
        }
    }

    /// @return the currently set time.
    long long get_time_nsec()
    {
        return lastTime_++;
    }

private:
};

#endif // _OS_FAKECLOCK_HXX_