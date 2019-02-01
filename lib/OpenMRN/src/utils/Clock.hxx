/** \copyright
 * Copyright (c) 2015, Balazs Racz
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
 * \file Clock.hxx
 *
 * Abstracts getting time in a testable fashion.
 *
 * @author Balazs Racz
 * @date 5 May 2015
 */

#include "os/os.h"

/// Virtual base class for a real-time clock.
///
/// The purpose for this base class is to have a real and a mock
/// implementation. The Clock* pointer can be injected into components that
/// need to use real time for business logic, and then tests can inject a mock
/// or fake clock to drive the test scenario manually without needing to call
/// sleep.
///
/// see @ref RealClock and @ref MockClock.
class Clock
{
public:
    /// @returns the current time of the clock in nanoseconds.
    virtual long long get_time_nsec() = 0;
};

/// Implementation of @ref Clock that returns the current real time form the OS.
class RealClock : public Clock
{
public:
    long long get_time_nsec()
    {
        return os_get_time_monotonic();
    }
};

/// Fake implementation of @ref Clock that returns an injected time.
///
/// @todo(balazs.racz) this should be called FakeClock instead.
class MockClock : public Clock
{
public:
    /// Consturctor. @param time the clock vale to set.
    MockClock(long long time) : time_(time)
    {
    }
    /// @return the mock time.
    long long get_time_nsec()
    {
        return time_;
    }
    /// @param time is the mock time to set.
    void set_time(long long time)
    {
        time_ = time;
    }
private:
    /// Current mock time.
    long long time_;
};
