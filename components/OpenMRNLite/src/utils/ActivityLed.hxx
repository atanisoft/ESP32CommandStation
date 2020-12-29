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
 * \file ActivityLed.hxx
 *
 * State flow that controls an activity LED based on triggers of events.
 *
 * @author Balazs Racz
 * @date 26 Sep 2020
 */

#ifndef _UTILS_ACTIVITYLED_HXX_
#define _UTILS_ACTIVITYLED_HXX_

#include "executor/StateFlow.hxx"
#include "os/Gpio.hxx"

/// Operates an LED to visually display some activity. When the activity()
/// function is called at least once within a period, we turn the LED on for
/// the next period, if no call was made, the LED turns off for the next
/// period.
class ActivityLed : private ::Timer
{
public:
    /// Constructor.
    /// @param service defines which executor this timer should be running on.
    /// @param pin the LED of the output. Will be high for activity, low for
    /// inactivity. Use InvertedGPIO if needed.
    /// @param period defines in nanosecond the time to spend between updates.
    ActivityLed(
        Service *service, const Gpio *pin, long long period = MSEC_TO_NSEC(33))
        : ::Timer(service->executor()->active_timers())
        , gpio_(pin)
    {
        start(period);
    }

    /// Call this function when activity happens.
    void activity()
    {
        triggerCount_++;
    }

private:
    long long timeout() override
    {
        if (triggerCount_)
        {
            gpio_->write(true);
            triggerCount_ = 0;
        }
        else
        {
            gpio_->write(false);
        }
        return RESTART;
    }

    /// Output pin to blink for activity.
    const Gpio *gpio_;
    /// How many triggers happened since the last run of the timer.
    unsigned triggerCount_ {0};
};

#endif // _UTILS_ACTIVITYLED_HXX_
