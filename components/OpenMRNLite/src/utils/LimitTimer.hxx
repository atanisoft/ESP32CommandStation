/** @copyright
 * Copyright (c) 2020, Balazs Racz; 2021 Stuart Baker
 * All rights reserved
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
 * @file LimitTimer.hxx
 *
 * Limits the number of updates per unit time. Initial version written by
 * Balazs Racz, modified for generalization by Stuart Baker
 *
 * @author Balazs Racz, Stuart Baker
 * @date 9 January 2021
 */

#ifndef _UTILS_LIMITTIMER_HXX_
#define _UTILS_LIMITTIMER_HXX_

#include <algorithm>

#include "executor/Timer.hxx"
 
/// This timer takes care of limiting the number of speed updates we send
/// out in a second. It is a token bucket filter.
class LimitTimer : public Timer
{
public:
    /// Constructor.
    /// @param ex executor to run on
    /// @param update_delay_msec cooldown time delay in milliseconds
    /// @param max_tokens number of available tokens, <= 127 max
    /// @param callback callback called once after cooldown time delay
    LimitTimer(ExecutorBase *ex, uint16_t update_delay_msec, uint8_t max_tokens,
               std::function<void()> callback)
        : Timer(ex->active_timers())
        , updateDelayMsec_(update_delay_msec)
        , bucket_(std::min(static_cast<uint8_t>(127), max_tokens))
        , bucketMax_(max_tokens)
        , needUpdate_(false)
        , callback_(callback)
    {
        HASSERT(callback);
    }

    /// Destructor.
    ~LimitTimer()
    {
        cancel();
    }

    /// Attempts to take a token out of the bucket. Must be called from the
    /// same executor that was passed in the object construction.
    /// @return true if the take is successful, false if there are no available
    ///         tokens, in which case there will be a callback generated when
    ///         tokens become available.
    bool try_take()
    {
        if (bucket_ == bucketMax_)
        {
            start(MSEC_TO_NSEC(updateDelayMsec_));
        }
        if (bucket_ > 0)
        {
            --bucket_;
            return true;
        }
        else
        {
            needUpdate_ = true;
            return false;
        }
    }

    /// Takes one entry from the bucket, and does not give a callback if
    /// there is no entry left.  Must be called from the
    /// same executor that was passed in the object construction.
    void take_no_callback()
    {
        if (bucket_ > 0)
        {
            --bucket_;
        }
    }

private:
    /// Callback from the timer infrastructure. Called periodically.
    long long timeout() override
    {
        ++bucket_;
        if (needUpdate_)
        {
            needUpdate_ = false;
            callback_();
        }
        if (bucket_ >= bucketMax_)
        {
            return NONE;
        }
        else
        {
            return RESTART;
        }
    }

    /// cooldown delay in msec
    uint16_t updateDelayMsec_;

    /// number of available tokens
    uint8_t bucket_ ;

    /// maximum number of tokens in the bucket
    uint8_t bucketMax_ : 7;

    /// if non-zero, wake up parent when token is available.
    uint8_t needUpdate_ : 1;

    /// callback after cooldown period.
    std::function<void()> callback_;
};

#endif // _UTILS_LIMITTIMER_HXX_