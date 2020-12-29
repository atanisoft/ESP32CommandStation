/** \copyright
 * Copyright (c) 2020, Balazs Racz
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
 * \file LruCounter.hxx
 *
 * A monotonic counter that is usable for approximate LRU age determination.
 *
 * @author Balazs Racz
 * @date 18 Sep 2020
 */

#ifndef _UTILS_LRUCOUNTER_HXX_
#define _UTILS_LRUCOUNTER_HXX_

#include <limits>

template <class T> class LruCounter;

/// The GlobalLruCounter and a set of LruCounter<> objects cooperate in order
/// to create an approximate LRU order over a set of objects. The particular
/// optimization criterion is that the memory storage per object should be very
/// low. (Target is one byte.) Touching an object is constant time, but there
/// is linear time background maintenance operations that need to run
/// regularly. Picking the oldest object is linear time. The oldest concept is
/// an approximation in that the older an object becomes the less time
/// granularity is available to distinguish exact age. This is generally fine
/// in applications.
///
/// How to use:
///
/// Create one GlobalLruCounter. Create for each tracked object an
/// LruCounter<uint8_t> or LruCounter<uint16_t>.
///
/// Periodically call the tick() function once on the GlobalLruCounter, then
/// for each live object the tick(global) function. This is linear cost in the
/// number of tracked objects, so do it rather rarely (e.g. once per second).
///
/// When a specific object is used, call the touch() function on it.
///
/// When the oldest object needs to be selected, pick the one which has the
/// highest returned value() from its LruCounter<>.
///
/// Theory of operation:
///
/// The GlobalLruCounter maintains a global tick count. It gets incremented by
/// one in each tick. In the per-object local counter we only increment the
/// counter for a subset of the global ticks. How many global ticks we skip
/// between two local counter increments depends on the age of the object. The
/// older an object becomes the more rarely we increment the object's counter.
///
/// Specifically, if the object counter has reached to be k bits long, then we
/// only increment it, when the global counter's bottom k bits are all
/// zero. Example: if the object counter is 35 (6 bits long), then we increment
/// it to 36 when the global counter is divisible by 64 (all 6 bottom bits are
/// zero). In a variant we double the zero-bits requirement, needing that the
/// bottom 12 bits are all zero.
///
/// Example calculations, assuming 1 tick per second:
///
/// +-------------------------------------------------------------------+
/// |  Exponent            1 bit/bit                 2 bits/bit         |
/// +------------+------------------------------------------------------+
/// | data type: |                                                      |
/// |            |                                                      |
/// | uint8_t    |    max count: ~43k             max count: 9.5M       |
/// |            |        (0.5 days)                  (110 days)        |
/// |            |    end granularity: 256        end granularity: 64k  |
/// |            |        (4 min)                     (0.5 days)        |
/// +------------+------------------------------------------------------+
/// | uint16_t   |    max count: ~2.8B             max count: 161T      |
/// |            |        (100 years)                 (5M years)        |
/// |            |    end granularity: 64k        end granularity: 4B   |
/// |            |        (0.5 days)                  (136 years)       |
/// +------------+------------------------------------------------------+
class GlobalLruCounter
{
public:
    /// Constructor.
    /// @param bits_per_bit How aggressive the exponential downsampling should
    /// be. Meaningful values are 1 and 2.
    GlobalLruCounter(unsigned bits_per_bit = 2)
        : bitsPerBit_(bits_per_bit)
    {
    }
    void tick()
    {
        ++tick_;
    }

private:
    template <class T> friend class LruCounter;
    /// Setting defining the exponent.
    unsigned bitsPerBit_;
    /// Rolling counter of global ticks. This is used by the local counters to
    /// synchronize their increments.
    unsigned tick_ {0};
};

/// Create an instance of this type for each object whose age needs to be
/// measured with the GlobalLruCounter. For further details, see
/// { \link GlobalLruCounter }.
/// @param T is the storage type, typically uint8_t or uint16_t.
template <class T> class LruCounter
{
public:
    /// @return A value monotonic in the age of the current counter.
    unsigned value()
    {
        return counter_;
    }

    /// Increments the local counter.
    /// @param global reference to the global tick counter. All calls must use
    /// the same global counter.
    void tick(const GlobalLruCounter &global)
    {
        if (!counter_)
        {
            ++counter_;
            return;
        }
        if (counter_ == std::numeric_limits<T>::max())
        {
            // Counter is saturated.
            return;
        }
        int nlz = __builtin_clz((unsigned)counter_);
        int needzero = (32 - nlz) * global.bitsPerBit_;
        if ((global.tick_ & ((1U << needzero) - 1)) == 0)
        {
            ++counter_;
        }
    }

    /// Signals that the object has been used now.
    void touch()
    {
        counter_ = 0;
    }

private:
    /// Internal counter.
    T counter_ {0};
};

#endif // _UTILS_LRUCOUNTER_HXX_
