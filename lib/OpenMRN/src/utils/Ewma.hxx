/** \copyright
 * Copyright (c) 2014, Balazs Racz
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
 * \file Ewma.hxx
 *
 * Exponentially weighted moving average for computing average transfer speed.
 *
 * @author Balazs Racz
 * @date 2 May 2015
 */

#include <time.h>
#include <stdint.h>

/// Exponentially weighted moving average.
///
/// This class allows an O(1) representation of an average over a timeseries of
/// data. This is the algorithm that Linux is using for the loadavg
/// calculation. The algorithm is parametrized by a coefficient \\alpha. The
/// larger \\alpha is, the longer "memory" the average has, meaning that the
/// slower the average adapts to a changing situation.
///
/// This class is implemented to perform computation of transfer speed. It
/// keeps track of the time since the last call, computes the speed in
/// bytes/sec, and averages the speed values according to the EWMA algorithm.
///
/// This class currently does not work under freertos due to the clock readout
/// mechanism used. To fix it, we'd need to create an API for an equally
/// accurate clock readout mechanism under linux and freertos.
class Ewma
{
public:
    /// Constructor. @param alpha coefficient of the EWMA computation. The
    /// lower the value is the faster the average converges to current
    /// speed. The higher this value is the more the averaging will smoothe the
    /// speed values read out.
    Ewma(float alpha = 0.8)
        : alpha_(alpha)
    {
    }

    /// Sets the absolute value where the transfer is. Sequential calls must
    /// have an increasing value of `offset'. @param offset tells where the
    /// transfer is currently.
    void add_absolute(uint32_t offset)
    {
        add_diff(offset - lastOffset_);
        lastOffset_ = offset;
    }

    /// Notifies the average algorithm that since the last call `bytes'
    /// additional bytes were transferred. @param bytes tell the additional
    /// number of bytes that arrived since the last call.
    void add_diff(uint32_t bytes)
    {
        long long t = current_time();
        if (lastMeasurementTimeNsec_)
        {
            float spd = bytes * 1e9 / (t - lastMeasurementTimeNsec_);
            if (avg_)
            {
                avg_ = avg_ * alpha_ + spd * (1 - alpha_);
            }
            else
            {
                avg_ = spd;
            }
        }
        lastMeasurementTimeNsec_ = t;
    }

    /// @returns the current average speed in bytes/sec.
    float avg()
    {
        return avg_;
    }

private:
    /// Helper function to get the current time. @return current time.
    long long current_time()
    {
        long long t;
#ifdef __linux__
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        t = ts.tv_sec * 1000000000;
        t += ts.tv_nsec;
#else
        t = os_get_time_monotonic();
#endif
        return t;
    }

    float alpha_;    ///< coefficient for EWMA
    float avg_{0.0}; ///< current state of EWMA
    /// When did we take the last measurement.
    long long lastMeasurementTimeNsec_{0};
    /// What was the progress offset at the time of the last measurement taken.
    uint32_t lastOffset_{0};
};

/// Exponentially weighted moving average.
///
/// This class allows an O(1) representation of an average over a timeseries of
/// data. This is the algorithm that Linux is using for the loadavg
/// calculation. The algorithm is parametrized by a coefficient \\alpha. The
/// larger \\alpha is, the longer "memory" the average has, meaning that the
/// slower the average adapts to a changing situation.
///
class AbsEwma
{
public:
    /// @param alpha ewma coefficient.  The lower the value is the faster the
    /// average converges to current speed. The higher this value is the more
    /// the averaging will smoothe the speed values read out.
    AbsEwma(float alpha)
        : alpha_(alpha)
    {
    }

    /// Sets the adjustment parameter. @param alpha the new value of the
    /// adjustment parameter.
    void set_alpha(float alpha)
    {
        alpha_ = alpha;
    }

    /// Adds a step to the EWMA average.
    /// @param value is the currently observed value to add to the average.
    void add_value(float value)
    {
        if (avg_)
        {
            avg_ = avg_ * alpha_ + value * (1 - alpha_);
        }
        else
        {
            avg_ = value;
        }
    }

    /// Clears the history and sets the state of the EWMA.
    /// @param value the desired new average.
    void reset_state(float value)
    {
        avg_ = value;
    }

    /// @return average.
    float avg()
    {
        return avg_;
    }

    float alpha_;    ///< coefficient for EWMA
    float avg_{0.0}; ///< current state of EWMA
};
