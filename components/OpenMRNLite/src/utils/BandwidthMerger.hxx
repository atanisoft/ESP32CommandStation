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
 * \file BandwidthMerger.hxx
 *
 * Simple stride scheduler for splitting bandwidth.
 *
 * @author Balazs Racz
 * @date 10 Oct 2020
 */

#ifndef _UTILS_BANDWIDTHMERGER_HXX_
#define _UTILS_BANDWIDTHMERGER_HXX_

#include <inttypes.h>

#include "utils/macros.h"

/// Simple stride scheduler. Emulates a two-way split of bandwidth between a
/// first source and a second source. The percentage assigned to the first
/// source is a parameter. Over time the fraction of outputs selected from the
/// first source converges to this percentage.
struct BandwidthMerger
{
    /// Constructor.
    /// @param percent is the percentage (0..100) of the bandwidth that
    /// should be assigned to the first source.
    BandwidthMerger(uint8_t percent)
        : percentFirst_(percent)
    {
        HASSERT(percentFirst_ <= 100);
    }
    /// Runs one step.
    /// @return true if the first source is selected in this step.
    bool step()
    {
        currentState_ += percentFirst_;
        if (currentState_ >= 100)
        {
            currentState_ -= 100;
            return true;
        }
        return false;
    }

    /// If the step function returned false, but still the first source was
    /// taken (e.g. because the second source was empty), call this function to
    /// clear the state. This will avoid selecting the first source twice in a
    /// row for example.
    void reset()
    {
        // Reduces the state by 100 but clips it to zero. Since the state is
        // always < 100, the clipping will always win.
        currentState_ = 0;
    }

    /// State of the current stride. This is always between 0..99. It
    /// represents the fractional steps that the first source has accumulated
    /// but not paid out in the form of selections yet.
    uint8_t currentState_ {0};
    /// Percentage of the bandwidth that should be assigned to the first
    /// source. Range 0..100.
    uint8_t percentFirst_;
};

#endif // _UTILS_BANDWIDTHMERGER_HXX_
