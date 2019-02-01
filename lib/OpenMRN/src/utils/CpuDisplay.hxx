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
 * \file CpuDisplay.hxx
 * Displays current CPU load on two LEDs.
 *
 * @author Balazs Racz
 * @date 30 August 2015
 */

#ifndef _UTILS_CPUDISPLAY_HXX_
#define _UTILS_CPUDISPLAY_HXX_

#include "executor/StateFlow.hxx"
#include "freertos_drivers/common/CpuLoad.hxx"

/// Displays the current CPU load on a bicolor LED (red + green) by showing
/// green when CPU load is light, yellow when moderate, and red when heavy.
///
/// Works great for example on the onboard RGB led of the Tiva tm4c123
/// development board.
class CpuDisplay : public StateFlowBase
{
public:
    /// Constructor
    ///
    /// @param s Service specifying which thread to run this stateflow on.
    /// @param red Gpio for the red output pin.
    /// @param green Gpio for the green output pin.
    CpuDisplay(Service *s, const Gpio *red, const Gpio *green)
        : StateFlowBase(s)
        , timer_(this)
        , red_(red)
        , green_(green)
    {
        start_flow(STATE(delay));
    }

    /// Wait for updating the display. @return action.
    Action delay()
    {
        return sleep_and_call(&timer_, MSEC_TO_NSEC(50), STATE(update_display));
    }

    /// Update the display. @return action.
    Action update_display()
    {
        uint8_t load = CpuLoad::instance()->get_load();
        if (load < 20)
        {
            green_->set();
            red_->clr();
        }
        else if (load < 75)
        {
            green_->set();
            red_->set();
        }
        else
        {
            green_->clr();
            red_->set();
        }
        return call_immediately(STATE(delay));
    }

private:
    /// Helper struct for timer state.
    StateFlowTimer timer_;
    /// Red LED for display.
    const Gpio *red_;
    /// Green LED for display.
    const Gpio *green_;
};

#endif // _UTILS_CPUDISPLAY_HXX_
