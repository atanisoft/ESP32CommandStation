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
 * \file DummyGPIO.hxx
 *
 * GPIO-abstraction of a nonexistant pin.
 *
 * @author Balazs Racz
 * @date 21 Jun 2015
 */

#ifndef _FREERTOS_DRIVERS_COMMON_DUMMYGPIO_HXX_
#define _FREERTOS_DRIVERS_COMMON_DUMMYGPIO_HXX_

#include "GpioWrapper.hxx"

/// GPIO Pin definition structure with no actual pin behind it. All writes to
/// this pin will be silently ignored. Reads from this pin will not compile.
struct DummyPin
{
    /// Initializes the hardware to the required settings.
    static void hw_init()
    {
    }
    /// Resets the hardware to a mode that is safe when there is no software
    /// running. This uaully means turning off outputs that consume power.
    static void hw_set_to_safe()
    {
    }
    /// Sets the output pin to a given level.
    static void set(bool value)
    {
    }
    /// Toggles the output pin level.
    static void toggle()
    {
    }
    /// Returns whether this is an output pin or not.
    static bool is_output()
    {
        return true;
    }
    /// Sets to "hardware" function.
    static void set_hw()
    {
    }
    /// Sets to "GPIO out" function.
    static void set_output()
    {
    }
    /// Sets to "GPIO in" function.
    static void set_input()
    {
    }
};

/// GPIO Pin definition structure with no actual pin behind it. All writes to
/// this pin will be silently ignored. Reads will always return false.
struct DummyPinWithRead : public DummyPin
{
    /// @return the input pin level.
    static bool get()
    {
        return false;
    }

    /// @return true if this is an output pin, false if an input pin.
    static bool is_output()
    {
        return false;
    }

    /// @return the static Gpio instance.
    static constexpr const Gpio *instance()
    {
        return GpioWrapper<DummyPinWithRead>::instance();
    }
};

/// GPIO Pin definition structure with no actual pin behind it. All writes to
/// this pin will be silently ignored. Reads will always return false.
struct DummyPinWithReadHigh : public DummyPin
{
    /// @return the input pin level.
    static bool get()
    {
        return true;
    }

    /// @return true if this is an output pin, false if an input pin.
    static bool is_output()
    {
        return false;
    }

    /// @return the static Gpio instance.
    static constexpr const Gpio *instance()
    {
        return GpioWrapper<DummyPinWithReadHigh>::instance();
    }
};

#endif // _FREERTOS_DRIVERS_COMMON_DUMMYGPIO_HXX_
