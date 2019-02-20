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
 * \file GpioWrapper.hxx
 *
 * Templated helper class to wrap a static Gpio structure into an
 * implementation of the os-unspecific gpio class.
 *
 * @author Balazs Racz
 * @date 5 Jul 2015
 */

#ifndef _FREERTOS_DRIVERS_COMMON_GPIOWRAPPER_HXX_
#define _FREERTOS_DRIVERS_COMMON_GPIOWRAPPER_HXX_

#include "os/Gpio.hxx"

/// Creates an implementation of an os-independent Gpio object from a
/// hardware-specific static Gpio structure. Does not permit changing the
/// direction of the hardware pin. For an example usage, see @ref
/// BLINKER_Pin::instance() in @ref BlinkerGPIO.hxx.
template <class PIN> class GpioWrapper : public Gpio
{
public:
    /// This constructor is constexpr which ensures that the object can be
    /// initialized in the data section.
    constexpr GpioWrapper()
    {
    }

    void write(Value new_state) const OVERRIDE
    {
        PIN::set(new_state);
    }

    void set() const OVERRIDE
    {
        PIN::set(true);
    }

    void clr() const OVERRIDE
    {
        PIN::set(false);
    }

    Value read() const OVERRIDE
    {
        return PIN::get() ? VHIGH : VLOW;
    }

    void set_direction(Direction dir) const OVERRIDE
    {
        // We cannot change the direction of a wrapped pin. Crash if the code
        // attempts to do so.
        if (dir == Direction::DOUTPUT)
        {
            HASSERT(PIN::is_output());
        }
        else
        {
            HASSERT(!PIN::is_output());
        }
    }

    Direction direction() const OVERRIDE
    {
        if (PIN::is_output())
        {
            return Direction::DOUTPUT;
        }
        else
        {
            return Direction::DINPUT;
        }
    }

    /// @return the static Gpio object instance controlling this pin.
    static constexpr const Gpio *instance()
    {
        return &instance_;
    }

    static const GpioWrapper instance_;
};

/// Defines the linker symbol for the wrapped Gpio instance.
template <class PIN> const GpioWrapper<PIN> GpioWrapper<PIN>::instance_;

/// Creates a hardware GPIO adaptor that inverts the pin state.
template<class PIN> class InvertedGpio {
public:
    static void hw_init() { PIN::hw_init(); }
    static void hw_set_to_safe() { PIN::hw_set_to_safe(); }

    static void set(bool value) { PIN::set(!value); }
    static bool get() { return !PIN::get(); }

    static void toggle() { PIN::toggle(); }
    static bool is_output() { return PIN::is_output(); }
    static void set_output(bool o) { PIN::set_output(o); }
    
    /// @return the static Gpio object instance controlling this pin.
    static constexpr const Gpio *instance()
    {
        return GpioWrapper<InvertedGpio<PIN>>::instance();
    }
};


#endif // _FREERTOS_DRIVERS_COMMON_GPIOWRAPPER_HXX_
