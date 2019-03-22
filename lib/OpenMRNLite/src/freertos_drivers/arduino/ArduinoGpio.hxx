/** \copyright
 * Copyright (c) 2019, Mike Dunston
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
 * \file ArduinoGpio.hxx
 *
 * Helper declarations for using GPIO pins via the Arduino APIs.
 *
 * @author Mike Dunston
 * @date 14 January 2019
 */

#ifndef _DRIVERS_ARDUINOGPIO_HXX_
#define _DRIVERS_ARDUINOGPIO_HXX_

#include "os/Gpio.hxx"
#include "GpioWrapper.hxx"
#if defined(ESP32)
#include <esp32-hal.h>
#include <driver/gpio.h>
#else
#include <Arduino.h>
#endif

/// Defines a GPIO output pin. Writes to this structure will change the output
/// level of the pin. Reads will return the pin's current level.
///
/// The pin is set to output at initialization time, with the level defined by
/// `SAFE_VALUE'.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <int PIN_NUM>
class ArduinoStaticGpio
{
public:
    /// Sets pin to output.
    static void set_output()
    {
        pinMode(PIN_NUM, OUTPUT);
    }

    /// Sets pin to input.
    static void set_input()
    {
        pinMode(PIN_NUM, INPUT);
    }

    /// Turns on pullup.
    static void set_pullup_on()
    {
        digitalWrite(PIN_NUM, HIGH);
    }

    /// Turns off pullup.
    static void set_pullup_off()
    {
        digitalWrite(PIN_NUM, LOW);
    }

    /// Sets output to HIGH.
    static void set_on()
    {
        digitalWrite(PIN_NUM, HIGH);
    }

    /// Sets output to LOW.
    static void set_off()
    {
        digitalWrite(PIN_NUM, LOW);
    }

    /// @return input pin level.
    static bool get()
    {
        return digitalRead(PIN_NUM);
    }

    /// Set output pin level. @param value is the level to set to.
    static void set(bool value)
    {
        if (value)
        {
            set_on();
        }
        else
        {
            set_off();
        }
    }

    /// Toggles output pin value.
    static void toggle()
    {
        set(!get());
    }

    /// @return true if pin is configured as an output pin.
    static bool is_output()
    {
#if defined(ESP32)
        if(digitalPinIsValid(PIN_NUM) && digitalPinCanOutput(PIN_NUM))
        {
            // pins 32 and below use the first GPIO controller
            if(PIN_NUM < 32)
            {
                return GPIO.enable_w1ts & ((uint32_t)1 << (PIN_NUM & 31));
            }
            else
            {
                return GPIO.enable1_w1ts.val & ((uint32_t)1 << (PIN_NUM & 31));
            }
        }
#endif
        return false;
    }

    /// @return an os-indepentent Gpio abstraction instance for use in
    /// libraries.
    static constexpr const Gpio *instance()
    {
        return GpioWrapper<ArduinoStaticGpio<PIN_NUM>>::instance();
    }
};

template <class Base, bool SAFE_VALUE, bool INVERT = false>
struct GpioOutputPin : public Base
{
public:
    /// Initializes the hardware pin.
    static void hw_init()
    {
        Base::set(SAFE_VALUE);
        Base::set_output();
        Base::set(SAFE_VALUE);
    }
    /// Sets the hardware pin to a safe value.
    static void hw_set_to_safe()
    {
        Base::set(SAFE_VALUE);
    }
    /// Sets the output pinm @param value if true, output is set to HIGH, if
    /// false, output is set to LOW.
    static void set(bool value)
    {
        if (INVERT)
        {
            Base::set(!value);
        }
        else
        {
            Base::set(value);
        }
    }
};

/// Defines a GPIO output pin, initialized to be an output pin with low level.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs>
struct GpioOutputSafeLow : public GpioOutputPin<Defs, false>
{
};

/// Defines a GPIO output pin, initialized to be an output pin with low
/// level. All set() commands are acted upon by inverting the value.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs>
struct GpioOutputSafeLowInvert : public GpioOutputPin<Defs, false, true>
{
};

/// Defines a GPIO output pin, initialized to be an output pin with high level.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs>
struct GpioOutputSafeHigh : public GpioOutputPin<Defs, true>
{
};

/// Defines a GPIO output pin, initialized to be an output pin with high
/// level. All set() commands are acted upon by inverting the value.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs>
struct GpioOutputSafeHighInvert : public GpioOutputPin<Defs, true, true>
{
};

/// Parametric GPIO input class.
/// @param Base is the GPIO pin's definition base class, supplied by the
/// GPIO_PIN macro.
/// @param PUEN is true if the pullup should be enabled.
template <class Base, bool PUEN> struct GpioInputPar : public Base
{
public:
    /// Initializes the hardware pin.
    static void hw_init()
    {
        Base::set_input();
        if (PUEN)
        {
            Base::set_pullup_on();
        }
        else
        {
            Base::set_pullup_off();
        }
    }
    /// Sets the hardware pin to a safe state.
    static void hw_set_to_safe()
    {
        hw_init();
    }
};

/// Defines a GPIO input pin. No pull-up.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs> struct GpioInputNP : public GpioInputPar<Defs, false>
{
};

/// Defines a GPIO input pin with pull-up.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs> struct GpioInputPU : public GpioInputPar<Defs, true>
{
};

/// Helper macro for defining GPIO pins with Arduino 
///
/// @param NAME is the basename of the declaration. For NAME==FOO the macro
/// declared FOO_Pin as a structure on which the read-write functions will be
/// available.
///
/// @param BaseClass is the initialization structure, such as @ref LedPin, or
/// @ref GpioOutputSafeHigh or @ref GpioOutputSafeLow.
///
/// @param NUM is the pin number, such as 3 (range: 0..15; GPIO16 is not
/// supported)
///
/// Example:
///  GPIO_PIN(FOO, GpioOutputSafeLow, 3);
///  ...
///  FOO_Pin::set(true);
#define GPIO_PIN(NAME, BaseClass, NUM)                                         \
    typedef BaseClass<ArduinoStaticGpio<NUM>> NAME##_Pin

#endif // _DRIVERS_ARDUINOGPIO_HXX_
