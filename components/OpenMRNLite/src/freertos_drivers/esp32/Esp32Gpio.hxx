/** \copyright
 * Copyright (c) 2020, Mike Dunston
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
 * \file Esp32Gpio.hxx
 *
 * Helper declarations for using GPIO pins via the ESP-IDF APIs.
 *
 * @author Mike Dunston
 * @date 27 March 2020
 */

#ifndef _DRIVERS_ESP32GPIO_HXX_
#define _DRIVERS_ESP32GPIO_HXX_

#include "freertos_drivers/arduino/GpioWrapper.hxx"
#include "os/Gpio.hxx"
#include "utils/macros.h"
#include <driver/gpio.h>

/// Defines a GPIO output pin. Writes to this structure will change the output
/// level of the pin. Reads will return the pin's current level.
///
/// The pin is set to output at initialization time, with the level defined by
/// `SAFE_VALUE'.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <uint8_t PIN_NUM>
class Esp32Gpio
{
public:
#if defined(CONFIG_IDF_TARGET_ESP32)
    static_assert(PIN_NUM >= 0 && PIN_NUM <= 39, "Valid pin range is 0..39.");
    static_assert(!(PIN_NUM >= 6 && PIN_NUM <= 11)
                , "Pin is reserved for flash usage.");
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
    static_assert(PIN_NUM >= 0 && PIN_NUM <= 46, "Valid pin range is 0..46.");
    static_assert(!(PIN_NUM >= 26 && PIN_NUM <= 32)
                , "Pin is reserved for flash usage.");
#endif // CONFIG_IDF_TARGET_ESP32

    /// @return the pin number for this GPIO.
    static gpio_num_t pin()
    {
        return (gpio_num_t)PIN_NUM;
    }

    /// Sets pin to output.
    static void set_output()
    {
        HASSERT(GPIO_IS_VALID_OUTPUT_GPIO(PIN_NUM));
        ESP_ERROR_CHECK(gpio_set_direction(pin(), GPIO_MODE_OUTPUT));
    }

    /// Sets pin to input.
    static void set_input()
    {
        ESP_ERROR_CHECK(gpio_set_direction(pin(), GPIO_MODE_INPUT));
    }

    /// Turns on pullup.
    static void set_pullup_on()
    {
#if defined(CONFIG_IDF_TARGET_ESP32)
        // these pins have HW PD always.
        HASSERT(PIN_NUM != 12);
        HASSERT(PIN_NUM != 4);
        HASSERT(PIN_NUM != 2);
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
        // these pins have HW PD always.
        HASSERT(PIN_NUM != 45);
        HASSERT(PIN_NUM != 46);
#endif // CONFIG_IDF_TARGET_ESP32
        ESP_ERROR_CHECK(gpio_pullup_en(pin()));
    }

    /// Turns off pullup.
    static void set_pullup_off()
    {
#if defined(CONFIG_IDF_TARGET_ESP32)
        // these pins have HW PU always.
        HASSERT(PIN_NUM != 0);
        HASSERT(PIN_NUM != 15);
        HASSERT(PIN_NUM != 5);
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
        // these pins have HW PU always.
        HASSERT(PIN_NUM != 0);
#endif // CONFIG_IDF_TARGET_ESP32

        ESP_ERROR_CHECK(gpio_pullup_dis(pin()));
    }

    /// Turns on pullup.
    static void set_pulldown_on()
    {
#if defined(CONFIG_IDF_TARGET_ESP32)
        // these pins have HW PU always.
        HASSERT(PIN_NUM != 0);
        HASSERT(PIN_NUM != 15);
        HASSERT(PIN_NUM != 5);
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
        // these pins have HW PU always.
        HASSERT(PIN_NUM != 0);
#endif // CONFIG_IDF_TARGET_ESP32

        ESP_ERROR_CHECK(gpio_pulldown_en(pin()));
    }

    /// Turns off pullup.
    static void set_pulldown_off()
    {
#if defined(CONFIG_IDF_TARGET_ESP32)
        // these pins have HW PD always.
        HASSERT(PIN_NUM != 12);
        HASSERT(PIN_NUM != 4);
        HASSERT(PIN_NUM != 2);
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
        // these pins have HW PD always.
        HASSERT(PIN_NUM != 45);
        HASSERT(PIN_NUM != 46);
#endif // CONFIG_IDF_TARGET_ESP32

        ESP_ERROR_CHECK(gpio_pulldown_dis(pin()));
    }

    /// Sets output to HIGH.
    static void set_on()
    {
        ESP_ERROR_CHECK(gpio_set_level(pin(), 1));
    }

    /// Sets output to LOW.
    static void set_off()
    {
        ESP_ERROR_CHECK(gpio_set_level(pin(), 0));
    }

    /// @return input pin level.
    static bool get()
    {
        // If the pin is configured as an output we can not use the ESP-IDF API
        // gpio_get_level(pin) since it will return LOW for any output pin. To
        // work around this API limitation it is necessary to read directly
        // from the memory mapped GPIO registers.
        if (is_output())
        {
            if (PIN_NUM < 32)
            {
                return GPIO.out & BIT(PIN_NUM & 31);
            }
            else
            {
                return GPIO.out1.data & BIT(PIN_NUM & 31);
            }
        }
        return gpio_get_level(pin());
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
        // If the pin is a valid output, check if it is configured for output.
        // Unfortunately, ESP-IDF does not provide gpio_get_direction(pin) but
        // does provide gpio_set_direction(pin, direction). To workaround this
        // limitation it is necessary to read directly from the memory mapped
        // GPIO registers.
        if (GPIO_IS_VALID_OUTPUT_GPIO(PIN_NUM))
        {
            // pins 32 and below use the first GPIO controller
            if (PIN_NUM < 32)
            {
                return GPIO.enable & BIT(PIN_NUM & 31);
            }
            else
            {
                return GPIO.enable1.data & BIT(PIN_NUM & 31);
            }
        }
        return false;
    }

    /// Initializes the underlying hardware pin on the ESP32.
    static void hw_init()
    {
        // sanity check that the pin number is valid
        HASSERT(GPIO_IS_VALID_GPIO(PIN_NUM));

        // configure the pad for GPIO function
        gpio_pad_select_gpio(pin());

        // reset the pin configuration to defaults
        ESP_ERROR_CHECK(gpio_reset_pin(pin()));
    }

    /// @return an os-indepentent Gpio abstraction instance for use in
    /// libraries.
    static constexpr const Gpio *instance()
    {
        return GpioWrapper<Esp32Gpio<PIN_NUM>>::instance();
    }
};

/// Parametric GPIO output class.
/// @param Base is the GPIO pin's definition base class, supplied by the
/// GPIO_PIN macro.
/// @param SAFE_VALUE is the initial value for the GPIO output pin.
/// @param INVERT inverts the high/low state of the pin when set.
template <class Base, bool SAFE_VALUE, bool INVERT = false>
struct GpioOutputPin : public Base
{
public:
    /// Initializes the hardware pin.
    static void hw_init()
    {
        HASSERT(GPIO_IS_VALID_OUTPUT_GPIO(Base::pin()));
        Base::hw_init();
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
/// @param PUEN is true if the pull-up should be enabled.
/// @param PDEN is true if the pull-down should be enabled.
template <class Base, bool PUEN, bool PDEN> struct GpioInputPar : public Base
{
public:
    /// Initializes the hardware pin.
    static void hw_init()
    {
        Base::hw_init();
        Base::set_input();
        if (PUEN)
        {
            Base::set_pullup_on();
        }
        else
        {
            Base::set_pullup_off();
        }
        
        if (PDEN)
        {
            Base::set_pulldown_on();
        }
        else
        {
            Base::set_pulldown_off();
        }
    }
    /// Sets the hardware pin to a safe state.
    static void hw_set_to_safe()
    {
        hw_init();
    }
};

/// Defines a GPIO input pin  with pull-up and pull-down disabled.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs> struct GpioInputNP : public GpioInputPar<Defs, false, false>
{
};

/// Defines a GPIO input pin with pull-up enabled.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs> struct GpioInputPU : public GpioInputPar<Defs, true, false>
{
};

/// Defines a GPIO input pin with pull-down enabled.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs> struct GpioInputPD : public GpioInputPar<Defs, false, true>
{
};

/// Defines a GPIO input pin with pull-up and pull-down enabled.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs> struct GpioInputPUPD : public GpioInputPar<Defs, true, true>
{
};

/// Helper macro for defining GPIO pins on the ESP32. 
///
/// @param NAME is the basename of the declaration. For NAME==FOO the macro
/// declared FOO_Pin as a structure on which the read-write functions will be
/// available.
///
/// @param BaseClass is the initialization structure, such as @ref LedPin, or
/// @ref GpioOutputSafeHigh or @ref GpioOutputSafeLow.
///
/// @param NUM is the pin number, such as 3 (see below for usable range).
///
/// The ESP32 (includes: WROVER, WROVER-B, PICO-D4) and ESP32-S2 modules have
/// some differences in behavior with GPIO pins and usages as documented below.
///
/// ESP32: Valid pin range is 0..39 with the following restrictions:
///    - 0       : pull-up resistor on most modules.
///    - 2       : pull-down resistor on most modules.
///    - 1, 3    : UART0, serial console.
///    - 4       : pull-down resistor on most modules.
///    - 5       : pull-up resistor on most modules.
///    - 6 - 11  : connected to flash.
///    - 12      : pull-down resistor on most modules.
///    - 15      : pull-up resistor on most modules.
///    - 16, 17  : used for PSRAM on WROVER/WROVER-B modules.
///    - 37, 38  : not exposed on most modules.
///    - 34 - 39 : these pins are INPUT only.
///
/// ESP32-S2: Valid pin range is 0..46 with the following restrictions:
///    - 0       : pull-up resistor on most modules.
///    - 22 - 25 : does not exist.
///    - 26 - 32 : connected to flash (GPIO 26 is used by PSRAM on S2-WROVER).
///    - 43, 44  : UART0, serial console.
///    - 45      : pull-down resistor on most modules.
///    - 46      : pull-down resistor on most modules, also INPUT only.
///
/// Data sheet references:
/// ESP32: https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf
/// ESP32-WROVER: https://www.espressif.com/sites/default/files/documentation/esp32-wrover_datasheet_en.pdf
/// ESP32-WROVER-B: https://www.espressif.com/sites/default/files/documentation/esp32-wrover-b_datasheet_en.pdf
/// ESP32-PICO-D4: https://www.espressif.com/sites/default/files/documentation/esp32-pico-d4_datasheet_en.pdf
/// ESP32-S2: https://www.espressif.com/sites/default/files/documentation/esp32-s2_datasheet_en.pdf
/// ESP32-S2-WROVER: https://www.espressif.com/sites/default/files/documentation/esp32-s2-wrover_esp32-s2-wrover-i_datasheet_en.pdf
///
/// Example:
///  GPIO_PIN(FOO, GpioOutputSafeLow, 3);
///  ...
///  FOO_Pin::set(true);
#define GPIO_PIN(NAME, BaseClass, NUM)                                         \
    typedef BaseClass<Esp32Gpio<NUM>> NAME##_Pin

#endif // _DRIVERS_ESP32GPIO_HXX_
