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
#include "utils/logging.h"
#include "utils/macros.h"
#include <driver/gpio.h>

/// Helper macro to test if a pin has been configured for output.
///
/// This is necessary since ESP-IDF does not expose gpio_get_direction(pin).
#define IS_GPIO_OUTPUT(pin) (GPIO_IS_VALID_OUTPUT_GPIO(pin) &&                 \
                             (pin < 32) ? GPIO.enable & BIT(pin & 31) :        \
                                          GPIO.enable1.data & BIT(pin & 31))

template <class Defs, bool SAFE_VALUE, bool INVERT> struct GpioOutputPin;
template <class Defs, bool PUEN, bool PDEN> struct GpioInputPin;

/// Defines a GPIO output pin. Writes to this structure will change the output
/// level of the pin. Reads will return the pin's current level.
///
/// The pin is set to output at initialization time, with the level defined by
/// `SAFE_VALUE'.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <gpio_num_t PIN_NUM>
class Esp32Gpio : public Gpio
{
public:
#if defined(CONFIG_IDF_TARGET_ESP32)
    static_assert(PIN_NUM >= 0 && PIN_NUM <= 39, "Valid pin range is 0..39.");
    static_assert(!(PIN_NUM >= 6 && PIN_NUM <= 11)
                , "Pin is reserved for flash usage.");
    static_assert(PIN_NUM != 24, "Pin does not exist");
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
    static_assert(PIN_NUM >= 0 && PIN_NUM <= 46, "Valid pin range is 0..46.");
    static_assert(!(PIN_NUM >= 22 && PIN_NUM <= 24)
                , "Pin does not exist");
    static_assert(!(PIN_NUM >= 26 && PIN_NUM <= 32)
                , "Pin is reserved for flash usage.");
#endif // CONFIG_IDF_TARGET_ESP32

    /// Sets the output state of the connected GPIO pin.
    ///
    /// @param new_state State to set the GPIO pin to.
    void write(Value new_state) const override
    {
        LOG(VERBOSE, "Esp32Gpio(%d) write %s", PIN_NUM,
            new_state == Value::SET ? "SET" : "CLR");
        ESP_ERROR_CHECK(gpio_set_level(PIN_NUM, new_state));
    }

    /// Reads the current state of the connected GPIO pin.
    /// @return @ref SET if currently high, @ref CLR if currently low.
    Value read() const override
    {
        return (Value)gpio_get_level(PIN_NUM);
    }

    /// Sets output to HIGH.
    void set() const override
    {
        write(Value::SET);
    }

    /// Sets output to LOW.
    void clr() const override
    {
        write(Value::CLR);
    }

    /// Sets the direction of the connected GPIO pin.
    void set_direction(Direction dir) const override
    {
        if (dir == Direction::DOUTPUT)
        {
            HASSERT(GPIO_IS_VALID_OUTPUT_GPIO(PIN_NUM));
            // using GPIO_MODE_INPUT_OUTPUT instead of GPIO_MODE_OUTPUT so that
            // we can read the IO state
            ESP_ERROR_CHECK(
                gpio_set_direction(PIN_NUM, GPIO_MODE_INPUT_OUTPUT));
            LOG(VERBOSE, "Esp32Gpio(%d) configured as OUTPUT", PIN_NUM);
        }
        else
        {
            ESP_ERROR_CHECK(gpio_set_direction(PIN_NUM, GPIO_MODE_INPUT));
            LOG(VERBOSE, "Esp32Gpio(%d) configured as INPUT", PIN_NUM);
        }
    }

    /// Gets the GPIO direction.
    /// @return @ref DINPUT or @ref DOUTPUT
    Direction direction() const override
    {
        if (IS_GPIO_OUTPUT(PIN_NUM))
        {
            return Direction::DOUTPUT;
        }
        return Direction::DINPUT;
    }
private:
    template <class Defs, bool SAFE_VALUE, bool INVERT> friend struct GpioOutputPin;
    template <class Defs, bool PUEN, bool PDEN> friend struct GpioInputPin;
    /// Static instance variable that can be used for libraries expecting a
    /// generic Gpio pointer. This instance variable will be initialized by the
    /// linker and (assuming the application developer initialized the hardware
    /// pins in hw_preinit) is accessible, including virtual methods at static
    /// constructor time.
    static const Esp32Gpio instance_;
};

/// Defines the linker symbol for the wrapped Gpio instance.
template <gpio_num_t PIN_NUM>
const Esp32Gpio<PIN_NUM> Esp32Gpio<PIN_NUM>::instance_;

/// Parametric GPIO output class.
/// @param Defs is the GPIO pin's definition base class, supplied by the
/// GPIO_PIN macro.
/// @param SAFE_VALUE is the initial value for the GPIO output pin.
/// @param INVERT inverts the high/low state of the pin when set.
template <class Defs, bool SAFE_VALUE, bool INVERT = false>
struct GpioOutputPin : public Defs
{
public:
    using Defs::PIN_NUM;
// compile time sanity check that the selected pin is valid for output.
#if defined(CONFIG_IDF_TARGET_ESP32)
    static_assert(PIN_NUM < 34, "Pins 34 and above can not be used as output.");
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
    static_assert(PIN_NUM != 46, "Pin 46 can not be used for output.");
#endif // CONFIG_IDF_TARGET_ESP32

    /// Initializes the hardware pin.
    static void hw_init()
    {
        gpio_pad_select_gpio(PIN_NUM);
        ESP_ERROR_CHECK(gpio_reset_pin(PIN_NUM));
        // using GPIO_MODE_INPUT_OUTPUT instead of GPIO_MODE_OUTPUT so that
        // we can read the IO state
        ESP_ERROR_CHECK(
            gpio_set_direction(PIN_NUM, GPIO_MODE_INPUT_OUTPUT));
        ESP_ERROR_CHECK(gpio_set_level(PIN_NUM, SAFE_VALUE));
    }

    /// Sets the hardware pin to a safe value.
    static void hw_set_to_safe()
    {
        ESP_ERROR_CHECK(gpio_set_level(PIN_NUM, SAFE_VALUE));
    }

    /// Sets the output pin @param value if true, output is set to HIGH, if
    /// false, output is set to LOW.
    static void set(bool value)
    {
        if (INVERT)
        {
            ESP_ERROR_CHECK(gpio_set_level(PIN_NUM, !value));
        }
        else
        {
            ESP_ERROR_CHECK(gpio_set_level(PIN_NUM, value));
        }
    }

    /// Toggles the state of the pin to the opposite of what it is currently.
    static void toggle()
    {
        ESP_ERROR_CHECK(gpio_set_level(PIN_NUM, !get()));
    }

    /// Reads the current state of the connected GPIO pin.
    /// @return @ref true if currently high, @ref false if currently low.
    static bool get()
    {
        return gpio_get_level(PIN_NUM);
    }

    /// @return static Gpio object instance that controls this output pin.
    static constexpr const Gpio *instance()
    {
        return &Esp32Gpio<PIN_NUM>::instance_;
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
/// @param Defs is the GPIO pin's definition base class, supplied by the
/// GPIO_PIN macro.
/// @param PUEN is true if the pull-up should be enabled.
/// @param PDEN is true if the pull-down should be enabled.
template <class Defs, bool PUEN, bool PDEN> struct GpioInputPin : public Defs
{
public:
    using Defs::PIN_NUM;
#if defined(CONFIG_IDF_TARGET_ESP32)
    // GPIO 2, 4 and 12 typically have pull-down resistors.
    static_assert(!PUEN ||
                  (PUEN && (PIN_NUM != 2 && PIN_NUM != 4 && PIN_NUM != 12)),
                  "GPIO 2, 4, 12 typically have built-in pull-down resistors, "
                  "enabling pull-up is not possible.");
    // GPIO 0, 5 and 15 typically have pull-up resistors.
    static_assert(!PDEN ||
                  (PDEN && (PIN_NUM != 0 && PIN_NUM != 5 && PIN_NUM == 15)),
                  "GPIO 0, 5, 15 typically have built-in pull-up resistors, "
                  "enabling pull-down is not possible.");
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
    // GPIO 45 and 46 typically have pull-down resistors.
    static_assert(!PUEN || (PUEN && (PIN_NUM != 45 && PIN_NUM != 46)),
                  "GPIO 45 and 46 typically have built-in pull-down "
                  "resistors, enabling pull-up is not possible.");
    // GPIO 0 typically has a pull-up resistor
    static_assert(!PDEN || (PDEN && PIN_NUM != 0),
                  "GPIO 0 typically has a built-in pull-up resistors, "
                  "enabling pull-down is not possible.");
#endif // CONFIG_IDF_TARGET_ESP32
    /// Initializes the hardware pin.
    static void hw_init()
    {
        gpio_pad_select_gpio(PIN_NUM);
        ESP_ERROR_CHECK(gpio_reset_pin(PIN_NUM));
        ESP_ERROR_CHECK(gpio_set_direction(PIN_NUM, GPIO_MODE_INPUT));
        if (PUEN)
        {
            ESP_ERROR_CHECK(gpio_pullup_en(PIN_NUM));
        }
        else
        {
            ESP_ERROR_CHECK(gpio_pullup_dis(PIN_NUM));
        }
        
        if (PDEN)
        {
            ESP_ERROR_CHECK(gpio_pulldown_en(PIN_NUM));
        }
        else
        {
            ESP_ERROR_CHECK(gpio_pulldown_dis(PIN_NUM));
        }
    }
    /// Sets the hardware pin to a safe state.
    static void hw_set_to_safe()
    {
        hw_init();
    }

    /// @return static Gpio object instance that controls this output pin.
    static constexpr const Gpio *instance()
    {
        return &Esp32Gpio<PIN_NUM>::instance_;
    }
};

/// Defines a GPIO input pin with pull-up and pull-down disabled.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs> struct GpioInputNP : public GpioInputPin<Defs, false, false>
{
};

/// Defines a GPIO input pin with pull-up enabled.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs> struct GpioInputPU : public GpioInputPin<Defs, true, false>
{
};

/// Defines a GPIO input pin with pull-down enabled.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs> struct GpioInputPD : public GpioInputPin<Defs, false, true>
{
};

/// Defines a GPIO input pin with pull-up and pull-down enabled.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs> struct GpioInputPUPD : public GpioInputPin<Defs, true, true>
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
/// There are multiple variations available for the ESP32: ESP32, WROVER,
/// WROVER-B, PICO-D4, ESP32-Solo and ESP32-S2. Each of these have slight
/// differences in the available pins.
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
///    - 24      : does not exist.
///    - 37, 38  : not exposed on most modules.
///    - 34 - 39 : these pins are INPUT only.
/// NOTE: ESP32 covers the ESP32-WROOM-32, DOWD, D2WD, S0WD, U4WDH and the
/// ESP32-Solo.
///
/// ESP32-PICO-D4: Same as ESP32 but possibly one restricted pin below:
///    - 16      : when PSRAM is included this pin may be used by flash.
///
/// ESP32-WROVER & WROVER-B: Same as ESP32 but with the following restrictions:
///    - 16, 17  : typically used for PSRAM on WROVER/WROVER-B modules.
///
/// ESP32-S2: Valid pin range is 0..46 with the following restrictions:
///    - 0       : pull-up resistor on most modules.
///    - 19      : USB OTG D- 
///    - 20      : USB OTG D+
///    - 22 - 25 : does not exist.
///    - 26 - 32 : connected to flash (GPIO 26 is used by PSRAM on S2-WROVER).
///    - 43, 44  : UART0, serial console.
///    - 45      : pull-down resistor on most modules.
///    - 46      : pull-down resistor on most modules, also INPUT only.
///
/// The built in pull-up/pull-down resistor for all ESP32 variants is typically
/// around 45 kiloohm.
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
    struct NAME##Defs                                                          \
    {                                                                          \
        static const gpio_num_t PIN_NUM = (gpio_num_t)NUM;                     \
    public:                                                                    \
        static const gpio_num_t pin()                                          \
        {                                                                      \
            return PIN_NUM;                                                    \
        }                                                                      \
    };                                                                         \
    typedef BaseClass<NAME##Defs> NAME##_Pin

#endif // _DRIVERS_ESP32GPIO_HXX_
