/** \copyright
 *
 *    Copyright (C) 2018  Robert Heller D/B/A Deepwoods Software
 *			51 Locke Hill Road
 *			Wendell, MA 01379-9728
 *
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
 *
 * \file LinuxGpio.hxx
 *
 * Defines GPIO pins using the Linux sysfs ABI.
 * 
 * \section HOWTOUSE How to use
 * 
 * You need to use the GPIO_PIN macro at the bottom, like this:
 * 
 * GPIO_PIN(LED1, GpioOutputSafeLow, 27);  // Defines LED1_Pin, for GPIO 27, initialized low. 
 * GPIO_PIN(CSLow, GpioOutputSafeHighInvert, 5);  // Defines CSLow_Pin, for GPIO 5, initialized high, with the set logic inverted. 
 * GPIO_PIN(Button1, GpioInputActiveLow, 20);  // Defines Button1_Pin, for GPIO 20, avtive low -- return true when shorted to ground. 
 *
 * Classes available for the second macro parameter are:
 * 
 *   - GpioOutputSafeLow        Output initialized low, true = high
 *   - GpioOutputSafeLowInvert  Output initialized low, true = low
 *   - GpioOutputSafeHigh       Output initialized high, true = high
 *   - GpioOutputSafeHighInvert Output initialized high, true = high
 *   - GpioInputActiveHigh      Input, high = true
 *   - GpioInputActiveLow       Input, low  = true
 * 
 * Be sure to use GpioInitializer to create an Initializer class:
 * 
 * typedef GpioInitializer<LED1_Pin, CSLow_Pin, Button1_Pin> GpioInit;
 * 
 * somewhere in main.cxx, and then in appl_main():
 * 
 * GpioInit::hw_init();
 * 
 * This makes sure the GPIO pins are properly set up (eg exported to /sys/class/gpio/).
 * Also, the process running the node needs to be in group gpio.
 * 
 * @author Robert Heller
 * @date 10 October 2018
 */

#ifndef __LINUXGPIO_HXX
#define __LINUXGPIO_HXX

#include "freertos_drivers/common/GpioWrapper.hxx"
#include "os/Gpio.hxx"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

/// Defines a GPIO output pin. Writes to this structure will change the output
/// level of the pin. Reads will return the pin's current level.
/// Uses Linux sysfs ABI
///
/// The pin is set to output at initialization time, with the level defined by
/// `SAFE_VALUE'.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <int PIN_NUM> class LinuxGpio
{
public:
    /// Number of the pin
    static constexpr const uint32_t PIN = PIN_NUM;
    
    /// Export pin
    static void export_pin()
    {
        FILE *fp = fopen("/sys/class/gpio/export","w");
        fprintf(fp,"%d\n",PIN);
        fclose(fp);
        // 50ms delay IS needed while kernel changes ownership of created GPIO directory
        usleep(50000); 
    }
    
    /// Sets pin to output.
    static void set_output()
    {
        char dirname[40];
        snprintf(dirname,sizeof(dirname),"/sys/class/gpio/gpio%d/direction",PIN);
        int dfd = -1;
        while ((dfd = open(dirname,O_WRONLY)) < 0) {
            export_pin();
        }
        write(dfd,"out\n",4);
        close(dfd);
    }
    /// Sets pin to input.
    static void set_input()
    {
        char dirname[40];
        snprintf(dirname,sizeof(dirname),"/sys/class/gpio/gpio%d/direction",PIN);
        int dfd = -1;
        while ((dfd = open(dirname,O_WRONLY)) < 0) {
            export_pin();
        }
        write(dfd,"in\n",3);
        close(dfd);
    }
    /// Sets output to HIGH.
    static void set_on()
    {
        char valname[40];
        snprintf(valname,sizeof(valname),"/sys/class/gpio/gpio%d/value",PIN);
        int vfd = open(valname,O_WRONLY);
        write(vfd,"1\n",2);
        close(vfd);
    }
    /// Sets output to LOW.
    static void set_off()
    {
        char valname[40];
        snprintf(valname,sizeof(valname),"/sys/class/gpio/gpio%d/value",PIN);
        int vfd = open(valname,O_WRONLY);
        write(vfd,"0\n",2);
        close(vfd);
    }
    /// @return input pin level.
    static bool get()
    {
        char valname[40], c;
        snprintf(valname,sizeof(valname),"/sys/class/gpio/gpio%d/value",PIN);
        int vfd = open(valname,O_RDONLY);
        read(vfd,&c,1);
        close(vfd);
        return (c != '0');
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
        char dirname[40], c;
        snprintf(dirname,sizeof(dirname),"/sys/class/gpio/gpio%d/direction",PIN);
        int dfd = open(dirname,O_RDONLY);
        read(dfd,&c,1);
        close(dfd);
        return (c == 'o');
    }
};


/// Generic output pin
template <class Base, bool SAFE_VALUE, bool INVERT = false> 
struct GpioOutputPin : public Base
{
public:
    /// Initializes the hardware pin.
    static void hw_init()
    {
        Base::export_pin();
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
    /// @return the static Gpio instance.
    static constexpr const Gpio *instance()
    {
        return GpioWrapper<GpioOutputPin<Base,SAFE_VALUE,INVERT>>::instance();
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
/// Generic input pin
template <class Base, bool ACTIVE_HIGH = true> 
struct GpioInputPin : public Base
{
public:
    /// Initializes the hardware pin.
    static void hw_init()
    {
        Base::export_pin();
        Base::set_input();
    }
    /// Sets the hardware pin to a safe state. 
    static void hw_set_to_safe()
    {
        hw_init();
    }
    /// @return the static Gpio instance.
    static const Gpio *instance()
    {
        return GpioWrapper<GpioInputPin<Base>>::instance();
    }
    static bool get()
    {
        if (ACTIVE_HIGH)
        {
            return Base::get();
        }
        else
        {
            return !Base::get();
        }
    }
};

/// Defines a GPIO input pin, Active High (High == true).
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs>
struct GpioInputActiveHigh : public GpioInputPin<Defs, true>
{
};

/// Defines a GPIO input pin, Active Low (Low == true).
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs>
struct GpioInputActiveLow : public GpioInputPin<Defs, false>
{
};



/// Helper macro for defining GPIO pins on Linux-based microcontrollers (like the Raspberry Pi or Bagle Bone.
///
/// @param NAME is the basename of the declaration. For NAME==FOO the macro
/// declared FOO_Pin as a structure on which the read-write functions will be
/// available.
///
/// @param BaseClass is the initialization structure, such as @ref LedPin, or
/// @ref GpioOutputSafeHigh or @ref GpioOutputSafeLow.
///
/// @param NUM is the pin number, such as 3 
///
/// Example:
///  GPIO_PIN(FOO, GpioOutputSafeLow, 3);
///  ...
///  FOO_Pin::set(true);
#define GPIO_PIN(NAME, BaseClass, NUM) \
    typedef BaseClass<LinuxGpio<NUM>> NAME##_Pin

#endif // __LINUXGPIO_HXX

