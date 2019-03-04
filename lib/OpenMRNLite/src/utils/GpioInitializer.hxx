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
 * \file GpioInitializer.hxx
 *
 * Helper class to call the initialization methods on all GPIO pins.
 *
 * @author Balazs Racz
 * @date 5 Jul 2015
 */

#ifndef _UTILS_GPIOINITIALIZER_HXX_
#define _UTILS_GPIOINITIALIZER_HXX_

/// Forward declaration to make the template matcher happy.
template <class T> struct GpioInitHelper;

/// Partial template specialization to end the type recursion.
template <> struct GpioInitHelper<std::tuple<>>
{
    /// hw_init part of the end of the recursion.
    static void hw_init()
    {
    }
    /// hw_set_to_safe part of the end of the recursion.
    static void hw_set_to_safe()
    {
    }
    /// set part of the end of the recursion.
    static void set(bool value)
    {
    }
};

/// General recursion step. Calls the hw_init / hw_set_to_safe function on the
/// first type argument, and then recurses to the rest of the types.
template <typename Head, typename... Tail>
struct GpioInitHelper<std::tuple<Head, Tail...>>
{
    /// Middle step of the recursion for an individual pin.
    static void hw_init()
    {
        Head::hw_init();
        GpioInitHelper<std::tuple<Tail...>>::hw_init();
    }
    /// Middle step of the recursion for an individual pin.
    static void hw_set_to_safe()
    {
        Head::hw_set_to_safe();
        GpioInitHelper<std::tuple<Tail...>>::hw_set_to_safe();
    }
    /// Middle step of the recursion for an individual pin.
    static void set(bool value)
    {
        Head::set(value);
        GpioInitHelper<std::tuple<Tail...>>::set(value);
    }
};

/// Class collecting a bunch of GPIO pins and allowing them to be initialized
/// in one go. Usage:
///
/// in hardware.hxx:
///
///  GPIO_PIN(FOO, ...)
///  GPIO_PIN(BAR, ...)
///
///  typedef GpioInitializer<FOO_Pin, BAR_Pin> GpioInit;
///
/// in HwInit.cxx call all GPIO pins:
///
///  void hw_preinit()
///  {
///      GpioInit::hw_init();
///  }
template <typename... Args> struct GpioInitializer
{
public:
    /// Calls the hw_init() function for all declared pins.
    static void hw_init()
    {
        GpioInitHelper<std::tuple<Args...>>::hw_init();
    }
    /// Calls the hw_set_to_safe() function for all declared pins.
    static void hw_set_to_safe()
    {
        GpioInitHelper<std::tuple<Args...>>::hw_set_to_safe();
    }
    /// Calls the set() function for all declared pins.
    static void set(bool value)
    {
        GpioInitHelper<std::tuple<Args...>>::set(value);
    }
};

#endif // _UTILS_GPIOINITIALIZER_HXX_
