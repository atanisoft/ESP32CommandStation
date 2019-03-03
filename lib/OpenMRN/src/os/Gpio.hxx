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
 * \file Gpio.hxx
 *
 * Generic implementation header for GPIO.
 *
 * @author Stuart Baker, Balazs Racz
 * @date 1 July 2015
 */

#ifndef _OS_GPIO_HXX_
#define _OS_GPIO_HXX_

#include "utils/macros.h"

/** OS-independent abstraction for GPIO.
 */
class Gpio
{
public:
    /** Constructor. It is important that this is constexpr so that the Gpio
     * object instances can be initialized by the data segment and be avalable
     * before constructor calls. */
    constexpr Gpio()
    {
    }

    /* It is important that there be no destructor declared here, not even with
     * empty and inlined and non-virtual body. The Gpio objects are never
     * deleted through their base class; the specific implementations must live
     * in HwInit.cxx with global scope. If there is a destructor here, that
     * will prevent the compiler from creating the Gpio instance variables in
     * the flash, and they will all start using RAM instead. That's a big
     * difference. */

    /** Defines the options for GPIO level. */
    enum Value : bool
    {
        CLR = false,
        SET = true,
        VLOW = CLR,
        VHIGH = SET
    };

    /** Defines the options for GPIO direction. This enum must always be used
     * fully qualified (i.e. Gpio::Direction::DINPUT and
     * Gpio::Direction::DOUTPUT). */
    enum class Direction
    {
        DINPUT,
        DOUTPUT,
    };

    /** Writes a GPIO output pin (set or clear to a specific state).
     * @param new_state the desired output state. See @ref Value.
     */
    virtual void write(Value new_state) const = 0;

    /** Writes a GPIO output pin (set or clear to a specific state).
     *
     * This is an inline function because this class encapsulates that
     * HIGH==true and LOW==false, thus a direct typecast belongs here and not
     * to customer code.
     *
     * @param new_state bool representing the desired output state. true ==
     * high, false == low.
     */
    void write(bool new_state) const
    {
        write((Value)new_state);
    }

    /** Retrieves the current @ref Value of a GPIO input pin.
     * @return @ref SET if currently high, @ref CLR if currently low.
     */
    virtual Value read() const = 0;

    /** Tests the GPIO input pin to see if it is set.
     * @return true if input pin is currently high, false if currently low.
     */
    bool is_set() const
    {
        return read() == SET;
    }

    /** Tests the GPIO input pin to see if it is clear.
     * @return true if input pin is currently low, false if currently high.
     */
    bool is_clr()const
    {
        return read() == CLR;
    }

    /** Sets the GPIO output pin to high.
     */
    virtual void set() const = 0;

    /** Clears the GPIO output pin to low.
     */
    virtual void clr() const = 0;

    /** Sets the GPIO direction.
     * @param dir @ref INPUT or @ref OUTPUT
     */
    virtual void set_direction(Direction dir) const = 0;

    /** Gets the GPIO direction.
     * @return @ref INPUT or @ref OUTPUT
     */
    virtual Direction direction() const = 0;
};

#endif /* _FREERTOS_DRIVERS_COMMON_GPIOGENERIC_HXX_ */
