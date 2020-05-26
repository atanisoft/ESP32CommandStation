/** \copyright
 * Copyright (c) 2018, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * \file MmapGpio.hxx
 *
 * Gpio wrapper for a single bit in memory.
 *
 * @author Balazs Racz
 * @date 29 April 2018
 */

#ifndef _OS_MMAPGPIO_HXX_
#define _OS_MMAPGPIO_HXX_

#include "os/Gpio.hxx"

/**
 * Gpio wrapper for a single bit in memory. This may be in a peripheral or in
 * SRAM. Access width will be defined by the template parameter. These objects
 * can be allocated in flash, not needing any SRAM or initialization. Use the
 * following syntax:
 *
 * uint32_t output_register[1] = {0};  // in RAM
 *
 * constexpr const MmapGpio PORTD_LINE1(output_register, 7, true);
 *
 */
template<typename STORAGE>
class MmapGpioTemplate : public Gpio
{
public:
    /// Constructor. Constexpr allowing storage in flash for these objects.
    /// @param ptr is a base pointer for a bit array. It must be aligned to
    /// word boundary.
    /// @param bit_ofs is the offset of the interesting bit from the beginning
    /// of the array. 0 == LSB of the first byte, 31 == MSB of byte 3. May be
    /// 32 or more and ptr will be auto adjusted.
    /// @param is_output true if we want to simulate an output gpio, false if
    /// an input gpio.
    constexpr MmapGpioTemplate(
        STORAGE *const ptr, const unsigned bit_ofs, const bool is_output)
        : ptr_(ptr + (bit_ofs >> 5))
        , bit_(bit_ofs & 31)
        , isOutput_(is_output ? 1 : 0)
    {
    }

    void write(Value new_state) const OVERRIDE
    {
        if (new_state == Value::SET) {
            *ptr_ |= (1<<bit_);
        } else {
            *ptr_ &= ~(1<<bit_);
        }
    }

    void set() const OVERRIDE
    {
        *ptr_ |= (1<<bit_);
    }

    void clr() const OVERRIDE
    {
        *ptr_ &= ~(1<<bit_);
    }

    Value read() const OVERRIDE
    {
        return ((*ptr_) & (1<<bit_)) != 0 ? Value::SET : Value::CLR;
    }

    void set_direction(Direction dir) const OVERRIDE
    {
        // Mmapped GPIO cannot change direction.
        HASSERT(dir == direction());
    }

    Direction direction() const OVERRIDE
    {
        return isOutput_ ? Direction::DOUTPUT : Direction::DINPUT;
    }

private:
    /// Pointer to storage.
    STORAGE * const ptr_;
    /// Which bit. 0 == LSB, from there it goes up towards MSB.
    const unsigned bit_ : 5;
    /// 1 if this GPIO is an output, 0 if it's an input.
    const unsigned isOutput_ : 1;
};

using MmapGpio = MmapGpioTemplate<uint32_t>;
using MmapGpio8 = MmapGpioTemplate<uint8_t>;

#endif // _OS_MMAPGPIO_HXX_
