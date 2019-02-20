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
 * \file Charlieplex.hxx
 * Helper classes for driving multiple LEDs using charlieplexing output lines.
 *
 * @author Balazs Racz
 * @date 8 Nov 2015
 */

#ifndef _UTILS_CHARLIEPLEXING_HXX_
#define _UTILS_CHARLIEPLEXING_HXX_

#include "os/Gpio.hxx"
#include "utils/Fixed16.hxx"
#include "utils/Uninitialized.hxx"

template <unsigned N> struct CharlieplexHelper;

/// Helper structure for the charlieplexing implementation.
template <> struct CharlieplexHelper<3>
{
    /// Flattened list of pars of output pin numbers to use for a given LED.
    static const uint8_t pinlist[];
    /// @return How many LEDs are we driving.
    static unsigned num_bits()
    {
        return 6;
    }

    /// Which pin to drive high for a given LED. @param bit is the number of
    /// the LED, from 0..num_bits()-1. @return index of the GPIO to drive high.
    static unsigned pin_high(unsigned bit)
    {
        return pinlist[bit << 1];
    }
    /// Which pin to drive low for a given LED. @param bit is the number of
    /// the LED, from 0..num_bits()-1. @return index of the GPIO to drive low.
    static unsigned pin_low(unsigned bit)
    {
        return pinlist[(bit << 1) | 1];
    }
};

const uint8_t CharlieplexHelper<3>::pinlist[] = {
    0, 2, //
    0, 1, //
    1, 2, //
    1, 0, //
    2, 1, //
    2, 0, //
};

/// Helper structure for the charlieplexing implementation.
template <> struct CharlieplexHelper<4>
{
    /// Flattened list of pars of output pin numbers to use for a given LED.
    static const uint8_t pinlist[];
    /// @return How many LEDs are we driving.
    static constexpr unsigned num_bits()
    {
        return 12;
    }

    /// Which pin to drive high for a given LED. @param bit is the number of
    /// the LED, from 0..num_bits()-1. @return index of the GPIO to drive high.
    static unsigned pin_high(unsigned bit)
    {
        return pinlist[bit << 1];
    }
    /// Which pin to drive low for a given LED. @param bit is the number of
    /// the LED, from 0..num_bits()-1. @return index of the GPIO to drive low.
    static unsigned pin_low(unsigned bit)
    {
        return pinlist[(bit << 1) | 1];
    }
};

const uint8_t CharlieplexHelper<4>::pinlist[] = {
    3, 0, //
    2, 0, //
    1, 0, //
    3, 1, //
    3, 2, //
    2, 1, //
    2, 3, //
    1, 2, //
    0, 2, //
    0, 3, //
    0, 1, //
    1, 3, //
};

/// Class that implements a Charlieplexing LED driver, operating N choose 2
/// output LEDs from N GPIO pins.
/// 
/// Usage:
/// 
/// Specify the iteration over the pairs of pins like above if you have more
/// than 3 pins.
/// 
/// Create the Charlieplex instance with supplying the Gpio object
/// pointers. The object pointers could point to flash.
///
/// Set the desired values for the individual LEDs through @ref payload(). Bit
/// 0 in the pointed object will be the output led 0.
///
/// From a hardware timer interrupt repeatedly call the @ref tick() function. A
/// rate of 100 Hz * number of LEDs is advised. It is important that the time
/// between different calls of tick() be somewhat homogenous or else an equal
/// intensity of the individual LEDs that are turned on cannot be guaranteed.
template <unsigned N, class helper = CharlieplexHelper<N>> class Charlieplex
{
public:
    /// Constructor.
    ///
    /// @param pins an array of Gpio object pointers defining the output pins to
    /// use. This array may be in flash (.rodata).
    ///
    Charlieplex(const Gpio *const pins[N])
        : pins_(pins)
        , nextBit_(0)
        , bits_(0)
    {
        for (unsigned i = 0; i < N; ++i)
        {
            pins_[i]->set_direction(Gpio::Direction::DINPUT);
        }
    }

    /// Switches to the next output configuration of the charlieplexing
    /// pins. Call this repeatedly, for example from a hardware timer
    /// interrupt.
    void tick()
    {
        pins_[helper::pin_high(nextBit_)]->set_direction(
            Gpio::Direction::DINPUT);
        pins_[helper::pin_low(nextBit_)]->set_direction(Gpio::Direction::DINPUT);
        nextBit_++;
        if (nextBit_ >= helper::num_bits()) {
            nextBit_ = 0;
        }
        if (bits_ & (1 << nextBit_))
        {
            pins_[helper::pin_high(nextBit_)]->set_direction(
                Gpio::Direction::DOUTPUT);
            pins_[helper::pin_high(nextBit_)]->set();
            pins_[helper::pin_low(nextBit_)]->set_direction(
                Gpio::Direction::DOUTPUT);
            pins_[helper::pin_low(nextBit_)]->clr();
        }
    }

    /// @return the storage representing the state of the output bits. Bit 0 is
    /// LED0, bit 1 is LED1, etc.
    unsigned *payload()
    {
        return &bits_;
    }

private:
    const Gpio *const *pins_; ///< array of all GPIO pins to use
    unsigned nextBit_;        ///< LED that comes next
    unsigned bits_;           ///< Desired output state of LEDs.
};

/// Variant of the {@link Charlieplex} class that allows intensity to be set
/// for each individual pin.
template <unsigned N, class helper = CharlieplexHelper<N>> class WeightedCharlieplex
{
public:
    /// Constructor.
    ///
    /// @param pins an array of Gpio object pointers defining the output pins to
    /// use. This array may be in flash (.rodata).
    ///
    WeightedCharlieplex(const Gpio *const pins[helper::num_bits()])
        : pins_(pins)
        , nextBit_(0)
        , bits_(0)
    {
        for (unsigned i = 0; i < N; ++i)
        {
            pins_[i]->set_direction(Gpio::Direction::DINPUT);
        }
        for (unsigned i = 0; i < helper::num_bits(); ++i) {
            actualIntensity_[i].emplace(0);
            desiredIntensity_[i] = 0;
        }
    }

    /// Switches to the next output configuration of the charlieplexing
    /// pins. Call this repeatedly, for example from a hardware timer
    /// interrupt.
    void tick()
    {
        pins_[helper::pin_high(nextBit_)]->set_direction(
            Gpio::Direction::DINPUT);
        pins_[helper::pin_low(nextBit_)]->set_direction(Gpio::Direction::DINPUT);
        nextBit_++;
        if (nextBit_ >= helper::num_bits()) {
            nextBit_ = 0;
        }
        bool lit = false;
        if (bits_ & (1 << nextBit_))
        {
            lit = true;
        }
        else if (desiredIntensity_[nextBit_] > 0)
        {
            (*actualIntensity_[nextBit_]) *= EWMA_RATIO;
            if (actualIntensity_[nextBit_]->round() <
                desiredIntensity_[nextBit_])
            {
                lit = true;
                (*actualIntensity_[nextBit_]) += EWMA_SUM;
            }
        }
        if (lit)
        {
            pins_[helper::pin_high(nextBit_)]->set_direction(
                Gpio::Direction::DOUTPUT);
            pins_[helper::pin_high(nextBit_)]->set();
            pins_[helper::pin_low(nextBit_)]->set_direction(
                Gpio::Direction::DOUTPUT);
            pins_[helper::pin_low(nextBit_)]->clr();
        }
    }

    /// @return the storage representing the state of the output bits. Bit 0 is
    /// LED0, bit 1 is LED1, etc.
    unsigned *payload()
    {
        return &bits_;
    }

    /// @return the storage where the desired intensity can be written for
    /// partial lighting.
    uint8_t *desired_intensity()
    {
        return desiredIntensity_;
    }

private:
    static constexpr Fixed16 EWMA_RATIO{0, 0xF000};
    static constexpr Fixed16 EWMA_SUM{16, 0};
    
    const Gpio *const *pins_; ///< array of all GPIO pins to use
    unsigned nextBit_;        ///< LED that comes next
    unsigned bits_;           ///< Desired output state of LEDs.
    uint8_t desiredIntensity_[helper::num_bits()]; ///< Partial lighting of each output.
    uninitialized<Fixed16> actualIntensity_[helper::num_bits()]; ///< How much we actually lit a given pin.
};

template<unsigned N, typename helper>
constexpr Fixed16 WeightedCharlieplex<N, helper>::EWMA_SUM;

#endif // _UTILS_CHARLIEPLEXING_HXX_
