/** \copyright
 * Copyright (c) 2016, Balazs Racz
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
 * \file Fixed16.hxx
 *
 * A Fixed-point numeric type with 16.16 specification.
 *
 * @author Balazs Racz
 * @date 19 Dec 2016
 */

#ifndef _UTILS_FIXED16_HXX_
#define _UTILS_FIXED16_HXX_

#include <stdint.h>

#include "utils/macros.h"

class Fixed16
{
public:
    constexpr Fixed16(int16_t integer, uint16_t frac = 0)
        : value_(((integer < 0 ? -integer : integer) << 16) | frac)
        , sign_(integer < 0 ? 1 : 0)
    {
    }

    enum FromDouble
    {
        FROM_DOUBLE
    };

    constexpr Fixed16(FromDouble, double value)
        : value_(value < 0 ? -value * 65536 + 0.5 : value * 65536 + 0.5)
        , sign_(value < 0 ? 1 : 0)
    {
        // it would be nice to make this work:
        // static_assert(value < 32767 && value > -32767,
        //   "fixed16 constructor out of range");
    }
    
    Fixed16(const Fixed16 &o) = default;
    Fixed16 &operator=(const Fixed16 &o) = default;

    Fixed16 &operator+=(Fixed16 o)
    {
        from_int(to_int() + o.to_int());
        return *this;
    }

    template <class T> Fixed16 operator+(T o) const
    {
        Fixed16 ret(*this);
        ret += o;
        return ret;
    }

    Fixed16 &operator-=(Fixed16 o)
    {
        from_int(to_int() - o.to_int());
        return *this;
    }

    template <typename T> Fixed16 operator-(T o) const
    {
        Fixed16 ret(*this);
        ret -= o;
        return ret;
    }

    Fixed16 &operator*=(Fixed16 o)
    {
        uint64_t v = value_;
        v *= o.value_;
        v >>= 16;
        value_ = v;
        sign_ ^= o.sign_;
        return *this;
    }

    template <typename T> Fixed16 operator*(T o) const
    {
        Fixed16 ret(*this);
        ret *= o;
        return ret;
    }

    Fixed16 &operator/=(Fixed16 o)
    {
        uint64_t v = value_;
        v <<= 32;
        v /= o.value_;
        v >>= 16;
        value_ = v;
        sign_ ^= o.sign_;
        return *this;
    }

    template <typename T> Fixed16 operator/(T o) const
    {
        Fixed16 ret(*this);
        ret /= o;
        return ret;
    }

    /// @return the rounded value to the nearest integer
    operator uint16_t() const
    {
        return round();
    }

    /// @return the value rounded to nearest integer.
    int16_t round() const {
        int16_t b = (value_ + 0x8000) >> 16;
        if (sign_) return -b;
        return b;
    }
    
    /// @return the integer part, rounded down
    int16_t trunc() const
    {
        int16_t b = value_ >> 16;
        if (sign_) return -b;
        return b;
    }

    /// @return the fractional part, as an uint16 value between 0 and 0xffff
    uint16_t frac() const
    {
        return value_ & 0xffff;
    }

    float to_float() const
    {
        if (!value_) {
            if (sign_)
            {
                return -0.0f;
            }
            else
            {
                return 0.0f;
            }
        }
        uint32_t f = 0;
        if (sign_) f |= 0x80000000u;
        int lz = __builtin_clz(value_);
        if (lz <= 8)
        {
            HASSERT(((value_ >> (8 - lz)) & 0xFF800000U) == 0x800000U);
            f |= (value_ >> (8 - lz)) & 0x7FFFFF;
        }
        else
        {
            HASSERT(((value_ << (lz - 8)) & 0xFF800000U) == 0x800000U);
            f |= (value_ << (lz - 8)) & 0x7FFFFF;
        }
        uint32_t exp = (127 + 15 - lz) & 0xFF;
        f |= exp << 23;
        return *reinterpret_cast<float *>(&f);
    }

    /*    uint16_t to_float16() {
            if (!value_) return 0;

            }*/

    bool is_positive() const {
        return sign_ == 0;
    }

    void negate() {
        sign_ ^= 1;
    }
    
private:
    /// Translates the current value to a signed fixed-point 32-bit integer.
    int32_t to_int() const
    {
        int32_t r = value_;
        if (sign_)
        {
            return -r;
        }
        else
        {
            return r;
        }
    }

    /// Overwrites the current value from a signed fixed-point 32-bit integer.
    void from_int(int32_t v) {
        if (v<0) {
            sign_ = 1;
            v = -v;
        } else {
            sign_ = 0;
        }
        value_ = v & 0x7fffffffu;
    }
    
    uint32_t value_ : 31;
    uint32_t sign_ : 1;
};

#endif // _UTILS_FIXED16_HXX_
