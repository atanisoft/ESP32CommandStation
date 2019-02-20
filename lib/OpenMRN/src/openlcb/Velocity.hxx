/** \copyright
 * Copyright (c) 2013, Stuart W Baker
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
 * \file Velocity.hxx
 * This file provides an implementation of velocity in NMRAnet terms.
 *
 * @author Stuart W. Baker
 * @date 2 August 2013
 */

#ifndef _OPENLCB_VELOCITY_HXX_
#define _OPENLCB_VELOCITY_HXX_

#include <cmath>
#include <cstdint>

#include "utils/macros.h"

extern "C" {
/* These come from the ieeehalfprecision.c */
/// Converts an IEEE single (float) to a half-precision (float16).
int singles2halfp(void *target, const void *source, int numel);
/// Converts an half-precision (float16) to an IEEE single (float).
int halfp2singles(void *target, const void *source, int numel);
}

/** Conversion factor for MPH. 1 mph = this many m/s. */
#define MPH_FACTOR 0.44704f

/** This type represents how velocity is seen on the wire (16 bit float).
 */
typedef uint16_t float16_t;

namespace openlcb
{

/** This class provides a mechanism for working with velocity in different
 *  forms.  A single precision floating point value is used internally to store
 *  velocity, but this class provides the ability to easily work with
 *  different velocity formats including DCC 14/28/128 speed step formats.
 *  NMRAnet velocity is represented as a floating point meters/sec.  The sign
 *  represents direction where negative is reverse and positive is forward.
 *  Sign is always preserved even when the result is 0.  For example,
 *  -7.0 + 7.0 = -0.0 and 7.0 - 7.0 = +0.0  Be careful thought, per the C
 *  standards, a negative zero (-.0.0) velocity compared to constant 0.0 will
 *  compare true.  Always use @ref direction() when trying to determine the
 *  sign or direction of travel.
 */
class Velocity
{
public:
    /** define an enumeration for direction
     */
    enum
    {
        FORWARD  = false, /**< forward direction */
        REVERSE  = true, /**< reverse direction */
    };

    /** Default constructor.
     */
    Velocity()
        : velocity(0)
    {
    }

    /** Basic constructor.
     * @param value starting value for Velocity.
     */
    Velocity(int value)
        : velocity(value)
    {
    }

    /** Basic constructor.
     * @param value starting value for Velocity.
     */
    Velocity(unsigned int value)
        : velocity(value)
    {
    }

    /** Basic constructor.
     * @param value starting value for Velocity.
     */
    Velocity(float value)
        : velocity(value)
    {
    }

    /** Basic constructor.
     * @param value starting value for Velocity.
     */
    Velocity(double value)
        : velocity(value)
    {
    }

    /** Constructor that takes the 16 bit wire format
     * @param value starting value for Velocity as IEEE half precision float.
     */
    Velocity(float16_t value)
        : velocity(halfp2singles(&velocity, &value, 1))
    {
    }

    /** Copy constructor. */
    Velocity(const Velocity& old_velocity)
        : velocity(old_velocity.velocity)
    {
    }

    /** Destructor does nothing. */
    ~Velocity() {}

    static Velocity from_mph(float mph) {
        Velocity v;
        v.set_mph(mph);
        return v;
    }

    /** Checks whether the speed is unknown. (NaN).
     * @return true if the speed value is NaN. */
    bool isnan() const
    {
        return std::isnan(velocity);
    }

    /** Return the speed independent of direction.
     * @return speed absolute value of velocity
     */
    float speed() const
    {
        return fabsf(velocity);
    }

    /** Return the direction independent of speed.
     * @return direction FORWARD or REVERSE
     */
    bool direction() const
    {
        if (std::signbit(velocity))
        {
            return REVERSE;
        }
        return FORWARD;
    }
    
    void set_direction(bool direction)
    {
        if (direction == FORWARD)
        {
            forward();
        }
        else
        {
            reverse();
        }
    }

    /** Set the direction to forward. */
    void forward()
    {
        if (std::signbit(velocity))
        {
            velocity = -velocity;
        }
    }
    
    /** Set the direction to reverse. */
    void reverse()
    {
        if (!std::signbit(velocity))
        {
            velocity = -velocity;
        }
    }
    
    /** Convert the native meters/sec representation into mile per hour.
     * @return velocity represented as miles per hour. Always non-negative.
     */
    float mph() const
    {
        return speed() / MPH_FACTOR;
    }

    /** Sets the speed value from a given mph value. The sign of the mph value
     * is ignored. */
    void set_mph(float mph)
    {
        // should this be std::copysign? armgcc says not.
        velocity = copysign(mph * MPH_FACTOR, velocity);
    }
    
    /** Get the speed in DCC 128 speed step format.
     *  The mapping from meters/sec is strait forward.  First convert to
     *  miles/hour, then each speed step represents 1 mile/hour.  Saturate at
     *  126 miles/hour.
     *
     *  bit 7:  direction
     *  bits 6..0:  0 = stopped, 1 = estop, 2 - 127 = speed steps 1 - 126
     *  @return DCC encoded speed steps
     */
    uint8_t get_dcc_128();

    /** Set the speed from DCC 128 speed step format.
     *  The mapping from meters/sec is strait forward.  First convert to
     *  miles/hour, then each speed step represents 1 mile/hour.  Saturate at
     *  126 miles/hour.
     *
     *  @param value bit 7:  direction
     *               bits 6..0:  0 = stopped, 1 = estop, 2 - 127 = speed steps 1 - 126
     */
    void set_dcc_128(uint8_t value);
    
    /** Get the speed in DCC 28 speed step format.
     *  This is a decimation of the 128 speed step mode.
     *
     *  bit 7..6:  fixed at b'01'
     *  bit 5:  direction
     *  bits 4:  speed step least significant bit
     *  bits 3..0:  speed step significatn bits 4..1
     *  @return DCC encoded speed steps
     */
    uint8_t get_dcc_28();
    
    /** Set the speed from DCC 28 speed step format.
     *  This is a decimation of the 128 speed step mode.
     *
     *  @param value bit 7..6:  fixed at b'01'
     *               bit 5:  direction
     *               bits 4:  speed step least significant bit
     *               bits 3..0:  speed step significatn bits 4..1
     */
    void set_dcc_28(uint8_t value);

    /** Get the speed in DCC 14 speed step format.
     *  This is a decimation of the 128 speed step mode.
     *
     *  bit 7..6:  fixed at b'01'
     *  bit 5:  direction
     *  bits 4:  reserved 0 for headlight
     *  bits 3..0:  0 = stopped, 4 - 31 = speed steps 1 - 28 
     *  @return DCC encoded speed steps
     */
    uint8_t get_dcc_14();
    
    /** Set the speed from DCC 14 speed step format.
     *  This is a decimation of the 128 speed step mode.
     *
     *  @param value bit 7..6:  fixed at b'01'
     *               bit 5:  direction
     *               bits 4:  reserved 0 for headlight
     *               bits 3..0:  0 = stopped, 4 - 31 = speed steps 1 - 28 
     */
    void set_dcc_14(uint8_t value);
    
    /** Get a wire version of the velocity.
     * @return IEEE half precision floating point representation of velocity
     */
    float16_t get_wire() const
    {
        float16_t result;
        singles2halfp(&result, &velocity, 1);
        return result;
    }
    
    /** Set the value based on the wire version of velocity.
     * @param value IEEE half precision floating point representation of velocity
     */
    void set_wire(float16_t value)
    {
        halfp2singles(&velocity, &value, 1);
    }

    /** Overloaded addition operator. */
    Velocity operator + (const Velocity& v) const
    {
        return Velocity(zero_adjust(velocity + v.velocity,velocity));
    }

    /** Overloaded addition operator. */
    Velocity operator + (const float& v) const
    {
        return Velocity(zero_adjust(velocity + v,velocity));
    }

    /** Overloaded subtraction operator. */
    Velocity operator - (const Velocity& v) const
    {
        return Velocity(zero_adjust(velocity - v.velocity,velocity));
    }

    /** Overloaded subtraction operator. */
    Velocity operator - (const float& v) const
    {
        return Velocity(zero_adjust(velocity - v,velocity));
    }

    /** Overloaded multiplication operator. */
    Velocity operator * (const Velocity& v) const
    {
        return Velocity(zero_adjust(velocity * v.velocity,velocity));
    }

    /** Overloaded multiplication operator. */
    Velocity operator * (const float& v) const
    {
        return Velocity(zero_adjust(velocity * v,velocity));
    }

    /** Overloaded division operator. */
    Velocity operator / (const Velocity& v) const
    {
        return Velocity(zero_adjust(velocity / v.velocity,velocity));
    }

    /** Overloaded division operator. */
    Velocity operator / (const float& v) const
    {
        return Velocity(zero_adjust(velocity / v,velocity));
    }

    /** Overloaded pre-increement operator. */
    Velocity& operator ++ ()
    {
        velocity = zero_adjust(velocity + 1,velocity);
        return *this;
    }

    /** Overloaded post-increment operator. */
    Velocity operator ++ (int)
    {
        Velocity result(*this);
        velocity = zero_adjust(velocity + 1,velocity);
        return result;
    }

    /** Overloaded pre-decreement operator. */
    Velocity& operator -- ()
    {
        velocity = zero_adjust(velocity - 1,velocity);
        return *this;
    }

    /** Overloaded post-decrement operator. */
    Velocity operator -- (int)
    {
        Velocity result(*this);
        velocity = zero_adjust(velocity - 1,velocity);
        return result;
    }

    /** Overloaded addition equals operator. */
    Velocity& operator += (const Velocity& v)
    {
        velocity = zero_adjust(velocity + v.velocity,velocity);
        return *this;
    }

    /** Overloaded addition equals operator. */
    Velocity& operator += (const float& v)
    {
        velocity = zero_adjust(velocity + v,velocity);
        return *this;
    }

    /** Overloaded subtraction equals operator. */
    Velocity& operator -= (const Velocity& v)
    {
        velocity = zero_adjust(velocity - v.velocity,velocity);
        return *this;
    }

    /** Overloaded subtraction equals operator. */
    Velocity& operator -= (const float& v)
    {
        velocity = zero_adjust(velocity - v,velocity);
        return *this;
    }

    /** Overloaded multiplication equals operator. */
    Velocity& operator *= (const Velocity& v)
    {
        velocity = zero_adjust(velocity * v.velocity,velocity);
        return *this;
    }

    /** Overloaded multiplication equals operator. */
    Velocity& operator *= (const float& v)
    {
        velocity = zero_adjust(velocity * v,velocity);
        return *this;
    }

    /** Overloaded division equals operator. */
    Velocity& operator /= (const Velocity& v)
    {
        velocity = zero_adjust(velocity / v.velocity,velocity);
        return *this;
    }

    /** Overloaded division equals operator. */
    Velocity& operator /= (const float& v)
    {
        velocity = zero_adjust(velocity / v,velocity);
        return *this;
    }

    /** Overloaded equals operator. */
    Velocity& operator = (const Velocity& v)
    {
        velocity = v.velocity;
        return *this;
    }

    /** Overloaded equals operator. */
    Velocity& operator = (const float& v)
    {
        velocity = v;
        return *this;
    }

    /** Overloaded equals equals operator */
    bool operator == (const Velocity& v) const
    {
        return (v.velocity == velocity);
    }
    
    /** Overloaded equals equals operator */
    bool operator == (const float& v) const
    {
        return (v == velocity);
    }
    
    /** Overloaded not equals operator */
    bool operator != (const Velocity& v) const
    {
        return (v.velocity != velocity);
    }
    
    /** Overloaded not equals operator */
    bool operator != (const float& v) const
    {
        return (v != velocity);
    }
    
private:
    /** Floating point representation of velocity. */
    float velocity;
    
    /** Adjust for a math result of negative 0.
     * @param value value of math result
     * @param old original value before expression
     */
    static float zero_adjust(float value, float old)
    {
        if (value == 0)
        {
            return copysign(value, old);
        }
        return value;
    }
};


}; /* namespace openlcb */

#endif // _OPENLCB_VELOCITY_HXX_
