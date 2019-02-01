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
 * \file Velocity.cxx
 * This file provides an implementation of velocity in NMRAnet terms.
 *
 * @author Stuart W. Baker
 * @date 2 August 2013
 */

#include "openlcb/Velocity.hxx"

namespace openlcb
{
    
/** Get the speed in DCC 128 speed step format.
 *  The mapping from meters/sec is strait forward.  First convert to
 *  miles/hour, then each speed step represents 1 mile/hour.  Saturate at
 *  126 miles/hour.
 *
 *  bit 7:  direction
 *  bits 6..0:  0 = stopped, 1 = estop, 2 - 127 = speed steps 1 - 126
 *  @return DCC encoded speed steps
 */
uint8_t Velocity::get_dcc_128()
{
    uint8_t result;
    uint32_t tmp = (speed() * MPH_FACTOR) + 0.5;
    
    if (tmp == 0)
    {
        result = 0;
    }
    else if (tmp > 126)
    {
        result = 127;
    }
    else
    {
        result = (uint8_t)(tmp + 1);
    }
    
    result |= std::signbit(velocity) ? 0x00 : 0x80;
    return result;
}

/** Set the speed from DCC 128 speed step format.
 *  The mapping from meters/sec is strait forward.  First convert to
 *  miles/hour, then each speed step represents 1 mile/hour.  Saturate at
 *  126 miles/hour.
 *
 *  @param value bit 7:  direction
 *               bits 6..0:  0 = stopped, 1 = estop, 2 - 127 = speed steps 1 - 126
 */
void Velocity::set_dcc_128(uint8_t value)
{
    if ((value & 0x7F) <= 1)
    {
        velocity = 0;
    }
    else
    {
        velocity = (value & 0x07F) - 1;
        velocity /= MPH_FACTOR;
    }
    
    if ((value & 0x80) == 0)
    {
        velocity = -velocity;
    }
}

/** Get the speed in DCC 28 speed step format.
 *  This is a decimation of the 128 speed step mode.
 *
 *  bit 7..6:  fixed at b'01'
 *  bit 5:  direction
 *  bits 4:  speed step least significant bit
 *  bits 3..0:  speed step significatn bits 4..1
 *  @return DCC encoded speed steps
 */
uint8_t Velocity::get_dcc_28()
{
    uint8_t result;
    uint32_t tmp = ((speed() * MPH_FACTOR * 28) / 128) + 0.5;
    
    if (tmp == 0)
    {
        result = 0;
    }
    else if (tmp > 28)
    {
        result = 31;
    }
    else
    {
        result = (uint8_t)(tmp + 3);
    }
    
    result |= result & 0x01 ? 0xA0 : 0x80;
    
    result >>= 1;

    result |= std::signbit(velocity) ? 0x00 : 0x20;
    return result;
}

/** Set the speed from DCC 28 speed step format.
 *  This is a decimation of the 128 speed step mode.
 *
 *  @param value bit 7..6:  fixed at b'01'
 *               bit 5:  direction
 *               bits 4:  speed step least significant bit
 *               bits 3..0:  speed step significatn bits 4..1
 */
void Velocity::set_dcc_28(uint8_t value)
{
    value <<= 1;
    
    value |= value & 0x20 ? 0x01 : 0x00;
    
    if ((value & 0x1F) <= 3)
    {
        velocity = 0;
    }
    else
    {
        velocity = (value & 0x01F) - 3;
        velocity *= 128;
        velocity /= (28 * MPH_FACTOR);
    }
    
    if ((value & 0x40) == 0)
    {
        velocity = -velocity;
    }
}

/** Get the speed in DCC 14 speed step format.
 *  This is a decimation of the 128 speed step mode.
 *
 *  bit 7..6:  fixed at b'01'
 *  bit 5:  direction
 *  bits 4:  reserved 0 for headlight
 *  bits 3..0:  0 = stopped, 4 - 31 = speed steps 1 - 28 
 *  @return DCC encoded speed steps
 */
uint8_t Velocity::get_dcc_14()
{
    uint8_t result;
    uint32_t tmp = ((speed() * MPH_FACTOR * 14) / 128) + 0.5;
    
    if (tmp == 0)
    {
        result = 0;
    }
    else if (tmp > 14)
    {
        result = 15;
    }
    else
    {
        result = (uint8_t)(tmp + 1);
    }
    
    result |= 0x40;

    result |= std::signbit(velocity) ? 0x00 : 0x20;
    return result;
}

/** Set the speed from DCC 14 speed step format.
 *  This is a decimation of the 128 speed step mode.
 *
 *  @param value bit 7..6:  fixed at b'01'
 *               bit 5:  direction
 *               bits 4:  reserved 0 for headlight
 *               bits 3..0:  0 = stopped, 2 - 15 = speed steps 1 - 14
 */
void Velocity::set_dcc_14(uint8_t value)
{
    if ((value & 0x0F) <= 1)
    {
        velocity = 0;
    }
    else
    {
        velocity = (value & 0x0F) - 1;
        velocity *= 128;
        velocity /= (14 * MPH_FACTOR);
    }
    
    if ((value & 0x20) == 0)
    {
        velocity = -velocity;
    }
}
    
}; /* namespace openlcb */
