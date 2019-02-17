/** \copyright
 * Copyright (c) 2013, Balazs Racz
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
 * \file EventHandler.cxx
 * Static declarations for handling NMRAnet events.
 *
 * @author Balazs Racz
 * @date 19 October 2013
 */

#include "openlcb/EventHandler.hxx"
#include "openlcb/WriteHelper.hxx"

namespace openlcb
{

EventRegistry::EventRegistry()
{
}

EventRegistry::~EventRegistry()
{
}

// static
unsigned EventRegistry::align_mask(EventId *event, unsigned size)
{
    // example: size = 140. highest bit set is bit 7
    // example2: size = 256. highest bit set is bit 8, but it is power-of-two.
    HASSERT(event);
    if (size <= 1)
    {
        return 0;
    }
    // should be 7 in both examples
    unsigned log2 = sizeof(size) * 8 - __builtin_clz(size - 1) - 1;
    uint64_t new_event, rounded_range;
    do
    {
        ++log2;                       // 8 in both examples
        rounded_range = 1ULL << log2; // 256
        new_event = *event & (~(rounded_range - 1));
        // we have to be careful for overflowing uint64 in new_event =
        // rounded_range
    } while (log2 < 64 &&
             ((new_event + (rounded_range - 1)) < (*event + (size - 1))));
    if (log2 >= 64)
    {
        new_event = 0;
        log2 = 64;
    }
    // The rounding is successful.
    *event = new_event;
    return log2;
}

} /* namespace openlcb */
