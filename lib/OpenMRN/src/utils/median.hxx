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
 * \file median.hxx
 *
 * Utilities for computing medians
 *
 * @author Balazs Racz
 * @date 24 June 2018
 */

#ifndef _UTILS_MEDIAN_HXX_
#define _UTILS_MEDIAN_HXX_

/// Sorts two values.
/// @param a first value (will be the smaller after the call)
/// @param b second value (will be the greater after the call)
inline void __attribute__((__always_inline__, optimize("O3")))
comp_swap(unsigned &a, unsigned &b)
{
    if (a > b)
    {
        unsigned tmp = a;
        a = b;
        b = tmp;
    }
}

/// Computes the median of five values.
/// @return median value.
inline unsigned __attribute__((__always_inline__, optimize("O3")))
median_5(unsigned a, unsigned b, unsigned c, unsigned d, unsigned e)
{
    comp_swap(a, c);
    comp_swap(b, d);
    comp_swap(a, b);
    comp_swap(c, d);
    // Now: a is the smallest of [a,b,c,d]
    // d is the greatest of [a,b,c,d]; neither of these can be the median.
    comp_swap(b, c);
    // Now: [a,b,c,d] are sorted
    if (e > c) {
        return c;
    }
    if (e < b) {
        return b;
    }
    return e;
}

#endif // _UTILS_MEDIAN_HXX_
