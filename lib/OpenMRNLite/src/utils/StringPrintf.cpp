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
 * \file StringPrintf.hxx
 *
 * Utility for creating c++ strings on demand with a printf-like structure.
 *
 * @author Balazs Racz
 * @date 10 May 2015
 */

#include "utils/StringPrintf.hxx"

std::string StringPrintf(const char *format, ...)
{
#ifdef __FreeRTOS__
    static const int kBufSize = 64;
#else
    static const int kBufSize = 1000;
#endif
    char buffer[kBufSize];
    va_list ap;

    va_start(ap, format);
    int n = vsnprintf(buffer, kBufSize, format, ap);
    va_end(ap);
    HASSERT(n >= 0);
    if (n < kBufSize)
    {
        return string(buffer, n);
    }
    string ret(n + 1, 0);
    va_start(ap, format);
    n = vsnprintf(&ret[0], ret.size(), format, ap);
    va_end(ap);
    HASSERT(n >= 0);
    ret.resize(n);
    return ret;
}
