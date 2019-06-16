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
 * \file RailcomDebug.cxx
 *
 * Implementation of railcom printf debugging.
 *
 * @author Balazs Racz
 * @date 18 May 2015
 */

#include "dcc/RailCom.hxx"
#include "utils/StringPrintf.hxx"

namespace dcc
{

string railcom_debug(const Feedback &fb)
{
    string ret;
    ret += StringPrintf("ch1(%d) ", fb.ch1Size);
    for (int i = 0; i < fb.ch1Size; ++i)
    {
        ret += StringPrintf(
            " %02x=%02x", fb.ch1Data[i], railcom_decode[fb.ch1Data[i]]);
    }
    ret += StringPrintf(" ch2(%d) ", fb.ch2Size);
    for (int i = 0; i < fb.ch2Size; ++i)
    {
        ret += StringPrintf(
            " %02x=%02x", fb.ch2Data[i], railcom_decode[fb.ch2Data[i]]);
    }
    return ret;
}

} // namespace dcc
