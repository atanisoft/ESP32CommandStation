/** \copyright
 * Copyright (c) 2019, Balazs Racz
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
 * \file Base64.cxx
 *
 * Helper function to perform encoding & decoding of Base64 data.
 *
 * @author Balazs Racz
 * @date 31 July 2019
 */

#include "utils/Base64.hxx"

#include "utils/macros.h"

static const char nib64[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

std::string base64_encode(const std::string &binary)
{
    std::string ret;
    ret.reserve((binary.size() + 2) / 3 * 4);
    unsigned ofs = 0;
    while (ofs + 3 <= binary.size())
    {
        uint32_t d = (((uint8_t)binary[ofs]) << 16) //
            | (((uint8_t)binary[ofs + 1]) << 8)     //
            | (((uint8_t)binary[ofs + 2]));
        ret.push_back(nib64[(d >> 18) & 63]);
        ret.push_back(nib64[(d >> 12) & 63]);
        ret.push_back(nib64[(d >> 6) & 63]);
        ret.push_back(nib64[(d >> 0) & 63]);
        ofs += 3;
    }
    if (ofs + 1 == binary.size())
    {
        uint32_t d = (((uint8_t)binary[ofs]) << 16);
        ret.push_back(nib64[(d >> 18) & 63]);
        ret.push_back(nib64[(d >> 12) & 63]);
        ret.push_back('=');
        ret.push_back('=');
    }
    else if (ofs + 2 == binary.size())
    {
        uint32_t d = (((uint8_t)binary[ofs]) << 16) //
            | (((uint8_t)binary[ofs + 1]) << 8);
        ret.push_back(nib64[(d >> 18) & 63]);
        ret.push_back(nib64[(d >> 12) & 63]);
        ret.push_back(nib64[(d >> 6) & 63]);
        ret.push_back('=');
    }
    return ret;
}

/// Decode a single base64 nibble.
/// @param c is the base64 nibble.
/// @return 64 if this is the padding nibble; 65 if it's an invalid nibble, or
/// 0..63 if it's a valid nibble.
unsigned nib64_to_byte(char c)
{
    switch (c)
    {
        case '=':
            return 64;
        case '-':
        case '+':
            return 62;
        case '_':
        case '/':
            return 63;
        default:
            break;
    }
    if ('A' <= c && c <= 'Z')
    {
        return c - 'A';
    }
    if ('a' <= c && c <= 'z')
    {
        return c - 'a' + 26;
    }
    if ('0' <= c && c <= '9')
    {
        return c - '0' + 52;
    }
    return 65;
}

bool base64_decode(const std::string &base64, std::string *data)
{
    HASSERT(data);
    data->clear();
    unsigned a = 0;
    unsigned have = 0; // how many bits we have in the accumulator
    unsigned ofs = 0;  // next input byte.
    while (ofs < base64.size())
    {
        int nib = nib64_to_byte(base64[ofs++]);
        if (nib >= 65)
            return false;
        if (nib == 64)
            return true;
        a <<= 6;
        a |= (nib & 63);
        have += 6;
        if (have >= 8)
        {
            data->push_back((char)((a >> (have - 8)) & 0xff));
            have -= 8;
        }
    }
    return true;
}
