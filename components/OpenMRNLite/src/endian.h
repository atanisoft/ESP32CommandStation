/** \copyright
 * Copyright (c) 2012, Stuart W Baker
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
 * \file freertos/endian.h
 * This file represents endian swapping for FreeRTOS.
 *
 * @author Stuart W. Baker
 * @date 13 August 2012
 */

#ifndef _endian_h_
#define _endian_h_

#include <stdint.h>

/** Byte swap a 16 bit value.
 * @param x value to swap
 * @return byte swapped value
 */
static inline uint16_t __bswap_16(uint16_t x)
{
    return (((x & 0xff00) >> 8) | ((x & 0x00ff) << 8));
}

/** Byte swap a 32 bit value.
 * @param x value to swap
 * @return byte swapped value
 */
static inline uint32_t __bswap_32(uint32_t x)
{
    return (((x & 0xff000000) >> 24) | ((x & 0x00ff0000) >>  8) |
            ((x & 0x0000ff00) <<  8) | ((x & 0x000000ff) << 24));
}

/** Byte swap a 64 bit value.
 * @param x value to swap
 * @return byte swapped value
 */
static inline uint64_t __bswap_64(uint64_t x)
{
    return (((x & 0xff00000000000000ULL) >> 56) |
            ((x & 0x00ff000000000000ULL) >> 40) |
            ((x & 0x0000ff0000000000ULL) >> 24) |
            ((x & 0x000000ff00000000ULL) >>  8) |
            ((x & 0x00000000ff000000ULL) <<  8) |
            ((x & 0x0000000000ff0000ULL) << 24) |
            ((x & 0x000000000000ff00ULL) << 40) |
            ((x & 0x00000000000000ffULL) << 56));
}

#ifdef CONFIG_ENDIAN_BIG
    #define htobe16(x) (x)
    #define htole16(x) __bswap_16 (x)
    #define be16toh(x) (x)
    #define le16toh(x) __bswap_16 (x)

    #define htobe32(x) (x)
    #define htole32(x) __bswap_32 (x)
    #define be32toh(x) (x)
    #define le32toh(x) __bswap_32 (x)

    #define htobe64(x) (x)
    #define htole64(x) __bswap_64 (x)
    #define be64toh(x) (x)
    #define le64toh(x) __bswap_64 (x)
#else
/// Converts a host endian 16-bit value to big endian.
    #define htobe16(x) __bswap_16 (x)
/// Converts a host endian 16-bit value to little endian.
    #define htole16(x) (x)
/// Converts a big endian 16-bit value to host endian.
    #define be16toh(x) __bswap_16 (x)
/// Converts a little endian 16-bit value to host endian.
    #define le16toh(x) (x)

/// Converts a host endian 32-bit value to big endian.
    #define htobe32(x) __bswap_32 (x)
/// Converts a host endian 32-bit value to little endian.
    #define htole32(x) (x)
/// Converts a big endian 32-bit value to host endian.
    #define be32toh(x) __bswap_32 (x)
/// Converts a little endian 32-bit value to host endian.
    #define le32toh(x) (x)

/// Converts a host endian 64-bit value to big endian.
    #define htobe64(x) __bswap_64 (x)
/// Converts a host endian 64-bit value to little endian.
    #define htole64(x) (x)
/// Converts a big endian 64-bit value to host endian.
    #define be64toh(x) __bswap_64 (x)
/// Converts a little endian 64-bit value to host endian.
    #define le64toh(x) (x)
#endif

#endif /* _endian_h_ */

