/** \copyright
 * Copyright (c) 2016, Balazs Racz
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
 * \file format_utils.hxx
 *
 * Simple codefor some trivial formatting uses.
 *
 * @author Balazs Racz
 * @date 7 Feb 2016
 */

#ifndef _UTILS_FORMAT_UTILS_HXX_
#define _UTILS_FORMAT_UTILS_HXX_

#include <string>

/** Renders an integer to string, left-justified. @param buffer must be an at
 * @param buffer must be an at least 10 character long array.
 * @param value will be rendered into the buffer.
 * @returns the pointer to the null character at the end of the rendering.
 */
char* integer_to_buffer(int value, char* buffer);

/** Renders an unsigned integer to string, left-justified.
 * @param buffer must be an at least 10 character long array.
 * @param value will be rendered into the buffer.
 * @returns the pointer to the null character at the end of the rendering.
 */
char* unsigned_integer_to_buffer(int value, char* buffer);

/** Renders an uint64_t to string, left-justified.
 * @param buffer must be an at least 21 character long array.
 * @param value will be rendered into the buffer.
 * @returns the pointer to the null character at the end of the rendering.
 */
char* uint64_integer_to_buffer(uint64_t value, char* buffer);

/** Renders an int64_t to string, left-justified.
 * @param buffer must be an at least 22 character long array.
 * @param value will be rendered into the buffer.
 * @returns the pointer to the null character at the end of the rendering.
 */
char* int64_integer_to_buffer(int64_t value, char* buffer);

/** Renders an unsigned integer to string, left-justified.
 * @param buffer must be an at least 10 character long array.
 * @param value will be rendered into the buffer.
 * @returns the pointer to the null character at the end of the rendering.
 */
char* unsigned_integer_to_buffer_hex(unsigned int value, char* buffer);

/** Renders an uint64_t to string, left-justified.
 * @param buffer must be an at least 17 character long array.
 * @param value will be rendered into the buffer.
 * @returns the pointer to the null character at the end of the rendering.
 */
char* uint64_integer_to_buffer_hex(uint64_t value, char* buffer);

/** Renders an int64_t to string, left-justified.
 * @param buffer must be an at least 18 character long array.
 * @param value will be rendered into the buffer.
 * @returns the pointer to the null character at the end of the rendering.
 */
char* int64_integer_to_buffer_hex(int64_t value, char* buffer);

/** Renders an integer to std::string, left-justified.
 * @param value will be rendered into the buffer.
 * @param padding number of bytes that the resulting string should be.
 * @returns the pointer to the null character at the end of the rendering.
 */
string integer_to_string(int value, unsigned padding = 0);

/** Renders an uint64_t to std::string, left-justified.
 * @param value will be rendered into the buffer.
 * @param padding number of bytes that the resulting string should be.
 * @returns the pointer to the null character at the end of the rendering.
 */
string uint64_to_string(uint64_t value, unsigned padding = 0);

/** Renders an int64_t to std::string, left-justified.
 * @param value will be rendered into the buffer.
 * @param padding number of bytes that the resulting string should be.
 * @returns the pointer to the null character at the end of the rendering.
 */
string int64_to_string(int64_t value, unsigned padding = 0);

/** Renders an uint64_t to std::string, left-justified.
 * @param value will be rendered into the buffer.
 * @param padding number of bytes that the resulting string should be.
 * @returns the pointer to the null character at the end of the rendering.
 */
string uint64_to_string_hex(uint64_t value, unsigned padding = 0);

/** Renders an int64_t to std::string, left-justified.
 * @param value will be rendered into the buffer.
 * @param padding number of bytes that the resulting string should be.
 * @returns the pointer to the null character at the end of the rendering.
 */
string int64_to_string_hex(int64_t value, unsigned padding = 0);

/// Formats a MAC address to string. Works both for Ethernet addresses as well
/// as for OpenLCB node IDs.
///
/// @param mac a 6-byte array storing the MAC address. mac[0] will be printed
/// at the beginning.
/// @param colons true to print colons, else false to exclude the colon
/// seperators
///
/// @return a string containing a colon-separated hexadecimal printout of the
/// given MAC address.
///
string mac_to_string(uint8_t mac[6], bool colons = true);

/// Formats an IPv4 address to string.
///
/// @param ip a 4-byte array storing the IPv4 address. ip[3] will be printed
/// at the beginning.
///
/// @return a string containing a dot-separated printout of the
/// given IPv4 address.
///
string ipv4_to_string(uint8_t ip[4]);

/// Formats an IPv4 address to string.
///
/// @param ip a uint32_t storing the IPv4 address. most significant bytes
/// will be printed at the beginning.
///
/// @return a string containing a dot-separated printout of the
/// given IPv4 address.
///
inline string ipv4_to_string(uint32_t ip)
{
    return ipv4_to_string((uint8_t*)&ip);
}

#endif // _UTILS_FORMAT_UTILS_HXX_
