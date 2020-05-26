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
 * \file GridConnect.hxx
 * This file implements the logic for encoding and decoding grid connect
 * protocol.
 *
 * @author Stuart W. Baker
 * @date 28 September 2013
 */

#ifndef _UTILS_GRIDCONNECT_HXX_
#define _UTILS_GRIDCONNECT_HXX_

#include <sys/types.h>
#include <cstdint>

#include "can_frame.h"
#include "utils/macros.h"

/** This class provides a means to write CAN data to an a file descriptor in
 * Grid Connect format.
 */
class GridConnect
{
protected:
    /** Constructor.
     */
    GridConnect()
    {
    }

    /** Destructor.
     */
    ~GridConnect()
    {
    }
    
    /** Write a CAN packet to Grid Connect interface in single format.
     * @param fd file descriptor for device
     * @param data can data to write
     * @param len length of data, should be a multiple of sizeof(struct can_frame)
     * @return number of bytes written, or -1 with errno set
     */
    static ssize_t write(int fd, const void *data, size_t len)
    {
        return write_generic(fd, data, len, false);
    }

    /** Write a CAN packet to Grid Connect interface in double format.
     * @param fd file descriptor for device
     * @param data can data to write
     * @param len length of data, should be a multiple of sizeof(struct can_frame)
     * @return number of bytes written, or -1 with errno set
     */
    static ssize_t write_double(int fd, const void *data, size_t len)
    {
        return write_generic(fd, data, len, true);
    }

    /** Read a CAN packet to Grid Connect interface in single format.
     * @param fd file descriptor for device
     * @param data location to read can data into
     * @param len length of data, should be a multiple of sizeof(struct can_frame)
     * @return number of bytes read, or -1 with errno set
     */
    static ssize_t read(int fd, void *data, size_t len)
    {
        return read_generic(fd, data, len, false);
    }

    /** Read a CAN packet to Grid Connect interface in double format.
     * @param fd file descriptor for device
     * @param data location to read can data into
     * @param len length of data, should be a multiple of sizeof(struct can_frame)
     * @return number of bytes read, or -1 with errno set
     */
    static ssize_t read_double(int fd, void *data, size_t len)
    {
        return read_generic(fd, data, len, true);
    }

private:
    /** Write a CAN packet to Grid Connect interface in single or double format.
     * @param fd file descriptor for device
     * @param data can data to write
     * @param len length of data, should be a multiple of sizeof(struct can_frame)
     * @param doub true if this is a double write
     * @return number of bytes written, or -1 with errno set
     */
    static ssize_t write_generic(int fd, const void *data, size_t len, bool doub);

    /** Read a CAN packet to Grid Connect interface in single or double format.
     * @param fd file descriptor for device
     * @param data location to read can data into
     * @param len length of data, should be a multiple of sizeof(struct can_frame)
     * @param doub true if this is a double read
     * @return number of bytes read, or -1 with errno set
     */
    static ssize_t read_generic(int fd, void *data, size_t len, bool doub);

    /** @param frame is the binary CAn frame to encode.
     * @param buf array of at least 28 bytes to place encoded data into
     * @return number of bytes making up the final encoded packet
     */
    static ssize_t encode(struct can_frame *frame, unsigned char buf[]);

    /** Builds an ASCII character representation of a nibble value
     * @param nibble value to convert
     * @return converted value
     */
    static char nibble_to_ascii(int nibble)
    {
        nibble &= 0xf;

        if (nibble < 10)
        {
            return ('0' + nibble);
        }
        
        return ('A' + (nibble - 10));
    }

    /** Take a pair of ASCII characters and convert them to a byte value.
     * pointer to two ASCII characters
     * @param pair points to two hex digits.
     * @return byte value
     */
    static int ascii_pair_to_byte(const char *pair)
    {
        unsigned char* data = (unsigned char*)pair;
        int result;
        
        if (data[1] < 'A')
        {
            result = data[1] - '0';
        }
        else
        {
            result = data[1] - 'A' + 10;
        }
        
        if (data[0] < 'A')
        {
            result += (data[0] - '0') << 4;
        }
        else
        {
            result += (data[0] - 'A' + 10) << 4;
        }
        
        return result;
    }

    DISALLOW_COPY_AND_ASSIGN(GridConnect);
};

#endif /* _UTILS_GRIDCONNECT_HXX_ */

