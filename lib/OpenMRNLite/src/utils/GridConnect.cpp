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
 * \file GridConnect.cxx
 * This file implements the logic for encoding and decoding grid connect
 * protocol.
 *
 * @author Stuart W. Baker
 * @date 28 September 2013
 */

#include <unistd.h>
#include <cerrno>

#include "utils/GridConnect.hxx"

ssize_t GridConnect::encode(struct can_frame *frame, unsigned char buf[])
{
    /* allocate a buffer for the Grid connect format */
    buf[0] = ':';

    ssize_t index;

    /* handle the identifier */
    if (IS_CAN_FRAME_EFF(*frame))
    {
        buf[1] = 'X';
        uint32_t id = GET_CAN_FRAME_ID_EFF(*frame);
        for (int i = 9; i >= 2; --i, id >>= 4)
        {
            buf[i] = nibble_to_ascii(id);
        }
        index = 10;
    }
    else
    {
        buf[1] = 'S';
        uint16_t id = GET_CAN_FRAME_ID(*frame);
        for (int i = 4; i >= 2; --i, id >>= 4)
        {
            buf[i] = nibble_to_ascii(id);
        }
        index = 5;
    }

    /* handle remote or normal */
    if (IS_CAN_FRAME_RTR(*frame))
    {
        buf[index] = 'R';
    }
    else
    {
        buf[index] = 'N';
    }
    index++;
    
    /* handle the data */
    for (int i = 0; i < frame->can_dlc; i++, index += 2)
    {
        buf[index + 0] = nibble_to_ascii(frame->data[i] >> 4);
        buf[index + 1] = nibble_to_ascii(frame->data[i]);
    }
    
    /* stop character */
    buf[index] = ';';
    index++;

    return index;
}



/** Write a CAN packet to Grid Connect interface in double format
 * @param fd file descriptor for device
 * @param data can data to write
 * @param len length of data, should be a multiple of sizeof(struct can_frame)
 * @param doub true if this is a double write
 * @return number of bytes written, or -1 with errno set
 */
ssize_t GridConnect::write_generic(int fd, const void *data, size_t len, bool doub)
{
    struct can_frame *can_frame = (struct can_frame*)data;
    size_t            remaining = len;
    
    /* check for len that is multiple of a can_frame size */
    if ((len % sizeof(struct can_frame)) != 0)
    {
        errno = EINVAL;
        return -1;
    }
    
    /* allocate a buffer for the Grid connect format */
    unsigned char buf[56];

    /* while there are packets to transmit */
    while (remaining)
    {
        ssize_t size = encode(can_frame, buf);

        if (doub)
        {
            /* duplicate each byte */
            for (ssize_t i = (size - 1), j = ((size * 2) - 1);
                 i > 0; --i, j -= 2)
            {
                buf[j] = buf[i];
                buf[j - 1] = buf[i];
            }
            buf[0] = buf[1] = '!';

            size *= 2;
        }
        
        /* write the formated packet */
        ssize_t result = ::write(fd, buf, size);
        
        if (result != size)
        {
            return -1;
        }

        /* get ready for next packet */
        remaining -= sizeof(struct can_frame);
        can_frame++;
    } /* while (remaining) */

    return len;
}

/** Read a CAN packet to Grid Connect interface in single or double format.
 * @param fd file descriptor for device
 * @param data location to read can data into
 * @param len length of data, should be a multiple of sizeof(struct can_frame)
 * @param doub true if this is a double read
 * @return number of bytes read, or -1 with errno set
 */
ssize_t GridConnect::read_generic(int fd, void *data, size_t len, bool doub)
{
    struct can_frame *can_frame = (struct can_frame*)data;
    size_t            remaining = len;
    
    const char SOF = doub ? '!' : ':';
    const int factor = doub ? 2 : 1;
    
    /* check for len that is multiple of a can_frame size */
    if ((len % sizeof(struct can_frame)) != 0)
    {
        errno = EINVAL;
        return -1;
    }

    /* while there are packets to receive */
    while (remaining)
    {
        /* Though the maximum packet size is 28, we need an extra byte
         * in the case that we are working with a double encoded frame
         */
        char buf[29];

        /** @todo this decode method is simple, but could be optimized. */
        for ( ; /* until we find a CAN frame */ ; )
        {
            do
            {
                ssize_t result = ::read(fd, buf, factor);
                if (result < factor)
                {
                    return -1;
                }
            } while(buf[0] != SOF);
            
            /* We have a start of frame, now lets get the rest of the packet */
            int i;
            for (i = 1; i < 28; ++i)
            {
                ssize_t result = ::read(fd, buf + i, factor);
                if (result < factor)
                {
                    return -1;
                }
                if (buf[i] == ';')
                {
                    /* found an end of frame */
                    break;
                }
            }
            if (i != 28)
            {
                /* we found the end of frame character */
                break;
            }
        } /* for ( ; until we find a CAN frame ; ) */
        
        int index;
        /* determine if the frame is standard or extended */
        if (buf[1] == 'X')
        {
            SET_CAN_FRAME_EFF(*can_frame);
            uint32_t id;
            id  = ascii_pair_to_byte(buf + 2) << 24;
            id += ascii_pair_to_byte(buf + 4) << 16;
            id += ascii_pair_to_byte(buf + 6) << 8;
            id += ascii_pair_to_byte(buf + 8) << 0;
            SET_CAN_FRAME_ID_EFF(*can_frame, id);
            index = 10;
        }
        else if (buf[1] == 'S')
        {
            CLR_CAN_FRAME_EFF(*can_frame);
            uint16_t id;
            id  = ascii_pair_to_byte(buf + 2) << 4;
            id |= ascii_pair_to_byte(buf + 3) << 0;
            SET_CAN_FRAME_ID(*can_frame, id);
            index = 5;
        }
        else
        {
            /* unexpected character, try again */
            continue;
        }
        
        /* determine if the frame is normal or remote */
        if (buf[index] == 'N')
        {
            CLR_CAN_FRAME_RTR(*can_frame);
        }
        else if (buf[index] == 'R')
        {
            SET_CAN_FRAME_RTR(*can_frame);
        }
        else
        {
            /* unexpected character, try again */
            continue;
        }
        index++;
        
        /* grab the data */
        int i;
        for (i = 0; buf[index] != ';'; index += 2, i++)
        {
            can_frame->data[i] = ascii_pair_to_byte(buf + index);
        }
        can_frame->can_dlc = i;

        CLR_CAN_FRAME_ERR(*can_frame);
        
        /* get ready for next packet */
        remaining -= sizeof(struct can_frame);
        can_frame++;
    } /* while (remaining) */
    return len;
}

