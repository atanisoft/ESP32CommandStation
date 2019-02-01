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
 * \file DatagramDefs.hxx
 * Constants useful for a datagram implementation.
 *
 * @author Stuart W. Baker
 * @date 5 October 2013
 */

#ifndef _OPENLCB_DATAGRAMDEFS_HXX_
#define _OPENLCB_DATAGRAMDEFS_HXX_

#include <stdint.h>

namespace openlcb
{

/** Static constants and functions related to the Datagram protocol. */
struct DatagramDefs
{
    /** All known datagram protocols */
    enum Protocol
    {
        LOG_REQUEST   = 0x01, /**< request a placement into the log */
        LOG_REPLY     = 0x02, /**< reply to a @ref LOG_REQUEST */
        CONFIGURATION = 0x20, /**< configuration message */
        REMOTE_BUTTON = 0x21, /**< remote button input */
        DISPLAY_PROTOCOL = 0x28, /**< place on display */
        TRAIN_CONTROL = 0x30, /**< operation of mobile nodes */
    };

    /** Various public datagram constants. */
    enum
    {
        MAX_SIZE = 72, /**< maximum size in bytes of a datagram */

    };
    
    /** Possible flags for a successful receipt (received okay) of a Datagram.
     */
    enum Flag
    {
        FLAGS_NONE    = 0x00, /**< no flags set */
        REPLY_PENDING = 0x80, /**< A reply is pending */
        
        TIMEOUT_NONE  = 0x00, /**< no timeout */
        TIMEOUT_2     = 0x01, /**< 2 second timeout */
        TIMEOUT_4     = 0x02, /**< 4 second timeout */
        TIMEOUT_8     = 0x03, /**< 8 second timeout */
        TIMEOUT_16    = 0x04, /**< 16 second timeout */
        TIMEOUT_32    = 0x05, /**< 32 second timeout */
        TIMEOUT_64    = 0x06, /**< 64 second timeout */
        TIMEOUT_128   = 0x07, /**< 128 second timeout */
        TIMEOUT_256   = 0x08, /**< 256 second timeout */
        TIMEOUT_512   = 0x09, /**< 512 second timeout */
        TIMEOUT_1024  = 0x0A, /**< 1024 second timeout */
        TIMEOUT_2048  = 0x0B, /**< 2048 second timeout */
        TIMEOUT_4096  = 0x0C, /**< 4096 second timeout */
        TIMEOUT_8192  = 0x0D, /**< 8192 second timeout */
        TIMEOUT_16384 = 0x0E, /**< 16384 second timeout */
        TIMEOUT_32768 = 0x0F, /**< 32768 second timeout */
        
        TIMEOUT_MASK  = 0x0F, /**< Mask for reply timeout */
    };
    
    /** Possible error codes for a rejected datagram.
     */
    enum Error
    {
        RESEND_OK          = 0x2000, /**< We can try to resend the datagram. */
        TRANSPORT_ERROR    = 0x6000, /**< Transport error occurred. */
        BUFFER_UNAVAILABLE = 0x2020, /**< Buffer unavailable error occurred. */
        OUT_OF_ORDER       = 0x2040, /**< Out of order error occurred. */
        PERMANENT_ERROR    = 0x1000, /**< Permanent error occurred. */
        SRC_NOT_PERMITTED  = 0x1020, /**< Source not permitted error occurred. */
        NOT_ACCEPTED       = 0x1040, /**< Destination node does not accept datagrams of any kind. */
        UNIMPLEMENTED      = 0x1080, /**< NON_STANDARD The feature or command requested is not implemented by the target node. */
        INVALID_ARGUMENTS  = 0x1010, /**< NON_STANDARD Invalid or unparseable arguments. */
    };

    /** We can try to resend the datagram.
     * @param error error number
     * @return true or false
     */
    static bool resend_ok(uint16_t error)
    {
        return error & RESEND_OK;
    }

    /** Transport error occurred.
     * @param error error number
     * @return true or false
     */
    static bool transport_error(uint16_t error)
    {
        return error & TRANSPORT_ERROR;
    }

    /** Buffer unavailable error occurred.
     * @param error error number
     * @return true or false
     */
    static bool buffer_unavailable(uint16_t error)
    {
        return error & BUFFER_UNAVAILABLE;
    }
    
    /** Determine if the protocol ID is represented by one, two, or six bytes.
     * @param _protocol protocol ID to interrogate
     * @return number of bytes representing the protocol
     */
    static unsigned int protocol_size(uint64_t protocol)
    {
        return (((protocol & PROTOCOL_SIZE_MASK) == PROTOCOL_SIZE_6) ? 6 :
                ((protocol & PROTOCOL_SIZE_MASK) == PROTOCOL_SIZE_2) ? 2 : 1);
    }

private:
    /** Constants used by the protocol_size function. */
    enum
    {
        PROTOCOL_SIZE_2    = 0xE0, /**< possible return value for @ref protocol_size */
        PROTOCOL_SIZE_6    = 0xF0, /**< possible return value for @ref protocol_size */
        PROTOCOL_SIZE_MASK = 0xF0, /**< mask used when determining protocol size */
    };

    /** Do not instantiate this class, ever. */
    DatagramDefs();
};


}  // namespace openlcb


#endif // _OPENLCB_DATAGRAMDEFS_HXX_

