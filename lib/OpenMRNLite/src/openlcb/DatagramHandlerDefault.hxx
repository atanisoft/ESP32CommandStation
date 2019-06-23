/** \copyright
 * Copyright (c) 2014, Balazs Racz
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
 * \file DatagramHandlerDefault.hxx
 *
 * Control flow for incoming datagram handlers.
 *
 * @author Balazs Racz
 * @date 11 Feb 2014
 */

#ifndef _OPENLCB_DATAGRAMHANDLERDEFAULT_HXX_
#define _OPENLCB_DATAGRAMHANDLERDEFAULT_HXX_

#include "openlcb/Datagram.hxx"

namespace openlcb
{

/** Base class with utility functionality that implements some common
 * functionality. Implementations should override the entry() function to
 * handle new incoming datagrams, then they must eventually call respond_ok or
 * respond_reject. */
class DefaultDatagramHandler : public DatagramHandlerFlow
{
protected:
    DefaultDatagramHandler(DatagramService *if_datagram)
        : DatagramHandlerFlow(if_datagram)
    {
    }

public:
    DatagramService *dg_service()
    {
        return static_cast<DatagramService *>(service());
    }

protected:
    /** Sends a DATAGRAM_OK response to the datagram originator node. Call this
     * from the user handler. The flow will end up in the ok_response_sent()
     * state.
     *
     * @param flags is the 1-byte payload of the DATAGRAM_OK message.*/
    Action respond_ok(uint8_t flags)
    {
        responseMti_ = Defs::MTI_DATAGRAM_OK;
        responseErrorCode_ = flags;
        return allocate_and_call(
            dg_service()->iface()->addressed_message_write_flow(),
            STATE(send_ok_response));
    }

private:
    Action send_ok_response()
    {
        auto *b = get_allocation_result(
            dg_service()->iface()->addressed_message_write_flow());
        b->data()->reset(responseMti_, message()->data()->dst->node_id(),
                         message()->data()->src,
                         Payload(1, (char)(responseErrorCode_ & 0xff)));
        dg_service()->iface()->addressed_message_write_flow()->send(b);
        return call_immediately(STATE(ok_response_sent));
    }

protected:
    /** Sends a DATAGRAM_REJECT response to the datagram originator node. Call
     * this from the user handler. The flow will not return to the caller, but
     * release the incoming datagram and return to wait for a new datagram.
     *
     * @param error_code is the 2-byte error code in the DATAGRAM_REJECT
     * message.*/
    Action respond_reject(uint16_t error_code)
    {
        responseMti_ = Defs::MTI_DATAGRAM_REJECTED;
        responseErrorCode_ = error_code;
        return allocate_and_call(
            dg_service()->iface()->addressed_message_write_flow(),
            STATE(send_reject_response));
    }

private:
    Action send_reject_response()
    {
        auto *b = get_allocation_result(
            dg_service()->iface()->addressed_message_write_flow());
        b->data()->reset(responseMti_, message()->data()->dst->node_id(),
                         message()->data()->src,
                         error_to_buffer(responseErrorCode_));
        dg_service()->iface()->addressed_message_write_flow()->send(b);
        return release_and_exit();
    }

protected:
    /** This state is where the handling will end up after a respond_ok
     * call. The user is responsible to eventually doing release and exit(). */
    virtual Action ok_response_sent()
    {
        return release_and_exit();
    }

    /** @returns the size of the incoming datagram payload. */
    size_t size()
    {
        return message()->data()->payload.size();
    }

    /** @returns the incoming datagram payload. Byte zero will be the datagram
     * ID. */
    const uint8_t *payload()
    {
        return reinterpret_cast<const uint8_t *>(
            message()->data()->payload.data());
    }

private:
    uint16_t responseErrorCode_;
    openlcb::Defs::MTI responseMti_;
};

} // namespace

#endif // _OPENLCB_DATAGRAMHANDLERDEFAULT_HXX_
