/** \copyright
 * Copyright (c) 2013, Balazs Racz
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
 * \file IfImpl.cxx
 *
 * Implementation details for the asynchronous NMRAnet interfaces. This file
 * should only be needed in hardware interface implementations.
 *
 * @author Balazs Racz
 * @date 4 Dec 2013
 */

#include "openlcb/IfImpl.hxx"

namespace openlcb
{

StateFlowBase::Action WriteFlowBase::addressed_entry()
{
    if (nmsg()->dst.id)
    {
        nmsg()->dstNode = async_if()->lookup_local_node_handle(nmsg()->dst);
        if (nmsg()->dstNode)
        {
            return call_immediately(STATE(send_to_local_node));
        }
    }
    return send_to_hardware();
}

StateFlowBase::Action WriteFlowBase::send_to_local_node()
{
    unsigned prio = priority();
    // We do not believe infinite priority and use the one from the MTI
    // instead.
    if (prio >= (1 << 16))
    {
        prio = message()->data()->priority();
    }
    async_if()->dispatcher()->send(transfer_message(), prio);
    return call_immediately(STATE(send_finished));
}

StateFlowBase::Action WriteFlowBase::global_entry()
{
    if (!message()->data()->has_flag_dst(
            GenMessage::WAIT_FOR_LOCAL_LOOPBACK))
    {
        // We do not pass on the done notifiable with the loopbacked message.
        message()->set_done(nullptr);
    }
    unsigned loopback_prio = message()->data()->priority();
    async_if()->dispatcher()->send(transfer_message(), loopback_prio);
    return release_and_exit();
}

#if 0
///@todo(balazs.racz) move this code to its final location when it is ready.
/** Send an ident info reply message.
 * @param dst destination Node ID to respond to
 */
void Node::ident_info_reply(NodeHandle dst)
{
    /* macro for condensing the size calculation code */
#define ADD_STRING_SIZE(_str, _max)                                            \
    {                                                                          \
        if ((_str))                                                            \
        {                                                                      \
            size_t len = strlen((_str));                                       \
            size += len > (_max) ? (_max) : len;                               \
        }                                                                      \
    }

    /* macro for condensing the string insertion  code */
#define INSERT_STRING(_str, _max)                                              \
    {                                                                          \
        if ((_str))                                                            \
        {                                                                      \
            size_t len = strlen((_str));                                       \
            len = len > (_max) ? (_max) : len;                                 \
            memcpy(pos, (_str), len);                                          \
            pos[len] = '\0';                                                   \
            pos += len + 1;                                                    \
        }                                                                      \
        else                                                                   \
        {                                                                      \
            pos[0] = '\0';                                                     \
            pos++;                                                             \
        }                                                                      \
    }
    
    /* we make this static so that it does not use up stack */
    /** @todo (Stuart Baker) if this could ever be accessed from more than one
     * thread, we will need a lock
     */
    static char ident[8+40+40+20+20+62+63];
    char       *pos = ident;
    size_t      size = 8;

    ADD_STRING_SIZE(MANUFACTURER, 40);
    ADD_STRING_SIZE(model, 40);
    ADD_STRING_SIZE(HARDWARE_REV, 20);
    ADD_STRING_SIZE(SOFTWARE_REV, 20);
    ADD_STRING_SIZE(userName, 62);
    ADD_STRING_SIZE(userDescription, 63);

    pos[0] = SIMPLE_NODE_IDENT_VERSION_A;
    pos++;
    
    INSERT_STRING(MANUFACTURER, 40);
    INSERT_STRING(model, 40);
    INSERT_STRING(HARDWARE_REV, 20);
    INSERT_STRING(SOFTWARE_REV, 20);

    pos[0] = SIMPLE_NODE_IDENT_VERSION_B;
    pos++;

    INSERT_STRING(userName, 62);
    INSERT_STRING(userDescription, 63);

    for (int index = 0; size; )
    {
        size_t segment_size = size > 6 ? 6 : size;
        Buffer *buffer = buffer_alloc(6);
        memcpy(buffer->start(), ident + index, segment_size);
        buffer->advance(segment_size);
        write(Defs::MTI_IDENT_INFO_REPLY, dst, buffer);
        size -= segment_size;
        index += segment_size;
    }
}

/** Send an protocols supported reply message.
 * @param dst destination Node ID to respond to
 */
void Node::protocol_support_reply(NodeHandle dst)
{
    /** @todo (Stuart Baker) this needs to be updated as additional protocols
     * are supported
     */
    uint64_t protocols = PROTOCOL_IDENTIFICATION |
                         DATAGRAM |
                         EVENT_EXCHANGE |
                         SIMPLE_NODE_INFORMATION |
                         MEMORY_CONFIGURATION |
                         CDI;

    Buffer *buffer = buffer_alloc(6);
    uint8_t *bytes = (uint8_t*)buffer->start();
    
    bytes[0] = (protocols >> 40) & 0xff;
    bytes[1] = (protocols >> 32) & 0xff;
    bytes[2] = (protocols >> 24) & 0xff;
    bytes[3] = (protocols >> 16) & 0xff;
    bytes[4] = (protocols >>  8) & 0xff;
    bytes[5] = (protocols >>  0) & 0xff;
    
    buffer->advance(6);

    write(Defs::MTI_PROTOCOL_SUPPORT_REPLY, dst, buffer);
}

#endif // if 0

} // namespace openlcb
