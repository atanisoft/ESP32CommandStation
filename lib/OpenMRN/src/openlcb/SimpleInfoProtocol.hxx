/** \copyright
 * Copyright (c) 2014, Balazs Racz
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
 * \file SimpleInfoProtocol.hxx
 *
 * Shared code among SNIP and similar protocols.
 *
 * @author Balazs Racz
 * @date 22 Jul 2013
 */

#ifndef _OPENLCB_SIMPLEINFOPROTOCOL_HXX_
#define _OPENLCB_SIMPLEINFOPROTOCOL_HXX_

#include <fcntl.h>
#include <unistd.h>

#include "openlcb/If.hxx"
#include "executor/StateFlow.hxx"

namespace openlcb
{

struct SimpleInfoDescriptor;

/** Use a Buffer<SimpleInfoResponse> to send a SNIP response to a particular
 * destination host. */
struct SimpleInfoResponse
{
    SimpleInfoResponse()
        : src(nullptr)
        , dst({0, 0})
        , descriptor(nullptr)
    {
    }
    /** Initializes the fields of this message to be a response to an
     * NMRAnetMessage (i.e., flips destination and source from that
     * message). The descriptor array must end with an EOF entry and must stay
     * alive so long as the message is not sent. */
    void reset(const GenMessage *msg_to_respond,
               const SimpleInfoDescriptor *desc,
               Defs::MTI response_mti)
    {
        HASSERT(msg_to_respond->dstNode);
        src = msg_to_respond->dstNode;
        dst = msg_to_respond->src;
        descriptor = desc;
        mti = response_mti;
    }
    /** Source node to send the response from. */
    Node *src;
    /** MTI of the response to be sent. */
    Defs::MTI mti;
    /** Destination node to send the response to. */
    NodeHandle dst;
    /** Descriptor of payload to send. */
    const SimpleInfoDescriptor *descriptor;
};

/** This structure defines how to piece together a reply to a Simple Info
 * request, sich as SNIP. These structures will come in an array, where each
 * member of the array defines a variable number of bytes to append to the end
 * of the reply message. The last structure should be an EOF. */
struct SimpleInfoDescriptor
{
    enum Cmd
    {
        END_OF_DATA = 0,  //< End of the data sequence
        C_STRING = 1,     //< pointer points to a C-string with NULL term. arg, if non-zero, specifies max length including the 0 byte.
        LITERAL_BYTE = 2, //< transfer argument as byte (e.g. version). Pointer, if not null, will be checked to match the transferred value.
        CHAR_ARRAY = 3,   //< len = argument, pointer in data. No null termination.
        FILE_C_STRING = 4,//< pointer is filename, offset is used for file offset. arg is the maximum length including nul termination.
        FILE_LITERAL_BYTE = 5,//< transfer argument as byte (e.g. version). pointer is filename, offset is used for file offset, used to check against the expected value.
        FILE_CHAR_ARRAY = 6,//< pointer is filename, offset is used for file offset, arg is length of value, no null termination.
    };

    uint8_t cmd; //< Command. See enum Cmd.
    uint8_t arg; //< Argument to the command.
    uint16_t arg2; //< Additional argument.
    /** Points to a string if the command requires so. */
    const char *data;
};

/// Base class for the SimpleInfoFlow.
typedef StateFlow<Buffer<SimpleInfoResponse>, QList<1>> SimpleInfoFlowBase;

/// StateFlow for sending out medium-sized data payloads like the Simple Node
/// Ident Info protocol.
///
/// The flow works by assembling a medium-sized payload according to a specific
/// pattern. The pattern consists of a sequence of literal bytes, fixed-length
/// strings, C strings, etc.
///
/// Usage:
///
/// Create a static array of SimpleInfoDescriptor structures to define the
/// response that needs to be pieced together. Add a MessageHandlerFlow to
/// receive the simple X info request messages. When such a request arrives,
/// extract the source node handle, and send a SimpleInfoResponse message to
/// the SimpleInfoFlow with the node handle and the descriptor array
/// pointer. The SimpleInfoFlow will assemble, fragment and send the response
/// message.
///
/// Example: see @SNIPHandler.
class SimpleInfoFlow : public SimpleInfoFlowBase
{
public:
    /** Creates a simple ident flow handler.
     *
     * @param max_bytes_per_message tells how many bytes we should package in
     * one outgoing buffer. Responses longer than this will be sent as multiple
     * separate messages. Set this to 6 on CAN to completely avoid raw
     * pagination. Set to 255 to create one memory buffer that will then be
     * split into frames by the low-level interface. Maximum value is 255.
     * @param use_continue_bits should be true if we should instruct the
     * low-level interface to use the continuation-pending bits so long as we
     * have pending bytes. This will make the messages be pieced together at
     * the receiving end into one message. Setting this to false will send a
     * reply in multiple messages. */
    SimpleInfoFlow(Service *s, unsigned max_bytes_per_message = 255,
                   bool use_continue_bits = true)
        : SimpleInfoFlowBase(s)
        , maxBytesPerMessage_(
              max_bytes_per_message > 255 ? 255 : max_bytes_per_message)
        , useContinueBits_(use_continue_bits ? 1 : 0)
    {
    }

    ~SimpleInfoFlow()
    {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
            fileName_ = nullptr;
        }
    }

private:
    Action entry() OVERRIDE
    {
        HASSERT(message()->data()->src);
        HASSERT(message()->data()->descriptor);
        entryOffset_ = 0;
        byteOffset_ = 0;
        isFirstMessage_ = 1;
        update_for_next_entry();
        return call_immediately(STATE(continue_send));
    }

    const SimpleInfoDescriptor &current_descriptor()
    {
        return message()->data()->descriptor[entryOffset_];
    }

    /** @returns true if there are no more bytes to send. */
    bool is_eof()
    {
        return (current_descriptor().cmd == SimpleInfoDescriptor::END_OF_DATA);
    }

    /** Assumes that the current descriptor is a file argument. Opens the
     * filename in the argument, seeks it to the offset. */
    void open_and_seek_next_file()
    {
        const SimpleInfoDescriptor &d = current_descriptor();
        const char* new_file_name = reinterpret_cast<const char*>(d.data);
        HASSERT(new_file_name);
        if (!(fileName_ == new_file_name ||
              (fileName_ && d.data && !strcmp(fileName_, new_file_name)))) {
            fileName_ = new_file_name;
            if (fd_ >= 0) {
                ::close(fd_);
            }
            fd_ = ::open(fileName_, O_RDONLY);
            HASSERT(fd_ >= 0);
        }
        int ret = lseek(fd_, d.arg2, SEEK_SET);
        HASSERT(ret != -1);
    }

    /** Call this function after updating entryOffset_. */
    void update_for_next_entry()
    {
        const SimpleInfoDescriptor &d = current_descriptor();
        switch (d.cmd) {
            case SimpleInfoDescriptor::C_STRING:
            {
                byteOffset_ = 0;
                currentLength_ = strlen((const char *)d.data) + 1;
                if (d.arg && d.arg < currentLength_) {
                    // Clips too long messages.
                    currentLength_ = d.arg;
                    LOG(INFO, "message clipped to length %d", currentLength_);
                }
                break;
            }
#if (!defined(ARDUINO)) || defined(ESP32)
            case SimpleInfoDescriptor::FILE_CHAR_ARRAY:
                open_and_seek_next_file();
                // fall through
#endif
            case SimpleInfoDescriptor::CHAR_ARRAY:
                byteOffset_ = 0;
                currentLength_ = d.arg;
                HASSERT(currentLength_);
                break;
#if (!defined(ARDUINO)) || defined(ESP32)
            case SimpleInfoDescriptor::FILE_LITERAL_BYTE:
            {
                open_and_seek_next_file();
                break;
            }
            case SimpleInfoDescriptor::FILE_C_STRING:
            {
                open_and_seek_next_file();
                currentLength_ = d.arg;
                byteOffset_ = 0;
                break;
            }
#endif // NOT ARDUINO, YES ESP32
            default:
                currentLength_ = 0;
        }
    }

    /** Returns the next byte from the file data, advancing the file offset. */
    uint8_t file_read_current_byte()
    {
        HASSERT(fd_ >= 0);
        uint8_t ret;
        int result = ::read(fd_, &ret, 1);
        HASSERT(result >= 0);
        if (result == 0) ret = 0;
        return ret;
    }

    /** Returns the current byte in the stream of data. */
    uint8_t current_byte()
    {
        const SimpleInfoDescriptor &d = current_descriptor();
        switch (d.cmd)
        {
            case SimpleInfoDescriptor::END_OF_DATA:
                return 0;
            case SimpleInfoDescriptor::LITERAL_BYTE: {
                if (d.data) {
                    HASSERT(d.arg == *d.data);
                }
                return d.arg;
            }
            case SimpleInfoDescriptor::C_STRING:
                if (byteOffset_ >= currentLength_ - 1)
                {
                    return 0;
                }
                else
                {
                    return d.data[byteOffset_];
                }
            case SimpleInfoDescriptor::CHAR_ARRAY:
                if (byteOffset_ >= currentLength_)
                {
                    return 0;
                }
                else
                {
                    return d.data[byteOffset_];
                }
            case SimpleInfoDescriptor::FILE_C_STRING:
            {
                uint8_t fdata = file_read_current_byte();
                if (!fdata) {
                    currentLength_ = byteOffset_;
                }
                if (byteOffset_ >= currentLength_ - 1)
                {
                    fdata = 0;
                }
                return fdata;
            }
            case SimpleInfoDescriptor::FILE_LITERAL_BYTE:
            {
                uint8_t fdata = file_read_current_byte();
                HASSERT(d.arg == fdata);
                return d.arg;
            }
            case SimpleInfoDescriptor::FILE_CHAR_ARRAY:
            {
                return file_read_current_byte();
            }
            default:
                DIE("Unexpected descriptor type.");
        }
    }

    /** Increments to the next byte in the stream of data. */
    void step_byte()
    {
        const SimpleInfoDescriptor &d = current_descriptor();
        switch (d.cmd)
        {
            case SimpleInfoDescriptor::END_OF_DATA:
                return;
            case SimpleInfoDescriptor::LITERAL_BYTE:
            case SimpleInfoDescriptor::FILE_LITERAL_BYTE:
                break;
            case SimpleInfoDescriptor::C_STRING:
            case SimpleInfoDescriptor::FILE_C_STRING:
            case SimpleInfoDescriptor::CHAR_ARRAY:
            case SimpleInfoDescriptor::FILE_CHAR_ARRAY:
                if (++byteOffset_ >= currentLength_ && currentLength_)
                {
                    break;
                }
                else
                {
                    return;
                }
            default:
                DIE("Unexpected descriptor type.");
        }
        ++entryOffset_;
        update_for_next_entry();
    }

    Action continue_send()
    {
        if (is_eof())
        {
            return release_and_exit();
        }
        return allocate_and_call(
            message()->data()->src->iface()->addressed_message_write_flow(),
            STATE(fill_buffer));
    }

    Action fill_buffer()
    {
        auto *b = get_allocation_result(message()
                                            ->data()
                                            ->src->iface()
                                            ->addressed_message_write_flow());
        const SimpleInfoResponse &r = *message()->data();
        b->data()->reset(r.mti, r.src->node_id(), r.dst, EMPTY_PAYLOAD);
        for (uint8_t offset = 0; offset < maxBytesPerMessage_ && !is_eof();
             ++offset, step_byte())
        {
            b->data()->payload.push_back(current_byte());
        }
        b->data()->set_flag_dst(GenMessage::WAIT_FOR_LOCAL_LOOPBACK);
        if (useContinueBits_)
        {
            if (!is_eof())
            {
                b->data()->set_flag_dst(
                    GenMessage::DSTFLAG_NOT_LAST_MESSAGE);
            }

            if (isFirstMessage_)
            {
                isFirstMessage_ = 0;
            }
            else
            {
                b->data()->set_flag_dst(
                    GenMessage::DSTFLAG_NOT_FIRST_MESSAGE);
            }
        }
        b->set_done(n_.reset(this));
        message()->data()->src->iface()->addressed_message_write_flow()->send(
            b);
        return wait_and_call(STATE(continue_send));
    }

    /** Configuration option. See constructor. */
    uint8_t maxBytesPerMessage_;
    /** Configuration option. See constructor. */
    uint8_t useContinueBits_ : 1;

    /** Whether this is the first reply message we are sending out. Used with
     * the continuation feature. */
    uint8_t isFirstMessage_ : 1;
    /** Tells which descriptor entry we are processing. */
    uint8_t entryOffset_ : 5;

    /** Byte offset within a descriptor entry. */
    uint8_t byteOffset_;
    /** Total / max length of the current block. This is typically strlen() + 1
     * (including the terminating zero, if any). */
    uint8_t currentLength_;

    /// Last file name we opened.
    const char* fileName_{nullptr};
    /// fd of the last file we opened.
    int fd_{-1};

    BarrierNotifiable n_;
};

} // namespace openlcb

#endif // _OPENLCB_SIMPLEINFOPROTOCOL_HXX_
