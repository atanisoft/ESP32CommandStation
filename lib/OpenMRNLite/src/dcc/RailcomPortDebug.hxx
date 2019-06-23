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
 * \file RailcomPortDebug.hxx
 *
 * Flows that can attach to Railcom Hubs and perform various debugging outputs.
 *
 * @author Balazs Racz
 * @date 6 Feb 2016
 */

#ifndef _DCC_RAILCOMPORTDEBUG_HXX
#define _DCC_RAILCOMPORTDEBUG_HXX

#include "dcc/RailcomHub.hxx"

namespace dcc
{

/** Registers as a member ofthe railcom hub. Formats all incoming railcom
 * packet in a human-visible way, and puts into the logging facility. */
class RailcomPrintfFlow : public dcc::RailcomHubPortInterface
{
public:
    /// Constructor. @param source is the railcom hub to listen to.
    RailcomPrintfFlow(dcc::RailcomHubFlow *source)
        : parent_(source)
    {
        source->register_port(this);
    }

    ~RailcomPrintfFlow()
    {
        parent_->unregister_port(this);
    }

private:
    /// Helper function to turn some railcom data into string. @param data is
    /// the pointer to the data. @param len tells how many bytes are there
    /// valid. @return textual representation of that data (debugging).
    string display_railcom_data(const uint8_t *data, int len)
    {
        static char buf[200];
        int ofs = 0;
        HASSERT(len <= 6);
        for (int i = 0; i < len; ++i)
        {
            ofs += sprintf(buf + ofs, "0x%02x (0x%02x), ", data[i],
                dcc::railcom_decode[data[i]]);
        }
        uint8_t type = (dcc::railcom_decode[data[0]] >> 2);
        if (len == 2)
        {
            uint8_t payload = dcc::railcom_decode[data[0]] & 0x3;
            payload <<= 6;
            payload |= dcc::railcom_decode[data[1]];
            switch (type)
            {
                case dcc::RMOB_ADRLOW:
                    ofs += sprintf(buf + ofs, "adrlow=%d", payload);
                    break;
                case dcc::RMOB_ADRHIGH:
                    ofs += sprintf(buf + ofs, "adrhigh=%d", payload);
                    break;
                case dcc::RMOB_EXT:
                    ofs += sprintf(buf + ofs, "ext=%d", payload);
                    break;
                case dcc::RMOB_DYN:
                    ofs += sprintf(buf + ofs, "dyn=%d", payload);
                    break;
                case dcc::RMOB_SUBID:
                    ofs += sprintf(buf + ofs, "subid=%d", payload);
                    break;
                default:
                    ofs += sprintf(buf + ofs, "type-%d=%d", type, payload);
            }
        }
        return string(buf, ofs);
    }

    /// Incoming railcom data.
    ///
    /// @param d railcom buffer.
    /// @param prio priority
    void send(Buffer<dcc::RailcomHubData> *d, unsigned prio) OVERRIDE
    {
        AutoReleaseBuffer<dcc::RailcomHubData> rb(d);
        dcc::Feedback &fb = *d->data();
        if (fb.feedbackKey <= 1000)
            return;
        if (fb.ch1Size && fb.channel != 0xff)
        {
            LOG(INFO, "Railcom %x CH1 data(%" PRIuPTR "): %s", fb.channel,
                fb.feedbackKey,
                display_railcom_data(fb.ch1Data, fb.ch1Size).c_str());
        }
        if (fb.ch2Size && fb.channel != 0xff)
        {
            LOG(INFO, "Railcom %x CH2 data(%" PRIuPTR "): %s", fb.channel,
                fb.feedbackKey,
                display_railcom_data(fb.ch2Data, fb.ch2Size).c_str());
        }
    }

    /// Flow to which we are registered.
    dcc::RailcomHubFlow *parent_;
};

} // namespace dcc

namespace openlcb
{

/** This flow proxies all incoming railcom traffic to the openlcb bus in
 *  non-standard messages. It should be used for debugging only. 
 *
 *  It also proxies all occupancy information to a downstream port. That's not
 *  super useful but a bit more efficient than registering the downstream port
 *  into the railcom hub port.
 */
class RailcomToOpenLCBDebugProxy : public dcc::RailcomHubPort
{
public:
    RailcomToOpenLCBDebugProxy(dcc::RailcomHubFlow *parent, Node *node,
        dcc::RailcomHubPort *occupancy_port)
        : dcc::RailcomHubPort(parent->service())
        , parent_(parent)
        , node_(node)
        , occupancyPort_(occupancy_port)
    {
        parent_->register_port(this);
    }

    RailcomToOpenLCBDebugProxy(Node *node)
        : dcc::RailcomHubPort(node->iface())
        , parent_(nullptr)
        , node_(node)
        , occupancyPort_(nullptr)
    {
    }
    
    ~RailcomToOpenLCBDebugProxy()
    {
        if (parent_)
        {
            parent_->unregister_port(this);
        }
    }

    Action entry() override
    {
        if (message()->data()->channel == 0xff)
        {
            if (occupancyPort_) {
                occupancyPort_->send(transfer_message());
            } else {
                release();
            }
            return exit();
        }
        if (message()->data()->channel == 0xfe)
        {
            return release_and_exit();
        }
        if (message()->data()->ch1Size)
        {
            return allocate_and_call(
                node_->iface()->global_message_write_flow(),
                STATE(ch1_msg_allocated));
        }
        else
        {
            return call_immediately(STATE(maybe_send_ch2));
        }
    }

    Action ch1_msg_allocated()
    {
        auto *b =
            get_allocation_result(node_->iface()->global_message_write_flow());

        b->data()->reset(
            static_cast<openlcb::Defs::MTI>(openlcb::Defs::MTI_XPRESSNET + 2),
            node_->node_id(), string());
        b->data()->payload.push_back(message()->data()->channel | 0x10);
        b->data()->payload.append(
            (char *)message()->data()->ch1Data, message()->data()->ch1Size);
        node_->iface()->global_message_write_flow()->send(b);

        return call_immediately(STATE(maybe_send_ch2));
    }

    Action maybe_send_ch2()
    {
        if (message()->data()->ch2Size)
        {
            return allocate_and_call(
                node_->iface()->global_message_write_flow(),
                STATE(ch2_msg_allocated));
        }
        else
        {
            return release_and_exit();
        }
    }

    Action ch2_msg_allocated()
    {
        auto *b =
            get_allocation_result(node_->iface()->global_message_write_flow());

        b->data()->reset(
            static_cast<openlcb::Defs::MTI>(openlcb::Defs::MTI_XPRESSNET + 3),
            node_->node_id(), string());
        b->data()->payload.push_back(message()->data()->channel | 0x20);
        b->data()->payload.append(
            (char *)message()->data()->ch2Data, message()->data()->ch2Size);
        node_->iface()->global_message_write_flow()->send(b);

        return release_and_exit();
    }

    dcc::RailcomHubFlow *parent_{nullptr};
    Node *node_;
    dcc::RailcomHubPort *occupancyPort_;
};

} // namespace openlcb

#endif // _DCC_RAILCOMPORTDEBUG_HXX
