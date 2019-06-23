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
 * \file JSHubPort.hxx
 *
 * Shared base class for all javascript "device drivers".
 *
 * @author Balazs Racz
 * @date 13 Sep 2015
 */

#ifndef _UTILS_JSHUBPORT_HXX_
#define _UTILS_JSHUBPORT_HXX_

#ifdef __EMSCRIPTEN__

#include <emscripten.h>
#include <emscripten/bind.h>
#include <emscripten/val.h>

#include "utils/Hub.hxx"
#include "utils/GridConnectHub.hxx"

class JSHubPort : public HubPortInterface
{
public:
    JSHubPort(unsigned long parent, emscripten::val send_fn)
        : parent_(reinterpret_cast<CanHubFlow *>(parent))
        , sendFn_(send_fn)
        , gcHub_(parent_->service())
        , gcAdapter_(
              GCAdapterBase::CreateGridConnectAdapter(&gcHub_, parent_, false))
    {
        extern int JSHubPort_debug_port_num;
        myPortNum_ = JSHubPort_debug_port_num++;
        HASSERT(sendFn_.typeof().as<std::string>() == "function");
        gcHub_.register_port(this);
        active_ = true;
    }

    ~JSHubPort()
    {
        pause();
    }

    void abandon()
    {
        pause();
        if (gcAdapter_.get() == nullptr && !gcHub_.is_waiting())
        {
            delete this;
        }
    }

    void pause()
    {
        if (active_)
        {
            gcHub_.unregister_port(this);
            if (gcAdapter_->shutdown())
            {
                gcAdapter_.reset();
            }
            active_ = false;
        }
    }

    void resume()
    {
        if (!active_)
        {
            gcHub_.register_port(this);
            gcAdapter_.reset(GCAdapterBase::CreateGridConnectAdapter(
                &gcHub_, parent_, false));
            active_ = true;
        }
    }

    void send(HubPortInterface::message_type *buffer,
        unsigned priority = UINT_MAX) OVERRIDE
    {
        sendFn_((string &)*buffer->data());
        buffer->unref();
    }

    void recv(string s)
    {
        auto *b = gcHub_.alloc();
        b->data()->assign(s);
        b->data()->skipMember_ = this;
        gcHub_.send(b);
    }

    int get_port_num() {
        return myPortNum_;
    }

private:
    CanHubFlow *parent_;
    emscripten::val sendFn_;
    HubFlow gcHub_;
    std::unique_ptr<GCAdapterBase> gcAdapter_;
    bool active_;
    int myPortNum_;
};

EMSCRIPTEN_BINDINGS(js_hub_module)
{
    emscripten::class_<JSHubPort>("JSHubPort")
        .constructor<unsigned long, emscripten::val>()
        .function("recv", &JSHubPort::recv)
        .function("pause", &JSHubPort::pause)
        .function("abandon", &JSHubPort::abandon)
        .function("get_port_num", &JSHubPort::get_port_num)
        .function("resume", &JSHubPort::resume);
}

#endif // __EMSCRIPTEN__
#endif // _UTILS_JSHUBPORT_HXX_
