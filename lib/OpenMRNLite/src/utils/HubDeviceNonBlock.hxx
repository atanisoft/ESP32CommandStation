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
 * \file HubDevice.hxx
 * Components for Hubs to connect to physical devices.
 *
 * @author Balazs Racz
 * @date 26 Apr 2014
 */

#ifndef _UTILS_HUBDEVICENONBLOCK_HXX_
#define _UTILS_HUBDEVICENONBLOCK_HXX_

// Nonblocking hubdevice only works on FreeRTOS.
#ifdef __FreeRTOS__

#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>

#include "executor/StateFlow.hxx"
#include "freertos/can_ioctl.h"
#include "utils/Hub.hxx"

extern int ioctl(int fd, unsigned long int key, ...);

template <class HFlow> class HubDeviceNonBlock : public Destructable, private Atomic, public Service
{
public:
    HubDeviceNonBlock(HFlow *hub, const char *path)
        : Service(hub->service()->executor())
        , fd_(::open(path, O_RDWR | O_NONBLOCK))
        , hub_(hub)
        , readFlow_(this)
        , writeFlow_(this)
    {
        HASSERT(fd_ >= 0);
        hub_->register_port(write_port());
    }

    virtual ~HubDeviceNonBlock()
    {
        hub_->unregister_port(write_port());
    }

    HFlow *hub()
    {
        return hub_;
    }

    typename HFlow::port_type *write_port()
    {
        return &writeFlow_;
    }

    int fd()
    {
        return fd_;
    }

protected:
    class ReadFlow : public StateFlowBase
    {
    public:
        ReadFlow(HubDeviceNonBlock *device)
            : StateFlowBase(device)
            , b_(nullptr)
        {
            this->start_flow(STATE(allocate_buffer));
        }

        HubDeviceNonBlock *device()
        {
            return static_cast<HubDeviceNonBlock *>(this->service());
        }

        void notify() OVERRIDE
        {
            service()->executor()->add(this, 0);
        }

        void notify_from_isr() OVERRIDE
        {
            // We override to priority zero.
            service()->executor()->add_from_isr(this, 0);
        }

        Action allocate_buffer()
        {
            return this->allocate_and_call(device()->hub(), STATE(try_read));
        }

        Action try_read()
        {
            b_ = this->get_allocation_result(device()->hub());
            b_->data()->skipMember_ = device()->write_port();
            return this->call_immediately(STATE(retry_read));
        }

        Action retry_read()
        {
            HASSERT(b_);
            int ret =
                ::read(device()->fd(), b_->data()->data(), b_->data()->size());
            if (ret <= 0)
            {
                // We are blocked. There is no race condition here, because the
                // contract of the ioctl is that if there is any data in the
                // queuy, they will immediately call us.
                HASSERT(::ioctl(device()->fd(), CAN_IOC_READ_ACTIVE, this) == 0);
                return this->wait();
            }
            else
            {
                HASSERT((unsigned)ret == b_->data()->size());
                device()->hub()->send(b_, 0);
                b_ = nullptr;
                return this->call_immediately(STATE(allocate_buffer));
            }
        }

    private:
        typename HFlow::buffer_type *b_;
    };

    typedef StateFlow<typename HFlow::buffer_type, QList<1>> WriteFlowBase;
    class WriteFlow : public WriteFlowBase
    {
    public:
        WriteFlow(HubDeviceNonBlock *dev)
            : WriteFlowBase(dev)
        {
        }

        HubDeviceNonBlock *device()
        {
            return static_cast<HubDeviceNonBlock *>(this->service());
        }

        void notify() OVERRIDE
        {
            this->service()->executor()->add(this, 0);
        }

        void notify_from_isr() OVERRIDE
        {
            this->service()->executor()->add_from_isr(this, 0);
        }

        StateFlowBase::Action entry() OVERRIDE
        {
            bufferPos_ = static_cast<uint8_t*>(this->message()->data()->data());
            len_ = this->message()->data()->size();
            return this->call_immediately(STATE(try_write));
        }

        StateFlowBase::Action try_write()
        {
            ssize_t ret = ::write(device()->fd(), bufferPos_, len_);
            if (!ret)
            {
                // We are blocked. There is no race condition here, because the
                // contract of the ioctl is that if there is any data in the
                // queuy, they will immediately call us.
                HASSERT(::ioctl(device()->fd(), CAN_IOC_WRITE_ACTIVE, this) == 0);
                return this->wait();
            }
            else if (ret > 0)
            {
                len_ -= ret;
                bufferPos_ += ret;
                if (!len_)
                {
                    return this->release_and_exit();
                }
                else
                {
                    return this->again();
                }
            }
            else
            {
                // error!
                DIE("Error writing.");
            }
        }

    private:
        uint8_t *bufferPos_;
        ssize_t len_;
    };

protected:
    /** The device file descriptor. */
    int fd_;
    HFlow *hub_;
    ReadFlow readFlow_;
    WriteFlow writeFlow_;
};

#endif // __FreeRTOS__
#endif // _UTILS_HUBDEVICENONBLOCK_HXX_
