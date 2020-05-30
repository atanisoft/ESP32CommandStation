/** \copyright
 * Copyright (c) 2016, Balazs Racz
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
 * \file BootloaderPort.hxx
 *
 * Helper class for connecting a bootloader HAL to a standard CAN hub. Used for
 * unittests and bootloaders that do not need to be small.
 *
 * @author Balazs Racz
 * @date 19 Jun 2016
 */

#ifndef _OPENLCB_BOOTLOADERPORT_HXX_
#define _OPENLCB_BOOTLOADERPORT_HXX_

#include "utils/Hub.hxx"

namespace openlcb {

/** Proxy class for sending canbus traffic to the Bootloader HAL. */
class BootloaderPort : public CanHubPort
{
public:
    BootloaderPort(Service* service)
        : CanHubPort(service)
    {
        is_waiting_ = false;
    }

    bool is_waiting()
    {
        return is_waiting_;
    }

    virtual Action entry()
    {
        AtomicHolder h(this);
        is_waiting_ = true;
        return wait_and_call(STATE(sent));
    }

    Action sent()
    {
        return release_and_exit();
    }

    bool read_can_frame(struct can_frame *frame)
    {
        {
            AtomicHolder h(this);
            if (is_waiting_)
            {
                *frame = *message()->data();
                is_waiting_ = false;
            }
            else
            {
                return false;
            }
        }
        notify();
        return true;
    }

private:
    /** True if an incoming message is ready for dispatching and the current
     * flow is waiting for a notify. */
    bool is_waiting_ = false;
};

}  // namespace openlcb

#endif //_OPENLCB_BOOTLOADERPORT_HXX_

