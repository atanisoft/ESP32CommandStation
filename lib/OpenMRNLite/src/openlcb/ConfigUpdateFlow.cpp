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
 * \file ConfigUpdateFlow.hxx
 *
 * Implementation of the notification flow for all config update
 * listeners. This flow calls each update listener and performs the necessary
 * actions.
 *
 * @author Balazs Racz
 * @date 13 June 2015
 */

#include "openlcb/ConfigUpdateFlow.hxx"
#include <fcntl.h>

namespace openlcb
{

int ConfigUpdateFlow::open_file(const char *path)
{
    if (fd_ >= 0) return fd_;
    if (!path)
    {
        fd_ = -1;
    }
    else
    {
        fd_ = ::open(path, O_RDWR);
        HASSERT(fd_ >= 0);
    }
    return fd_;
}

void ConfigUpdateFlow::init_flow()
{
    trigger_update();
}

void ConfigUpdateFlow::factory_reset()
{
    for (auto it = listeners_.begin(); it != listeners_.end(); ++it) {
        it->factory_reset(fd_);
    }
    for (auto it = pendingListeners_.begin(); it != pendingListeners_.end();
         ++it)
    {
        it->factory_reset(fd_);
    }
}

void ConfigUpdateFlow::register_update_listener(ConfigUpdateListener *listener)
{
    AtomicHolder h(this);
    pendingListeners_.push_front(listener);
    if (is_state(exit().next_state()))
    {
        start_flow(STATE(do_initial_load));
    }
}

void ConfigUpdateFlow::unregister_update_listener(
    ConfigUpdateListener *listener)
{
    AtomicHolder h(this);
    for (auto it = listeners_.begin(); it != listeners_.end();)
    {
        if (it.operator->() == listener)
        {
            listeners_.erase(it);
            continue;
        }
        ++it;
    }
    for (auto it = pendingListeners_.begin(); it != pendingListeners_.end();)
    {
        if (it.operator->() == listener)
        {
            pendingListeners_.erase(it);
            continue;
        }
        ++it;
    }
    // We invalidated the iterators due to the erase.
    nextRefresh_ = listeners_.begin();
}

extern const char *const CONFIG_FILENAME __attribute__((weak)) = nullptr;
extern const size_t CONFIG_FILE_SIZE __attribute__((weak)) = 0;

} // namespace openlcb
