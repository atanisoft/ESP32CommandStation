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

#ifndef _OPENLCB_CONFIGUPDATEFLOW_HXX_
#define _OPENLCB_CONFIGUPDATEFLOW_HXX_

#include "utils/ConfigUpdateListener.hxx"
#include "utils/ConfigUpdateService.hxx"
#include "openlcb/NodeInitializeFlow.hxx"
#include "executor/StateFlow.hxx"

#if !defined (__MACH__)
extern "C" {
/// Called when the node needs to be rebooted.
extern void reboot();
}
#endif

namespace openlcb
{

/// Implementation of the ConfigUpdateService: state flow issuing all the calls
/// to the registered ConfigUpdateListener descendants. This flow also handles
/// any necessary action such as reboot or factory reset. This flow keeps the
/// file descriptor for the config file that's currently open.
class ConfigUpdateFlow : public StateFlowBase,
                         public ConfigUpdateService,
                         private Atomic
{
public:
    ConfigUpdateFlow(If *iface)
        : StateFlowBase(iface)
        , nextRefresh_(listeners_.begin())
        , fd_(-1)
    {
    }

    /// Must be called once before calling anything else. Returns the file
    /// descriptor.
    int open_file(const char *path);
    /// Asynchronously invokes all update listeners with the config FD.
    void init_flow();
    /// Synchronously invokes all update listeners to factory reset.
    void factory_reset();

    void TEST_set_fd(int fd)
    {
        fd_ = fd;
    }

    void trigger_update() override
    {
        AtomicHolder h(this);
        nextRefresh_ = listeners_.begin();
        needsReboot_ = 0;
        needsReInit_ = 0;
        if (is_state(exit().next_state()))
        {
            start_flow(STATE(call_next_listener));
        }
    }

    void register_update_listener(ConfigUpdateListener *listener) override;
    void unregister_update_listener(ConfigUpdateListener *listener) override;
private:
    Action call_next_listener()
    {
        ConfigUpdateListener *l = nullptr;
        {
            AtomicHolder h(this);
            if (nextRefresh_ == listeners_.end())
            {
                return call_immediately(STATE(do_initial_load));
            }
            l = nextRefresh_.operator->();
        }
        ++nextRefresh_;
        return call_listener(l, false);
    }

    Action call_listener(ConfigUpdateListener *l, bool is_initial)
    {
        if (fd_ < 0)
        {
            DIE("CONFIG_FILENAME not specified, or init() was not called, but "
                "there are configuration listeners.");
        }
        ConfigUpdateListener::UpdateAction action =
            l->apply_configuration(fd_, is_initial, n_.reset(this));
        switch (action)
        {
            case ConfigUpdateListener::UPDATED:
            {
                break;
            }
            case ConfigUpdateListener::REINIT_NEEDED:
            {
                needsReInit_ = 1;
                break;
            }
            case ConfigUpdateListener::REBOOT_NEEDED:
            {
                needsReboot_ = 1;
                break;
            }
        }
        return wait();
    }

    Action do_initial_load()
    {
        ConfigUpdateListener *l = nullptr;
        {
            AtomicHolder h(this);
            if (!pendingListeners_.empty())
            {
                l = pendingListeners_.pop_front();
                listeners_.push_front(l);
            }
        }
        if (!l)
        {
            return apply_action();
        }
        return call_listener(l, true);
    }

    Action apply_action()
    {
        /// TODO(balazs.racz) apply the changes reported.
        if (needsReboot_)
        {
#ifdef __FreeRTOS__
            reboot();
#endif
        }
        if (needsReInit_)
        {
            // Takes over ownership of itself, will delete when done.
            new ReinitAllNodes(static_cast<If *>(service()));
        }
        return exit();
    }

    typedef TypedQueue<ConfigUpdateListener> queue_type;
    /// All registered update listeners. Protected by Atomic *this.
    queue_type listeners_;
    /// All listeners that have not yet been added to listeners_ and their
    /// initial load needs to be called.
    queue_type pendingListeners_;
    /// Where are we in the refresh cycle.
    typename queue_type::iterator nextRefresh_;
    /// did anybody request a reboot to happen?
    unsigned needsReboot_ : 1;
    /// did anybody request a node reinit to happen?
    unsigned needsReInit_ : 1;
    int fd_;
    BarrierNotifiable n_;
};

} // namespace openlcb

#endif // _OPENLCB_CONFIGUPDATEFLOW_HXX_
