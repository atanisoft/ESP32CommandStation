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
 * \file ConfigUpdateService.hxx
 *
 * Base class for the configuration listeners to register themself.
 *
 * @author Balazs Racz
 * @date 13 June 2015
 */

#ifndef _UTILS_CONFIGUPDATESERVICE_HXX_
#define _UTILS_CONFIGUPDATESERVICE_HXX_

#include "utils/Singleton.hxx"

class ConfigUpdateListener;

/// Virtual interface for the config update listeners to register themselves
/// for receiving configuration updates.
class ConfigUpdateService : public Singleton<ConfigUpdateService>
{
public:
    /// Adds a config update listener to be called upon configuration
    /// updates. Should be called before the startup of the stack in order to
    /// ensure that the initial load will be successful.
    ///
    /// @param listener pointer to the implementation that needs to listen to
    /// config updates.
    ///
    virtual void register_update_listener(ConfigUpdateListener *listener) = 0;
    /// Removes a config update listener. Requires: the listener has been
    /// inserted before using \ref register_update_listener.
    ///
    /// @param listener pointer to the implementation that needs to be removed.
    ///
    virtual void unregister_update_listener(ConfigUpdateListener *listener) = 0;

    /// Executes an update in response to the configuration having changed.
    virtual void trigger_update() = 0;
};

#endif // _UTILS_CONFIGUPDATESERVICE_HXX_
