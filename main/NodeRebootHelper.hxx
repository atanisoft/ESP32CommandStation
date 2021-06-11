/** \copyright
 * Copyright (c) 2021, Mike Dunston
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
 * \file NodeRebootHelper.hxx
 *
 * Helper for triggering a remote reboot of the Esp32C3OlcbIO Board.
 *
 * @author Mike Dunston
 * @date 26 May 2021
 */

#include <openlcb/SimpleStack.hxx>
#include <utils/logging.h>
#include <utils/Singleton.hxx>
#include <esp_system.h>

#include "FileSystem.hxx"

namespace esp32cs
{

/// Utility class for rebooting the node safely.
class NodeRebootHelper : public Singleton<NodeRebootHelper>
{
public:
    /// Constructor.
    ///
    /// @param stack is the @ref SimpleCanStack to shutdown.
    /// @param fd is the file handle for the configuration file.
    NodeRebootHelper(openlcb::SimpleCanStack *stack, int fd)
                   : stack_(stack), fd_(fd)
    {
    }

    /// Initiates an orderly shutdown of all components before restarting the
    /// ESP32.
    void reboot()
    {
        // make sure we are not called from the executor thread otherwise there
        // will be a deadlock
        HASSERT(os_thread_self() != stack_->executor()->thread_handle());
        LOG(INFO, "[Reboot] Shutting down LCC executor...");
        stack_->executor()->sync_run([&]()
        {
            close(fd_);
            unmount_fs();
            // restart the node
            LOG(INFO, "[Reboot] Restarting!");
            esp_restart();
        });
    }
private:
    /// @ref SimpleCanStack to be shutdown.
    openlcb::SimpleCanStack *stack_;

    /// Configuration file descriptor to be closed prior to shutdown.
    int fd_;
};

} // namespace esp32cs