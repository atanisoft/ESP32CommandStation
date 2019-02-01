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
 * \file ConfigUpdateListener.hxx
 *
 * Interface for receiving configuration from EEPROM, including callback at
 * boot time and updates to this configuration when the config is modified.
 *
 * @author Balazs Racz
 * @date 31 May 2014
 */

#ifndef _UTILS_CONFIGUPDATELISTENER_HXX_
#define _UTILS_CONFIGUPDATELISTENER_HXX_

#include "utils/QMember.hxx"
#include "executor/Notifiable.hxx"

/// Abstract class for components that need to receive configuration from
/// EEPROM. Configuration update comes in two flavors: initial load and refresh
/// (upon potential change). Each configuration update call supplies the file
/// desriptor of the EEPROM file, which is already opened for reading.
///
/// Config listeners provide feedback to the system in two forms: they notify
/// the system in case the configuration change can be applied only with a
/// reboot of the hardware node, or whether an application-level
/// reinitialization sequence is needed.
class ConfigUpdateListener : public QMember
{
public:
    /// Specifies what additional steps are needed to apply the new
    /// configuration.
    enum UpdateAction
    {
        /// No additional step is necessary.
        UPDATED = 0,
        /// Need to perform application-level reinitialization. (In case of
        /// OpenLCB this means an identify all events procedure is needed.)
        REINIT_NEEDED,
        /// Need to reboot the hardware.
        REBOOT_NEEDED,
    };

    /// Notifies the component that there is new configuration available for
    /// loading.
    ///
    /// The call is made on the main executor, so the call must not
    /// block. Reading the given EEPROM device should be fine. Asynchronous
    /// operations may be implemented by using a special return status RETRY:
    /// the runner will call the same method on the same component once more
    /// after the done callback is invoked. This allows implementing state
    /// machines.
    ///
    /// @param fd is the file descriptor for the EEPROM file. The current
    /// offset in this file is unspecified, callees must do lseek.
    /// @param initial_load is true if this is the first load upon starting the
    /// binary.
    /// @param done must be notified when the call and its dependent actions
    /// are complete. No other configuration component will be called until the
    /// done callback is invoked.
    ///
    /// @return any necessary action. If returns UPDATED, then assumes that the
    /// configuration change was applied. If returns RETRY, then the same call
    /// will be made again after the notifiable is called. If return
    /// REINIT_NEEDED or REBOOT_NEEDED then at the end of the configuration
    /// update process the node will be reinitialized or rebooted accordingly.
    virtual UpdateAction apply_configuration(int fd, bool initial_load,
                                             BarrierNotifiable *done) = 0;

    /// Clears configuration file and resets the configuration settings to
    /// factory value.
    ///
    /// @param fd is the file descriptor for the EEPROM file. The current
    /// offset in this file is unspecified, callees must do lseek.
    virtual void factory_reset(int fd) = 0;
};


/// Implementation of ConfigUpdateListener that registers itself in the
/// constructor and unregisters itself in the destructor.
class DefaultConfigUpdateListener : public ConfigUpdateListener
{
public:
    DefaultConfigUpdateListener();
    ~DefaultConfigUpdateListener();
};


#endif // _UTILS_CONFIGUPDATELISTENER_HXX_
