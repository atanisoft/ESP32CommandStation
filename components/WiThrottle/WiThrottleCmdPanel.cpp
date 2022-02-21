/** \copyright
 * Copyright (c) 2022, Mike Dunston
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
 * \file WiThrottleCommands.cpp
 *
 * WiThrottle commands implementation.
 *
 * @author Mike Dunston
 * @date 8 Feb 2022
 */

/// @file WiThrottle commands implementation.

#include "WiThrottle.hxx"

#include <AccessoryDecoderDatabase.hxx>
#include <HttpStringUtils.h>
#include <openlcb/Defs.hxx>
#include <EventBroadcastHelper.hxx>

namespace withrottle
{
    using esp32cs::AccessoryDecoderDB;
    using esp32cs::EventBroadcastHelper;
    using openlcb::Defs;

    WiThrottleClientFlow::WiThrottleCommandPanel::WiThrottleCommandPanel(WiThrottleClientFlow *throttle)
        : WiThrottleCommandBase(throttle, WiThrottleCommands::PANEL)
    {
    }

    WiThrottleClientFlow::WiThrottleCommandPanel::~WiThrottleCommandPanel()
    {
    }

    StateFlowBase::Action WiThrottleClientFlow::WiThrottleCommandPanel::entry()
    {
        if (message()->data()->payload[0] == 'P' &&
            message()->data()->payload[1] == 'A')       // Track Power
        {
            if (message()->data()->payload[2] == '1')   // Off
            {
                Singleton<EventBroadcastHelper>::instance()->send_event(
                    Defs::EMERGENCY_OFF_EVENT);
                sendBuf_ = StringPrintf("PPA0%s", REQUEST_EOL_CHARACTER_NL);
                return logged_response(STATE(done));
            }
            else                                        // On
            {
                Singleton<EventBroadcastHelper>::instance()->send_event(
                    Defs::CLEAR_EMERGENCY_OFF_EVENT);
                sendBuf_ = StringPrintf("PPA1%s", REQUEST_EOL_CHARACTER_NL);
                return logged_response(STATE(done));
            }
        }
        else if (message()->data()->payload[0] == 'T' &&
                 message()->data()->payload[1] == 'A')  // Accessory
        {
            uint8_t state = message()->data()->payload[2];
            auto accessory = Singleton<AccessoryDecoderDB>::instance();
            uint16_t address = std::stoi(message()->data()->payload.substr(3));
            if (state == 'T')                       // thrown
            {
                accessory->set(address, true);
                state = '4';
            }
            else if (state == 'C' || state == '4')  // closed
            {
                accessory->set(address, false);
                state = '2';
            }
            else if (state == '2')                  // toggle
            {
                if (accessory->toggle(address))
                {
                    state = '4';
                }
                else
                {
                    state = '2';
                }
            }
            else
            {
                return yield_and_call(STATE(command_not_recognized));
            }

            // TODO: move this to client flow via accessorydb subscription
            sendBuf_ = StringPrintf("PTA%c%d%s", state, address,
                REQUEST_EOL_CHARACTER_NL);
            return logged_response(STATE(done));
        }
        else if (message()->data()->payload[0] == 'R' &&
                 message()->data()->payload[1] == 'A')  // Routes
        {
            return yield_and_call(STATE(command_not_supported));    
        }

        return yield_and_call(STATE(done));
    }

    StateFlowBase::Action WiThrottleClientFlow::WiThrottleCommandPanel::done()
    {
        return release_and_exit();
    }

    StateFlowBase::Action WiThrottleClientFlow::WiThrottleCommandPanel::command_not_recognized()
    {
        LOG_ERROR("[WiThrottleCommandPanel fd:%d] unrecognized command: %s",
            throttleFlow_->fd_, message()->data()->payload.c_str());
        sendBuf_ = StringPrintf("HMESP32CS: Command not understood.%s",
            REQUEST_EOL_CHARACTER_NL);
        return logged_response(STATE(done));
    }

    StateFlowBase::Action WiThrottleClientFlow::WiThrottleCommandPanel::command_not_supported()
    {
        sendBuf_ = StringPrintf("HMESP32CS: Command not supported.%s",
            REQUEST_EOL_CHARACTER_NL);
        return logged_response(STATE(done));
    }
} // namespace withrottle