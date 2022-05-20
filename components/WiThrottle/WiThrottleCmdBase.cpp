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

#include <AllTrainNodes.hxx>
#include <dcc/DccOutput.hxx>
#include <HttpStringUtils.h>
#include <StringUtils.hxx>

namespace withrottle
{

    WiThrottleClientFlow::WiThrottleCommandBase::WiThrottleCommandBase(WiThrottleClientFlow *throttle,
                                                 WiThrottleCommands type)
        : StateFlow<Buffer<WiThrottleClientFlow::ThrottleCommand>, QList<1>>(throttle->service()),
          throttleFlow_(throttle)
    {
        register_command_type(type);
    }

    WiThrottleClientFlow::WiThrottleCommandBase::~WiThrottleCommandBase()
    {
        throttleFlow_->commandDispatcher_.unregister_handler_all(this);
    }

    void WiThrottleClientFlow::WiThrottleCommandBase::register_command_type(WiThrottleCommands type)
    {
        throttleFlow_->commandDispatcher_.register_handler(this, type, WiThrottleCommands::TYPE_MASK);
    }

    void WiThrottleClientFlow::WiThrottleCommandBase::deregister_command_type(WiThrottleCommands type)
    {
        throttleFlow_->commandDispatcher_.register_handler(this, type, WiThrottleCommands::TYPE_MASK);
    }

    StateFlowBase::Action WiThrottleClientFlow::WiThrottleCommandBase::logged_response(Callback next)
    {
        LOG(CONFIG_WITHROTTLE_LOGGING,
            "[WiThrottleCommandBase fd:%d] Sending: %s", throttleFlow_->fd_,
            sendBuf_.c_str());
        return write_repeated(&helper_, throttleFlow_->fd_, sendBuf_.data(),
            sendBuf_.length(), next);
    }

    void WiThrottleClientFlow::WiThrottleCommandBase::verify_locomotive_node(openlcb::NodeID node_id)
    {
        // TODO: this should really send out a train search request
        if (throttleFlow_->server_->node_->iface()->lookup_local_node(node_id) == nullptr)
        {
            LOG(INFO,
                "[WiThrottleCommandBase fd:%d] Node:%s not yet created, "
                "trying to create.", throttleFlow_->fd_,
                esp32cs::node_id_to_string(node_id).c_str());
            auto train_server = Singleton<commandstation::AllTrainNodes>::instance();
            dcc::TrainAddressType type = dcc::TrainAddressType::UNSUPPORTED;
            uint32_t address =  0;
            if (openlcb::TractionDefs::legacy_address_from_train_node_id(node_id, &type, &address))
            {
                if (type == dcc::TrainAddressType::DCC_SHORT_ADDRESS)
                {
                    train_server->allocate_node(commandstation::DccMode::DCC_128, address);
                }
                else if (type == dcc::TrainAddressType::DCC_LONG_ADDRESS)
                {
                    train_server->allocate_node(commandstation::DccMode::DCC_128_LONG_ADDRESS, address);
                }
            }
        }
    }

} // namespace withrottle