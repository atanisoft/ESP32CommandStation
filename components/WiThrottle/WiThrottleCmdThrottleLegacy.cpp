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

#include <dcc/DccOutput.hxx>
#include <HttpStringUtils.h>
#include <TrainDatabase.h>

namespace withrottle
{
    using commandstation::Symbols;
    using dcc::TrainAddressType;
    using openlcb::SpeedType;
    using openlcb::TractionDefs;
    using openlcb::TractionThrottle;
    using openlcb::TractionThrottleCommands;

    WiThrottleClientFlow::WiThrottleCommandLocomotiveLegacy::WiThrottleCommandLocomotiveLegacy(WiThrottleClientFlow *throttle)
        : WiThrottleCommandBase(throttle, WiThrottleCommands::PRIMARY)
    {
        register_command_type(WiThrottleCommands::SECONDARY);
    }

    WiThrottleClientFlow::WiThrottleCommandLocomotiveLegacy::~WiThrottleCommandLocomotiveLegacy()
    {
        deregister_command_type(WiThrottleCommands::SECONDARY);
    }

    StateFlowBase::Action WiThrottleClientFlow::WiThrottleCommandLocomotiveLegacy::entry()
    {
        throttleKey_ = message()->data()->command;
        uint32_t address = 0;
        throttle_ = throttleFlow_->get_throttle(throttleKey_, &address);

        if (throttle_ == nullptr)
        {
            sendBuf_ =
                StringPrintf("HMESP32CS: Failed to allocate throttle%s",
                REQUEST_EOL_CHARACTER_NL);
            return write_repeated(&helper_, throttleFlow_->fd_,
                sendBuf_.data(), sendBuf_.length(), STATE(done));
        }

        if (!throttle_->is_train_assigned() && (
                message()->data()->payload[0] != 'L' &&
                message()->data()->payload[0] != 'S'
            ))
        {
            LOG_ERROR("[WiThrottleClient: %d] No locomotive has been assigned!",
                throttleFlow_->fd_);
            return yield_and_call(STATE(done));
        }

        switch(message()->data()->payload[0])
        {
            case 'V':           // velocity
            {
                auto speed = throttle_->get_speed();
                speed.set_dcc_128(std::stoi(message()->data()->payload.substr(1)));
                throttle_->set_speed(speed);
                LOG(INFO, "[WiThrottleClient: %d] Speed set to:%d",
                    throttleFlow_->fd_, speed.get_dcc_128());
                break;
            }
            case 'X':           // e-Stop
            {
                throttle_->set_emergencystop();
                LOG(INFO, "[WiThrottleClient: %d] eStop!", throttleFlow_->fd_);
                break;
            }
            case 'F':           // function
            case 'f':           // function (forced)
            {
                uint8_t state = (message()->data()->payload[1] - '0');
                uint8_t fn = std::stoi(message()->data()->payload.substr(2));
                throttle_->set_fn(fn, state);
                LOG(INFO, "[WiThrottleClient: %d] FN(%d):%s",
                    throttleFlow_->fd_, fn, state? "On" : "Off");
                break;
            }
            case 'R':           // direction
            {
                auto speed = throttle_->get_speed();
                speed.set_direction(message()->data()->payload[1] != '0');
                throttle_->set_speed(speed);
                LOG(INFO, "[WiThrottleClient: %d] Direction:%s",
                    throttleFlow_->fd_,
                    speed.direction() == SpeedType::REVERSE ? "Rev" : "Fwd");
                break;
            }
            case 'r':           // release
            case 'd':           // dispatch
            {
                auto node = throttle_->target_node();
                TrainAddressType addr_type;

                if(TractionDefs::legacy_address_from_train_node_id(node, &addr_type, &address_))
                {
                    return invoke_subflow_and_wait(throttle_, STATE(loco_released),
                        TractionThrottleCommands::RELEASE_TRAIN);
                }
                return yield_and_call(STATE(done));
            }
            case 'L':           // Long address
            {
                address_ = std::stoi(message()->data()->payload.substr(1));
                addressType_ = TrainAddressType::DCC_LONG_ADDRESS;
                auto node_id = TractionDefs::train_node_id_from_legacy(
                    addressType_, address_);
                return invoke_subflow_and_wait(throttle_, STATE(loco_assigned),
                   TractionThrottleCommands::ASSIGN_TRAIN, node_id, 0);
                break;
            }
            case 'S':           // Short address
            {
                address_ = std::stoi(message()->data()->payload.substr(1));
                addressType_ = TrainAddressType::DCC_SHORT_ADDRESS;
                auto node_id = TractionDefs::train_node_id_from_legacy(
                    addressType_, address_);
                return invoke_subflow_and_wait(throttle_, STATE(loco_assigned),
                   openlcb::TractionThrottleCommands::ASSIGN_TRAIN, node_id, 0);
                break;
            }
            case 'I':           // IDLE
            {
                auto speed = throttle_->get_speed();
                speed.set_dcc_128(0);
                throttle_->set_speed(speed);
                LOG(INFO, "[WiThrottleClient: %d] Idle", throttleFlow_->fd_);
                break;
            }
            case 'm':           // momentary function
            {
                uint8_t state = (message()->data()->payload[1] - '0');
                uint8_t fn = std::stoi(message()->data()->payload.substr(2));
                throttle_->set_fn(fn, state);
                LOG(INFO, "[WiThrottleClient: %d] FN(%d):%s",
                    throttleFlow_->fd_, fn, state? "On" : "Off");
                break;
            }
            case 'q':           // query state
            {
                break;
            }
            default:
                LOG_ERROR("[WiThrottleClient: %d] Unrecognized action: %c",
                    throttleFlow_->fd_, message()->data()->payload[0]);
                return yield_and_call(STATE(command_not_recognized));
        }
        return yield_and_call(STATE(done));
    }

    StateFlowBase::Action WiThrottleClientFlow::WiThrottleCommandLocomotiveLegacy::loco_assigned()
    {
        auto *m = full_allocation_result(throttle_);
        bool error = (m->data()->resultCode != openlcb::Defs::ERROR_CODE_OK);
        m->unref();
        if (error)
        {
            sendBuf_ =
                StringPrintf("HMESP32CS: Failed to assign locomotive:%d%s",
                    address_, REQUEST_EOL_CHARACTER_NL);
            return write_repeated(&helper_, throttleFlow_->fd_,
                sendBuf_.data(), sendBuf_.length(), STATE(done));
        }
        LOG(INFO, "[WiThrottleClient: %d] Assigned locomotive:%d",
            throttleFlow_->fd_, address_);
        return invoke_subflow_and_wait(throttle_, STATE(loco_state_loaded),
            TractionThrottleCommands::LOAD_STATE);
    }

    StateFlowBase::Action WiThrottleClientFlow::WiThrottleCommandLocomotiveLegacy::loco_released()
    {
        auto *m = full_allocation_result(throttle_);
        bool error = (m->data()->resultCode != openlcb::Defs::ERROR_CODE_OK);
        m->unref();
        if (error)
        {
            sendBuf_ =
                StringPrintf("HMESP32CS: Failed to release locomotive:%d%s",
                    address_, REQUEST_EOL_CHARACTER_NL);
            return write_repeated(&helper_, throttleFlow_->fd_,
                sendBuf_.data(), sendBuf_.length(), STATE(done));
        }
        LOG(INFO, "[WiThrottleClient: %d] Released locomotive:%d",
            throttleFlow_->fd_, address_);
        sendBuf_ = StringPrintf("%cNot Set%s", throttleKey_,
            REQUEST_EOL_CHARACTER_NL);
        throttleFlow_->release_throttle(throttle_);
        return write_repeated(&helper_, throttleFlow_->fd_, sendBuf_.data(),
            sendBuf_.length(), STATE(done));
    }

    StateFlowBase::Action WiThrottleClientFlow::WiThrottleCommandLocomotiveLegacy::loco_state_loaded()
    {
        auto *m = full_allocation_result(throttle_);
        bool error = (m->data()->resultCode != openlcb::Defs::ERROR_CODE_OK);
        m->unref();
        Callback nextState = STATE(send_function_labels);
        if (error)
        {
            sendBuf_ =
                StringPrintf("HMESP32CS: Failed to load locomotive:%d state%s",
                    address_, REQUEST_EOL_CHARACTER_NL);
            nextState = STATE(done);
        }
        else
        {
            sendBuf_ = StringPrintf("%c%c%d%s", throttleKey_,
                addressType_ == TrainAddressType::DCC_LONG_ADDRESS ? 'L' : 'S',
                address_, REQUEST_EOL_CHARACTER_NL);
        }
        return write_repeated(&helper_, throttleFlow_->fd_, sendBuf_.data(),
            sendBuf_.length(), nextState);
    }

    StateFlowBase::Action WiThrottleClientFlow::WiThrottleCommandLocomotiveLegacy::send_function_labels()
    {
        auto train_db = Singleton<esp32cs::Esp32TrainDatabase>::instance();
        auto train_index = train_db->get_index(address_);
        if (train_index >= 0)
        {
            auto command = message()->data()->command;
            sendBuf_ = StringPrintf("R%c29%s%c%d%s", 
                command == WiThrottleCommands::PRIMARY ? 'F' : 'S',
                FIELD_DELIMITER,
                addressType_ == TrainAddressType::DCC_LONG_ADDRESS ? 'L' : 'S',
                address_, COMMAND_PAYLOAD_DELIMITER);
            
            auto train_entry = train_db->get_entry(train_index);
            // send function label
            for (int fn = 0; fn < commandstation::DCC_MAX_FN; fn++)
            {
                auto label = train_entry->get_function_label(fn);
                sendBuf_.append(COLLECTION_DELIMITER);
                switch (label)
                {
                    case Symbols::FN_NONEXISTANT:
                    case Symbols::FN_UNKNOWN:
                    default:
                        break;
                    case Symbols::LIGHT:
                        sendBuf_.append("Light");
                        break;
                    case Symbols::BELL:
                        sendBuf_.append("Bell");
                        break;
                    case Symbols::HORN:
                        sendBuf_.append("Horn");
                        break;
                    case Symbols::WHISTLE:
                        sendBuf_.append("Whistle");
                        break;
                    case Symbols::SHUNT:
                        sendBuf_.append("Shunting mod");
                        break;
                    case Symbols::MOMENTUM:
                        sendBuf_.append("Momentum");
                        break;
                    case Symbols::SMOKE:
                        sendBuf_.append("Whistle");
                        break;
                    case Symbols::MUTE:
                        sendBuf_.append("Mute");
                        break;
                    case Symbols::GENERIC:
                        sendBuf_.append(StringPrintf("F%d", fn));
                        break;
                    case Symbols::COUPLER:
                        sendBuf_.append("Coupler");
                        break;
                }
            }
            return write_repeated(&helper_, throttleFlow_->fd_,
                sendBuf_.data(), sendBuf_.length(),
                STATE(send_function_states));
        }
        return yield_and_call(STATE(send_function_states));
    }

    StateFlowBase::Action WiThrottleClientFlow::WiThrottleCommandLocomotiveLegacy::send_function_states()
    {
        sendBuf_ = StringPrintf("RPF%s%c", FIELD_DELIMITER, throttleKey_);
        for (int fn = 0; fn < commandstation::DCC_MAX_FN; fn++)
        {
            uint16_t state = throttle_->get_fn(fn);
            if (state == TractionThrottle::FN_NOT_KNOWN)
            {
                state = 0;
            }
            sendBuf_.append(StringPrintf("%sF%d%02d%s",
                COLLECTION_DELIMITER, state, fn, FIELD_DELIMITER));
        }
        sendBuf_.append(REQUEST_EOL_CHARACTER_NL);
        return write_repeated(&helper_, throttleFlow_->fd_, sendBuf_.data(),
            sendBuf_.length(), STATE(done));
    }

    StateFlowBase::Action WiThrottleClientFlow::WiThrottleCommandLocomotiveLegacy::done()
    {
        return release_and_exit();
    }

    StateFlowBase::Action WiThrottleClientFlow::WiThrottleCommandLocomotiveLegacy::command_not_recognized()
    {
        sendBuf_ = StringPrintf("HMESP32CS: Command not understood.%s",
            REQUEST_EOL_CHARACTER_NL);
        return write_repeated(&helper_, throttleFlow_->fd_, sendBuf_.data(),
            sendBuf_.length(), STATE(done));
    }
} // namespace withrottle