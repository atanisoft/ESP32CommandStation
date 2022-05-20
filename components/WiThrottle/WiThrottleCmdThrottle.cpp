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
    using commandstation::DccMode;
    using dcc::TrainAddressType;
    using esp32cs::Esp32TrainDatabase;
    using openlcb::Defs;
    using openlcb::SpeedType;
    using openlcb::TractionDefs;
    using openlcb::TractionThrottle;
    using openlcb::TractionThrottleCommands;

    WiThrottleClientFlow::WiThrottleCommandLocomotive::WiThrottleCommandLocomotive(WiThrottleClientFlow *throttle)
        : WiThrottleCommandBase(throttle, WiThrottleCommands::MULTI)
    {
    }

    WiThrottleClientFlow::WiThrottleCommandLocomotive::~WiThrottleCommandLocomotive()
    {
    }

    StateFlowBase::Action WiThrottleClientFlow::WiThrottleCommandLocomotive::entry()
    {
        // Make sure the payload string contains the required delimiter: <;> if
        // not, reject it immediately.
        if (message()->data()->payload.find_first_of(COMMAND_PAYLOAD_DELIMITER) ==
            std::string::npos)
        {
            return yield_and_call(STATE(command_not_recognized));
        }

        throttleKey_ = message()->data()->payload.at(0);
        uint8_t action = message()->data()->payload.at(1);
        // remove the throttle key and action.
        message()->data()->payload.erase(0, 2);

        // Break the command payload at the delimiter for parsing.
        std::pair<string, string> command_parts =
            http::break_string(message()->data()->payload,
                COMMAND_PAYLOAD_DELIMITER);

        // Extract target locomotive address from the first part of the provided
        // payload, if it is '*' the code will default to 
        if (command_parts.first.at(0) == 'L')
        {
            addressTypeChar_ = 'L';
            addressType_ = TrainAddressType::DCC_LONG_ADDRESS;
            address_ = std::stoi(command_parts.first.substr(1));
            driveMode_ = DccMode::DCC_DEFAULT;
        }
        else if (command_parts.first.at(0) == 'S')
        {            
            addressTypeChar_ = 'S';
            addressType_ = TrainAddressType::DCC_SHORT_ADDRESS;
            address_ = std::stoi(command_parts.first.substr(1));
            driveMode_ = DccMode::DCC_DEFAULT;
        }
        else if (command_parts.first.at(0) == '*')
        {
            // command is for all locomotives on this throttle key.
            addressType_ = TrainAddressType::UNSPECIFIED;
            // default to long
            addressTypeChar_ = 'L';
            address_ = 0;
        }
        else
        {
            return yield_and_call(STATE(command_not_recognized));
        }

        // Drop first part of the payload
        message()->data()->payload.assign(command_parts.second);

        if (action == '+')
        {
            // If we haven't parsed the address type by this point the command
            // does not match the protocol documentation, reject it.
            if (addressType_ == TrainAddressType::UNSPECIFIED)
            {
                return yield_and_call(STATE(command_not_recognized));
            }

            // Check if it is request for a specific roster entry, if so look
            // up the roster entry and extract it's address / type rather than
            // the provided values.
            if (!message()->data()->payload.empty() &&
                message()->data()->payload.at(0) == 'E')
            {
                auto train_name = message()->data()->payload.substr(1);
                if (!lookup_roster_entry(train_name))
                {
                    sendBuf_ =
                        StringPrintf("HMESP32CS: Roster entry not found:%s%s",
                        train_name.c_str(), REQUEST_EOL_CHARACTER_NL);
                    return logged_response(STATE(done));
                }
            }

            throttle_ = throttleFlow_->get_throttle(throttleKey_, &address_);
            if (throttle_ == nullptr)
            {
                sendBuf_ =
                    StringPrintf("HMESP32CS: Maximum locomotive count(%d) "
                                "reached, locomotive %d not added.%s",
                    config_withrottle_max_client_locomotives(), address_,
                    REQUEST_EOL_CHARACTER_NL);
                return logged_response(STATE(done));
            }
            auto node_id = TractionDefs::train_node_id_from_legacy(
                addressType_, address_);
            verify_locomotive_node(node_id);
            return invoke_subflow_and_wait(throttle_, STATE(loco_assigned),
                TractionThrottleCommands::ASSIGN_TRAIN, node_id,
                listenForUpdates_);
        }
        if (action == '-')
        {
            // release all assigned locomotives
            if (addressType_ == TrainAddressType::UNSPECIFIED)
            {
                return yield_and_call(STATE(release_next_loco));
            }
            else
            {
                throttle_ =
                    throttleFlow_->get_throttle(throttleKey_, &address_);

                if (throttle_)
                {
                    return invoke_subflow_and_wait(throttle_, 
                        STATE(loco_released),
                        TractionThrottleCommands::RELEASE_TRAIN);
                }
            }
        }
        else if (action == 'S')
        {
            sendBuf_ =
                StringPrintf("HMESP32CS: Locomotive steal not implemented.%s",
                REQUEST_EOL_CHARACTER_NL);
            return logged_response(STATE(done));
        }
        else if (action == 'A')
        {
            return yield_and_call(STATE(parse_loco_action));
        }
        return yield_and_call(STATE(command_not_recognized));
    }

    bool WiThrottleClientFlow::WiThrottleCommandLocomotive::lookup_roster_entry(std::string &entry_name)
    {
        auto train_db = Singleton<Esp32TrainDatabase>::instance();
        auto train = train_db->get_entry(entry_name);
        if (train)
        {
            if (address_ != train->get_legacy_address())
            {
                LOG(WARNING,
                    "[WiThrottleClient: %d] Loco add address override: "
                    "%d -> %d",
                    throttleFlow_->fd_, address_,
                    train->get_legacy_address());
            }
            address_ = train->get_legacy_address();
            driveMode_ = train->get_legacy_drive_mode();
            TrainAddressType new_addr_type;
            if ((driveMode_ & DccMode::DCC_LONG_ADDRESS) ==
                DccMode::DCC_LONG_ADDRESS)
            {
                new_addr_type = TrainAddressType::DCC_LONG_ADDRESS;
                addressTypeChar_ = 'L';
            }
            else
            {
                new_addr_type = TrainAddressType::DCC_SHORT_ADDRESS;
                addressTypeChar_ = 'S';
            }

            if (new_addr_type != addressType_)
            {
                LOG(WARNING,
                    "[WiThrottleClient: %d] Loco add address type "
                    "override: %d -> %d",
                    throttleFlow_->fd_, static_cast<int>(addressType_),
                    static_cast<int>(new_addr_type));
            }
            addressType_ = new_addr_type;
            return true;
        }
        return false;
    }

    StateFlowBase::Action WiThrottleClientFlow::WiThrottleCommandLocomotive::parse_loco_action()
    {
        Callback nextState = STATE(done);
        uint32_t lastAddress = address_;
        // handle 'Q' (quit) immediately and exit early.
        if (message()->data()->payload.at(0) == 'Q')
        {
            throttleFlow_->trigger_shutdown();
            return call_immediately(STATE(done));
        }
        else if (addressType_ == TrainAddressType::UNSPECIFIED)
        {
            if (message()->data()->payload.at(0) == 'C' || // consist
                message()->data()->payload.at(0) == 'c' || // consist
                message()->data()->payload.at(0) == 'E' || // roster entry
                message()->data()->payload.at(0) == 'L' || // set address
                message()->data()->payload.at(0) == 'S')   // set address
            {
                return yield_and_call(STATE(command_not_supported));
            }
            // FIXME: this needs to be fixed as it doesn't work.
            // reset the address and retrieve the next throttle
            address_ = 0;
            nextState = STATE(parse_loco_action);
        }
        throttle_ =
            throttleFlow_->get_throttle(throttleKey_, &address_);
        if (throttle_ == nullptr || (
            addressType_ == TrainAddressType::UNSPECIFIED &&
            lastAddress == address_))
        {
            return yield_and_call(STATE(done));
        }
        if (!throttle_->is_train_assigned() &&
            message()->data()->payload.at(0) != 'L' && // set address
            message()->data()->payload.at(0) != 'S' && // set address
            message()->data()->payload.at(0) != 'E' && // roster entry
            message()->data()->payload.at(0) != 'C' && // consist
            message()->data()->payload.at(0) != 'c')   // consist
        {
            sendBuf_ =
                StringPrintf("HMESP32CS: No locomotive has been assigned to "
                             "this throttle!%s", REQUEST_EOL_CHARACTER_NL);
            return logged_response(STATE(done));
        }
        // extract the address type from the node assigned node id.
        if (addressType_ == TrainAddressType::UNSPECIFIED)
        {
            auto node = throttle_->target_node();
            TrainAddressType type;
            uint32_t address;
            if(TractionDefs::legacy_address_from_train_node_id(
                node, &type, &address))
            {
                if (type == TrainAddressType::DCC_SHORT_ADDRESS)
                {
                    addressTypeChar_ = 'S';
                }
                else
                {
                    addressTypeChar_ = 'L';
                }
            }
            else
            {
                addressTypeChar_ = 'L';
            }
        }

        switch (message()->data()->payload.at(0))
        {
            case 'C':           // Consist
            case 'c':
                break;
            case 'd':           // Dispatch
            case 'r':           // Release
                return invoke_subflow_and_wait(throttle_, STATE(loco_released),
                    TractionThrottleCommands::RELEASE_TRAIN);
                break;
            case 'E':           // Set address by roster entry?
                {
                    string entry_name = message()->data()->payload.substr(1);
                    if (lookup_roster_entry(entry_name))
                    {
                        auto node_id = TractionDefs::train_node_id_from_legacy(
                            addressType_, address_);
                        verify_locomotive_node(node_id);
                        return invoke_subflow_and_wait(throttle_,
                            STATE(loco_assigned),
                            TractionThrottleCommands::ASSIGN_TRAIN, node_id,
                            listenForUpdates_);
                    }
                    else
                    {
                        sendBuf_ =
                            StringPrintf("HMESP32CS: Roster entry not found:%s%s",
                            entry_name.c_str(), REQUEST_EOL_CHARACTER_NL);
                        return logged_response(nextState);
                    }
                    break;
                }
            case 'F':           // Set function
            case 'f':           // Set function (forced)
            case 'm':           // Set momentary
                {
                    uint16_t state = message()->data()->payload.at(1) - '0';
                    uint32_t fn =
                        std::stoi(message()->data()->payload.substr(2));
                    throttle_->set_fn(fn, state);
                    sendBuf_.append(StringPrintf("M%cA%c%d%sF%d%02d%s",
                        throttleKey_, addressTypeChar_, address_,
                        COMMAND_PAYLOAD_DELIMITER, state, fn,
                        REQUEST_EOL_CHARACTER_NL));
                    return logged_response(nextState);
                }
                break;
            case 'I':           // Idle
                {
                    bool direction = throttle_->get_speed().direction();
                    SpeedType new_speed(0);
                    new_speed.set_direction(direction);
                    throttle_->set_speed(new_speed);
                }
                break;
            case 'L':           // Set address
            case 'S':           // Set address
                {
                    address_ = std::stoi(message()->data()->payload.substr(1));
                    if (message()->data()->payload.at(0) == 'S')
                    {
                        addressType_ = TrainAddressType::DCC_SHORT_ADDRESS;
                    }
                    else
                    {
                        addressType_ = TrainAddressType::DCC_LONG_ADDRESS;
                    }
                    auto node_id = TractionDefs::train_node_id_from_legacy(
                        addressType_, address_);
                    verify_locomotive_node(node_id);
                    return invoke_subflow_and_wait(throttle_,
                        STATE(loco_assigned),
                        TractionThrottleCommands::ASSIGN_TRAIN, node_id,
                        listenForUpdates_);
                }
                break;
            case 'q':           // Query state
                {
                    auto speed = throttle_->get_speed();
                    if (message()->data()->payload.at(1) == 'R')
                    {
                        sendBuf_ =
                            StringPrintf("M%cA%c%d%sV%d%s", throttleKey_,
                                addressTypeChar_, address_,
                                COMMAND_PAYLOAD_DELIMITER,
                                (speed.direction() != SpeedType::REVERSE),
                                REQUEST_EOL_CHARACTER_NL);
                    }
                    else if (message()->data()->payload.at(1) == 'V')
                    {
                        int speed_step = 0;
                        if (driveMode_ == DccMode::DCC_14 ||
                            driveMode_ == DccMode::DCC_14_LONG_ADDRESS)
                        {
                            speed_step = speed.get_dcc_14() & 0x1F;
                        }
                        else if (driveMode_ == DccMode::DCC_28 ||
                                 driveMode_ == DccMode::DCC_28_LONG_ADDRESS)
                        {
                            speed_step = speed.get_dcc_28() & 0x1F;
                        }
                        else if (driveMode_ == DccMode::DCC_128 ||
                                 driveMode_ == DccMode::DCC_128_LONG_ADDRESS ||
                                 driveMode_ == DccMode::DCC_DEFAULT ||
                                 driveMode_ == DccMode::DCC_DEFAULT_LONG_ADDRESS)
                        {
                            speed_step = speed.get_dcc_128() & 0x7F;
                        }
                        sendBuf_ =
                            StringPrintf("M%cA%c%d%sV%d%s", throttleKey_,
                                addressTypeChar_, address_,
                                COMMAND_PAYLOAD_DELIMITER, speed_step,
                                REQUEST_EOL_CHARACTER_NL);
                    }
                    else
                    {
                        return yield_and_call(STATE(command_not_recognized));
                    }
                    return logged_response(nextState);
                }
            case 'R':           // Set direction
                {
                    auto speed = throttle_->get_speed();
                    speed.set_direction(message()->data()->payload.at(1) == '0');
                    sendBuf_ =
                        StringPrintf("M%cA%c%d%sR%d%s", throttleKey_,
                            addressTypeChar_, address_,
                            COMMAND_PAYLOAD_DELIMITER,
                            speed.direction() != SpeedType::REVERSE,
                            REQUEST_EOL_CHARACTER_NL);
                    return logged_response(nextState);
                }
            case 's':           // Set speed step mode
                {
                    return yield_and_call(STATE(command_not_supported));
                }
            case 'V':           // Set speed
                {
                    bool direction = throttle_->get_speed().direction();
                    uint8_t speed_step =
                        std::stoi(message()->data()->payload.substr(1));
                    SpeedType new_speed;

                    if (driveMode_ == DccMode::DCC_14 ||
                        driveMode_ == DccMode::DCC_14_LONG_ADDRESS)
                    {
                        new_speed.set_dcc_14(speed_step);
                    }
                    else if (driveMode_ == DccMode::DCC_28 ||
                            driveMode_ == DccMode::DCC_28_LONG_ADDRESS)
                    {
                        new_speed.set_dcc_28(speed_step);
                    }
                    else if (driveMode_ == DccMode::DCC_128 ||
                             driveMode_ == DccMode::DCC_128_LONG_ADDRESS ||
                             driveMode_ == DccMode::DCC_DEFAULT ||
                             driveMode_ == DccMode::DCC_DEFAULT_LONG_ADDRESS)
                    {
                        new_speed.set_dcc_128(speed_step);
                    }
                    new_speed.set_direction(direction);
                    throttle_->set_speed(new_speed);
                    sendBuf_ =
                        StringPrintf("M%cA%c%d%sV%d%s", throttleKey_,
                            addressTypeChar_, address_,
                            COMMAND_PAYLOAD_DELIMITER, speed_step,
                            REQUEST_EOL_CHARACTER_NL);
                    return logged_response(nextState);
                }
            case 'X':           // Set eStop
                throttle_->set_emergencystop();
                break;
            default:            // Catch all for unrecognized
                return yield_and_call(STATE(command_not_recognized));
        }
        return yield_and_call(nextState);
    }

    StateFlowBase::Action WiThrottleClientFlow::WiThrottleCommandLocomotive::loco_assigned()
    {
        auto *m = full_allocation_result(throttle_);
        bool error = (m->data()->resultCode != Defs::ERROR_CODE_OK);
        m->unref();
        if (error)
        {
            LOG_ERROR("[WiThrottleClient: %d] Locomotive:%d assigment failed: %04x",
                throttleFlow_->fd_, address_, m->data()->resultCode);
            sendBuf_ =
                StringPrintf("HMESP32CS: Failed to assign locomotive:%d%s",
                    address_, REQUEST_EOL_CHARACTER_NL);
            return logged_response(STATE(done));
        }
        LOG(INFO, "[WiThrottleClient: %d] Assigned locomotive:%d",
            throttleFlow_->fd_, address_);
        return invoke_subflow_and_wait(throttle_, STATE(loco_state_loaded),
            TractionThrottleCommands::LOAD_STATE);
    }

    StateFlowBase::Action WiThrottleClientFlow::WiThrottleCommandLocomotive::loco_released()
    {
        auto *m = full_allocation_result(throttle_);
        bool error = (m->data()->resultCode != Defs::ERROR_CODE_OK);
        m->unref();
        if (error)
        {
            sendBuf_ =
                StringPrintf("HMESP32CS: Failed to release locomotive:%d%s",
                    address_, REQUEST_EOL_CHARACTER_NL);
            return logged_response(STATE(done));
        }
        LOG(INFO, "[WiThrottleClient: %d] Released locomotive:%d",
            throttleFlow_->fd_, address_);
        sendBuf_ =
            StringPrintf("M%c-%c%d%sr", throttleKey_, addressTypeChar_,
                address_, REQUEST_EOL_CHARACTER_NL);
        throttleFlow_->release_throttle(throttle_);
        Callback nextState = STATE(done);
        if (addressType_ == TrainAddressType::UNSPECIFIED)
        {
            nextState = STATE(release_next_loco);
        }
        return logged_response(nextState);
    }

    StateFlowBase::Action WiThrottleClientFlow::WiThrottleCommandLocomotive::release_next_loco()
    {
        // reset the address to zero so get_throttle looks only by throttle key
        address_ = 0;
        throttle_ = throttleFlow_->get_throttle(throttleKey_, &address_);
        // if a throttle was found, release the assigned locomotive.
        if (throttle_)
        {
            return invoke_subflow_and_wait(throttle_, STATE(loco_released),
                TractionThrottleCommands::RELEASE_TRAIN);
        }
        return yield_and_call(STATE(done));
    }

    StateFlowBase::Action WiThrottleClientFlow::WiThrottleCommandLocomotive::loco_state_loaded()
    {
        auto *m = full_allocation_result(throttle_);
        bool error = (m->data()->resultCode != Defs::ERROR_CODE_OK);
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
            sendBuf_ = StringPrintf("M%c+%c%d%s%s", throttleKey_,
                addressTypeChar_, address_, COMMAND_PAYLOAD_DELIMITER,
                REQUEST_EOL_CHARACTER_NL);
        }
        return logged_response(nextState);
    }

    StateFlowBase::Action WiThrottleClientFlow::WiThrottleCommandLocomotive::send_function_labels()
    {
        auto train_db = Singleton<esp32cs::Esp32TrainDatabase>::instance();
        auto train_index = train_db->get_index(address_);
        if (train_index >= 0)
        {
            sendBuf_ =
                StringPrintf("M%cA%c%d%s", throttleKey_, addressTypeChar_,
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
            return logged_response(STATE(send_function_states));
        }
        return yield_and_call(STATE(send_function_states));
    }

    StateFlowBase::Action WiThrottleClientFlow::WiThrottleCommandLocomotive::send_function_states()
    {
        sendBuf_ =
            StringPrintf("M%cA%c%d%s", throttleKey_, addressTypeChar_,
                address_, COMMAND_PAYLOAD_DELIMITER);
        for (int fn = 0; fn < commandstation::DCC_MAX_FN; fn++)
        {
            uint16_t state = throttle_->get_fn(fn);
            if (state == TractionThrottle::FN_NOT_KNOWN)
            {
                state = 0;
            }
            sendBuf_.append(StringPrintf("M%cA%c%d%sF%d%02d%s", throttleKey_,
                addressTypeChar_, address_, COMMAND_PAYLOAD_DELIMITER, state,
                fn, REQUEST_EOL_CHARACTER_NL));
        }
        return logged_response(STATE(send_loco_speed));
    }

    StateFlowBase::Action WiThrottleClientFlow::WiThrottleCommandLocomotive::send_loco_speed()
    {
        auto speed = throttle_->get_speed();
        int speed_step = 0;
        if (driveMode_ == DccMode::DCC_14 ||
            driveMode_ == DccMode::DCC_14_LONG_ADDRESS)
        {
            speed_step = speed.get_dcc_14() & 0x1F;
        }
        else if (driveMode_ == DccMode::DCC_28 ||
                 driveMode_ == DccMode::DCC_28_LONG_ADDRESS)
        {
            speed_step = speed.get_dcc_28() & 0x1F;
        }
        else if (driveMode_ == DccMode::DCC_128 ||
                 driveMode_ == DccMode::DCC_128_LONG_ADDRESS ||
                 driveMode_ == DccMode::DCC_DEFAULT ||
                 driveMode_ == DccMode::DCC_DEFAULT_LONG_ADDRESS)
        {
            speed_step = speed.get_dcc_128() & 0x7F;
        }
        sendBuf_ =
            StringPrintf("M%cA%c%d%sV%d%s", throttleKey_, addressTypeChar_,
                address_, COMMAND_PAYLOAD_DELIMITER, speed_step,
                REQUEST_EOL_CHARACTER_NL);
        return logged_response(STATE(send_loco_direction));
    }

    StateFlowBase::Action WiThrottleClientFlow::WiThrottleCommandLocomotive::send_loco_direction()
    {
        auto speed = throttle_->get_speed();
        sendBuf_ =
            StringPrintf("M%cA%c%d%sV%d%s", throttleKey_, addressTypeChar_,
            address_, COMMAND_PAYLOAD_DELIMITER,
            (speed.direction() != SpeedType::REVERSE),
            REQUEST_EOL_CHARACTER_NL);
        return logged_response(STATE(send_loco_stepmode));
    }

    StateFlowBase::Action WiThrottleClientFlow::WiThrottleCommandLocomotive::send_loco_stepmode()
    {
        int speed_step = 1;
        if (driveMode_ == DccMode::DCC_14 ||
            driveMode_ == DccMode::DCC_14_LONG_ADDRESS)
        {
            speed_step = 8;
        }
        else if (driveMode_ == DccMode::DCC_28 ||
                 driveMode_ == DccMode::DCC_28_LONG_ADDRESS)
        {
            speed_step = 2;
        }
        else if (driveMode_ == DccMode::DCC_128 ||
                 driveMode_ == DccMode::DCC_128_LONG_ADDRESS ||
                 driveMode_ == DccMode::DCC_DEFAULT ||
                 driveMode_ == DccMode::DCC_DEFAULT_LONG_ADDRESS)
        {
            speed_step = 1;
        }
        else
        {
            LOG_ERROR( "[WiThrottleClient: %d] unhandled drive mode: %d",
                throttleFlow_->fd_, static_cast<int>(driveMode_));
        }
        sendBuf_ =
            StringPrintf("M%cA%c%d%sS%d%s", throttleKey_, addressTypeChar_,
            address_, COMMAND_PAYLOAD_DELIMITER, speed_step,
            REQUEST_EOL_CHARACTER_NL);
        return logged_response(STATE(done));
    }

    StateFlowBase::Action WiThrottleClientFlow::WiThrottleCommandLocomotive::done()
    {
        return release_and_exit();
    }

    StateFlowBase::Action WiThrottleClientFlow::WiThrottleCommandLocomotive::command_not_recognized()
    {
        sendBuf_ = StringPrintf("HMESP32CS: Command not understood.%s",
            REQUEST_EOL_CHARACTER_NL);
        return logged_response(STATE(done));
    }

    StateFlowBase::Action WiThrottleClientFlow::WiThrottleCommandLocomotive::command_not_supported()
    {
        sendBuf_ = StringPrintf("HMESP32CS: Command not supported.%s",
            REQUEST_EOL_CHARACTER_NL);
        return logged_response(STATE(done));
    }
} // namespace withrottle