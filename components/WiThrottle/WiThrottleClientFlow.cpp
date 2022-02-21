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
 * \file WiThrottleClientFlow.cpp
 *
 * WiThrottle Client request implementation.
 *
 * @author Mike Dunston
 * @date 8 Feb 2022
 */

/// @file WiThrottle client request implementation.

#include "WiThrottle.hxx"

#include <AccessoryDecoderDatabase.hxx>
#include <dcc/DccOutput.hxx>
#include <HttpStringUtils.h>
#include <StringUtils.hxx>
#include <TrainDatabase.h>

namespace withrottle
{
    WiThrottleClientFlow::WiThrottleClientFlow(
        WiThrottleServer *server, int fd, uint32_t remote_ip, uint8_t heartbeat)
        : StateFlowBase(server), remoteIP_(remote_ip), fd_(fd),
          server_(server), commandDispatcher_(server), 
          nextCommand_(commandDispatcher_.alloc()), heartbeat_(heartbeat),
          locoCmd_(this), rosterCmd_(this), panelCmd_(this),
          packetCmd_(this, server->track_)
    {
        server_->connectionCount_++;
        buf_.resize(readSize_);
        reqData_.reserve(readSize_ + 1);
        start_flow(STATE(read_more_data));
        LOG(CONFIG_WITHROTTLE_LOGGING, "[WiThrottleClient fd:%d] Connected.",
            fd_);
    }

    WiThrottleClientFlow::~WiThrottleClientFlow()
    {
        server_->connectionCount_--;
        LOG(CONFIG_WITHROTTLE_LOGGING, "[WiThrottleClient fd:%d] Closed", fd_);
        ::close(fd_);

        // cleanup any created throttles
        for (auto throttle : throttles_)
        {
            delete throttle.throttle;
        }

        nextCommand_->unref();
    }

    StateFlowBase::Action WiThrottleClientFlow::parse()
    {
        Callback nextState = STATE(read_more_data);
        if (helper_.hasError_)
        {
            return call_immediately(STATE(shutdown));
        }
        else if (helper_.remaining_ == readSize_ || buf_.empty())
        {
            return yield_and_call(STATE(read_more_data));
        }

        reqData_.append((char *)buf_.data(), readSize_ - helper_.remaining_);

        // replace occurrences of various end of line characters with a single
        // variant that we can use internally.
        esp32cs::string_replace_all(reqData_, REQUEST_EOL_CHARACTER_CRNL,
            REQUEST_EOL_CHARACTER_NL);
        esp32cs::string_replace_all(reqData_, REQUEST_EOL_CHARACTER_CR,
            REQUEST_EOL_CHARACTER_NL);

        // if we don't have an EOL string in the data yet, get more data
        if (reqData_.find(REQUEST_EOL_CHARACTER_NL) == string::npos)
        {
            return yield_and_call(STATE(read_more_data));
        }

        // parse the data we have into delimited lines of requests, this will
        // leave some data in the reqData_ which will need to be re-parsed on
        // next run.
        vector<string> commands;
        size_t parsed = http::tokenize(reqData_, commands,
            REQUEST_EOL_CHARACTER_NL, false);

        // drop whatever has been tokenized so we don't process it again
        LOG(CONFIG_WITHROTTLE_LOGGING,
            "[WiThrottleClient fd:%d] parsed: %zu / %zu, commands: %zu", fd_,
            parsed, reqData_.length(), commands.size());
        reqData_.erase(0, parsed);

        size_t count = 0;
        for (auto &line : commands)
        {
            count++;
            LOG(CONFIG_WITHROTTLE_LOGGING,
                "[WiThrottleClient fd:%d] (%zu/%zu): ||%s||", fd_, count,
                commands.size(), line.c_str());
            
            bool dispatch = false;
            nextCommand_->data()->command = static_cast<WiThrottleCommands>(line[0]);
            switch(line[0])
            {
                case WiThrottleCommands::PRIMARY:       // T
                case WiThrottleCommands::SECONDARY:     // S
                case WiThrottleCommands::MULTI:         // M
                {
                    if (line.length() < 2)
                    {
                        LOG_ERROR("[WiThrottleClient fd:%d] Throttle command "
                                  "is not long enough to be valid, rejecting",
                            fd_);
                    }
                    else
                    {
                        LOG(CONFIG_WITHROTTLE_LOGGING,
                            "[WiThrottleClient fd:%d] throttle(%c): %s", fd_,
                            line[1], line.substr(1).c_str());
                        switch(line[2])
                        {
                            case 'A':
                            case '+':
                            case '-':
                                dispatch = true;
                                break;
                            default:
                                LOG_ERROR(
                                    "[WiThrottleClient fd:%d] Unrecognized "
                                    "Throttle command code: %c", fd_, line[2]);
                                break;
                        }
                    }
                    break;
                }
                case WiThrottleCommands::QUIT:          // Q
                {
                    // since the client requested to shutdown the connection,
                    // ignore any remaining commands and proceed directly to
                    // cleanup/disconnect.
                    return yield_and_call(STATE(shutdown));
                }
                case WiThrottleCommands::SET_ID:        // HU{id}
                {
                    if (line.length() < 3 || line[1] != 'U')
                    {
                        LOG_ERROR("[WiThrottleClient fd:%d] UDID does not "
                                  "appear to be valid, rejecting",
                            fd_);
                        // invalid UDID detected, shutdown the connection
                        nextState = STATE(shutdown);
                    }
                    else
                    {
                        udid_.assign(line.substr(2));
                        LOG(CONFIG_WITHROTTLE_LOGGING,
                            "[WiThrottleClient fd:%d] UDID: %s", fd_,
                            udid_.c_str());
                    }
                    break;
                }
                case WiThrottleCommands::SET_NAME:      // N{name}
                {
                    if (line.length() < 2)
                    {
                        LOG_ERROR("[WiThrottleClient fd:%d] Throttle name was "
                                  "not provided!", fd_);
                        // invalid name detected, shutdown the connection
                        nextState = STATE(shutdown);
                    }
                    else
                    {
                        name_.assign(line.substr(1));
                        LOG(CONFIG_WITHROTTLE_LOGGING,
                            "[WiThrottleClient fd:%d] Name: %s", fd_,
                            name_.c_str());
                        
                        // received a valid name, proceed to sending the CS
                        // information to the client, this will include all
                        // locomotives, consists, accessories, routes, etc.
                        nextState = STATE(send_cs_info_packets);
                    }
                }
                case WiThrottleCommands::ROSTER:        // R
                {
                    if (line.length() < 2 || line[1] != 'C')
                    {
                        LOG_ERROR("[WiThrottleClient fd:%d] Roster command "
                                  "does not appear to be valid, rejecting",
                            fd_);
                    }
                    else
                    {
                        dispatch = true;
                    }
                    break;
                }
                case WiThrottleCommands::PANEL:         // P
                {
                    if (line.length() < 2 || (
                        line[1] != 'P' &&               // track power
                        line[1] != 'T' &&               // turnouts
                        line[1] != 'R' ))               // routes
                    {
                        LOG_ERROR("[WiThrottleClient fd:%d] Panel command "
                                  "does not appear to be valid, rejecting",
                            fd_);
                    }
                    else
                    {
                        dispatch = true;
                    }
                    break;
                }
                case WiThrottleCommands::HEX_PACKET:    // D
                {
                    dispatch = true;
                    break;
                }
                case WiThrottleCommands::HEARTBEAT:     // *
                {
                    if (line.length() > 1)
                    {
                        if (line[1] == '+')             // Enable
                        {
                            LOG(CONFIG_WITHROTTLE_LOGGING,
                                "Enabling Heartbeat Timer");
                        }
                        else                            // Disable
                        {
                            LOG(CONFIG_WITHROTTLE_LOGGING,
                                "Disabling Heartbeat Timer");
                        }
                    }
                    else
                    {
                        // reset heartbeat timer
                    }
                    break;
                }
            }
            if (dispatch)
            {
                nextCommand_->data()->payload.assign(line.substr(1));
                commandDispatcher_.send(nextCommand_);
                nextCommand_ = commandDispatcher_.alloc();
            }
            else
            {
                LOG(WARNING,
                    "[WiThrottleClient fd:%d] requested command was discarded",
                    fd_);
            }
        }

        return yield_and_call(nextState);
    }

    StateFlowBase::Action WiThrottleClientFlow::shutdown()
    {
        LOG(CONFIG_WITHROTTLE_LOGGING,
            "[WiThrottleClient fd:%d] Shutting down connection", fd_);

        if (throttles_.empty())
        {
            return delete_this();
        }

        throttleIter_ = throttles_.begin();

        return call_immediately(STATE(shutdown_throttles));
    }

    StateFlowBase::Action WiThrottleClientFlow::shutdown_throttles()
    {
        if (throttleIter_ == throttles_.end())
        {
            return delete_this();
        }
        auto throttle = throttleIter_->throttle;
        auto throttle_id = throttleIter_->key;
        throttleIter_++;
        if (throttle->is_train_assigned())
        {
            LOG(CONFIG_WITHROTTLE_LOGGING,
                "[WiThrottleClient fd:%d] Releasing train from throttle '%c'",
                fd_, throttle_id);
            invoke_subflow_and_wait(throttle, STATE(shutdown_throttles),
                openlcb::TractionThrottleCommands::RELEASE_TRAIN);
        }
        else
        {
            LOG(CONFIG_WITHROTTLE_LOGGING,
                "[WiThrottleClient fd:%d] Throttle '%c' appears to be idle",
                fd_, throttle_id);
        }
        return call_immediately(STATE(shutdown_throttles));
    }

    StateFlowBase::Action WiThrottleClientFlow::read_more_data()
    {
        LOG(CONFIG_WITHROTTLE_LOGGING,
            "[WiThrottleClient fd:%d] Requesting more data", fd_);
        return read_repeated_with_timeout(&helper_, timeout_, fd_, buf_.data(),
                                          readSize_, STATE(parse));
    }

    StateFlowBase::Action WiThrottleClientFlow::send_cs_info_packets()
    {
        string data = StringPrintf(
R"raw(VN2.0
HTESP32CS
HtESP32 Command Station (%s) %s
)raw", CONFIG_IDF_TARGET, openlcb::SNIP_STATIC_DATA.software_version);
        return write_repeated(&helper_, fd_, data.data(), data.length(),
            STATE(send_roster_list));
    }

    StateFlowBase::Action WiThrottleClientFlow::send_roster_list()
    {
        auto train_db = Singleton<esp32cs::Esp32TrainDatabase>::instance();
        string data = train_db->to_withrottle();
        return write_repeated(&helper_, fd_, data.data(), data.length(),
            STATE(send_consist_list));
    }

    StateFlowBase::Action WiThrottleClientFlow::send_consist_list()
    {
        // RCL = list only, RCC = control, RCD = roster consist data
        string data = StringPrintf("RCL%d%s", 0, REQUEST_EOL_CHARACTER_NL);
        return write_repeated(&helper_, fd_, data.data(), data.length(),
            STATE(send_accessory_list));
    }

    StateFlowBase::Action WiThrottleClientFlow::send_accessory_list()
    {
        auto accessory_db = Singleton<esp32cs::AccessoryDecoderDB>::instance();
        string data = accessory_db->to_withrottle();
        return write_repeated(&helper_, fd_, data.data(), data.length(),
            STATE(send_power_status));
    }

    StateFlowBase::Action WiThrottleClientFlow::send_power_status()
    {
        auto track = get_dcc_output(DccOutput::Type::TRACK);
        string data = StringPrintf("PPA%d%s",
            track->get_disable_output_reasons() == 0,
            REQUEST_EOL_CHARACTER_NL);
        if (heartbeat_)
        {
            return write_repeated(&helper_, fd_, data.data(), data.length(),
                STATE(send_heartbeat_config));
        }
        return write_repeated(&helper_, fd_, data.data(), data.length(),
            STATE(read_more_data));
    }

    StateFlowBase::Action WiThrottleClientFlow::send_heartbeat_config()
    {
        string data =
            StringPrintf("*%d%s", heartbeat_, REQUEST_EOL_CHARACTER_NL);
        return write_repeated(&helper_, fd_, data.data(), data.length(),
            STATE(read_more_data));
    }

    void WiThrottleClientFlow::trigger_shutdown()
    {
        reset_flow(STATE(shutdown));
    }

    openlcb::TractionThrottle *WiThrottleClientFlow::get_throttle(
        uint8_t key, uint32_t *address)
    {
        // scan registered throttles to find one matching the key and address
        auto entry = std::find_if(throttles_.begin(), throttles_.end(),
        [key, address](auto &ent)
        {
            if (*address == 0)
            {
                return ent.key == key;
            }
            return (ent.key == key && ent.address == *address);
        });

        // If an entry was found, return it to the caller.
        if (entry != throttles_.end())
        {
            *address = entry->address;
            return entry->throttle;
        }

        // If the address is zero we are looking for all throttles that match
        // a given key for a remove or action and do not need to allocate a new
        // throttle.
        if (*address == 0)
        {
            return nullptr;
        }

        // no entry was found, check if the client has reached the maximum
        // number of controlled locomotives
        if (throttles_.size() >= config_withrottle_max_client_locomotives())
        {
            LOG_ERROR("[WiThrottleClient fd:%d] Maximum throttles reached!",
                fd_);
            return nullptr;
        }
        throttle_t new_throttle =
        {
            .throttle = new openlcb::TractionThrottle(server_->node_),
            .key = key,
            .address = *address
        };
        throttles_.push_back(new_throttle);

        return new_throttle.throttle;
    }

    void WiThrottleClientFlow::release_throttle(openlcb::TractionThrottle *throttle)
    {
        auto entry = std::find_if(throttles_.begin(), throttles_.end(),
        [throttle](auto &ent)
        {
            return ent.throttle == throttle;
        });
        if (entry != throttles_.end())
        {
            delete throttle;
            throttles_.erase(entry);
        }
    }

} // namespace withrottle