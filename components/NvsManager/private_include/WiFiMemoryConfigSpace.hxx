/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2017-2021 Mike Dunston

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see http://www.gnu.org/licenses
**********************************************************************/

#ifndef WIFICFG_MEMORY_CONFIG_SPACE_HXX_
#define WIFICFG_MEMORY_CONFIG_SPACE_HXX_

#include <openlcb/SimpleStack.hxx>
#include <openlcb/VirtualMemorySpace.hxx>
#include <utils/Base64.hxx>
#include <utils/format_utils.hxx>
#include <utils/logging.h>
#include <WiFiConfigurationGroup.hxx>

#include "NvsManagerStruct.hxx"
#include "sdkconfig.h"
#include "AbstractVirtualMemorySpace.hxx"

namespace esp32cs
{
    /// Node configuration holder
    static constexpr WiFiConfiguration WiFiConfigHolder(CONFIG_OLCB_WIFI_MEMORY_SPACE_OFFSET);

    /// Virtual memory space that allows reconfiguration of the persistent node
    /// identifier.
    class WiFiMemoryConfigSpace : public openlcb::VirtualMemorySpace,
                                  public AbstractVirtualMemorySpace
    {
    public:
        /// Constructor.
        ///
        /// @param stack is the @ref SimpleStackBase that this memory space
        /// should be registered with.
        /// @param node_id is the current node identifier.
        WiFiMemoryConfigSpace(openlcb::SimpleStackBase *stack,
            node_config_t *config) : config_(config)
        {
            #define REGISTER_NUMBER(field, type, name)                         \
                LOG(VERBOSE, "[WiFiMemCfg:%02x] register_numeric(%s): %d",     \
                    SPACE, name, field.offset());                              \
                register_numeric(field, numeric_reader<type>(field.offset()),  \
                                numeric_writer<type>(field.offset()))
            #define REGISTER_STRING(field, name)                               \
                LOG(VERBOSE, "[WiFiMemCfg:%02x] register_string(%s): %d",      \
                    SPACE, name, field.offset());                              \
                register_string(field, string_reader(field.offset()),          \
                                string_writer(field.offset()))

            REGISTER_NUMBER(WiFiConfigHolder.wifi_mode(), uint8_t,
                            "wifi_mode");
            REGISTER_STRING(WiFiConfigHolder.hostname_prefix(),
                            "hostname_prefix");
            // Station Config
            REGISTER_STRING(WiFiConfigHolder.station().ssid(), "station_ssid");
            REGISTER_STRING(WiFiConfigHolder.station().password(),
                            "station_pass");
            // SoftAP Config
            REGISTER_STRING(WiFiConfigHolder.softap().ssid(), "softap_ssid");
            REGISTER_STRING(WiFiConfigHolder.softap().password(),
                            "softap_pass");
            REGISTER_NUMBER(WiFiConfigHolder.softap().auth(), uint8_t,
                            "softap_auth");
            REGISTER_NUMBER(WiFiConfigHolder.softap().channel(), uint8_t,
                            "softap_channel");
            // SNTP Congig
            REGISTER_NUMBER(WiFiConfigHolder.sntp().enabled(), uint8_t,
                            "sntp_enabled");
            REGISTER_STRING(WiFiConfigHolder.sntp().server(), "sntp_server");
            REGISTER_STRING(WiFiConfigHolder.sntp().timezone(),
                            "sntp_timezone");
            stack->memory_config_handler()->registry()->insert(
                stack->node(), SPACE, this);
            #undef REGISTER_NUMBER
            #undef REGISTER_STRING
            LOG(INFO, "[WiFiMemCfg:%02x] registered", SPACE);
        }

    private:
        node_config_t *config_;
        static constexpr uint8_t SPACE = CONFIG_OLCB_WIFI_MEMORY_SPACE_ID;
        template <typename T>
        typename std::function<T(unsigned repeat, BarrierNotifiable *done)>
        numeric_reader(int offset)
        {
            return [this, offset](unsigned repeat, BarrierNotifiable *done)
            {
                LOG(VERBOSE, "[WiFiMemCfg:%02x-RD] offs: %d", SPACE, offset);
                T value = (T)0;
                switch(offset)
                {
                    case WiFiConfigHolder.wifi_mode().offset():
                        LOG(VERBOSE, "[WiFiMemCfg:%02x-RD] wifi_mode: %d",
                            SPACE, config_->wifi_mode);
                        value = config_->wifi_mode;
                        break;
                    case WiFiConfigHolder.softap().auth().offset():
                        LOG(VERBOSE, "[WiFiMemCfg:%02x-RD] softap_auth: %d",
                            SPACE, config_->softap_auth);
                        value = config_->softap_auth;
                        break;
                    case WiFiConfigHolder.softap().channel().offset():
                        LOG(VERBOSE, "[WiFiMemCfg:%02x-RD] softap_channel: %d",
                            SPACE, config_->softap_channel);
                        value = config_->softap_channel;
                        break;
                    case WiFiConfigHolder.sntp().enabled().offset():
                        LOG(VERBOSE, "[WiFiMemCfg:%02x-RD] sntp_enabled: %d",
                            SPACE, config_->sntp_enabled);
                        value = config_->sntp_enabled;
                        break;
                    default:
                        LOG_ERROR("[WiFiMemCfg:%02x-RD] request for "
                                  "unrecognized offset:%d", SPACE, offset);
                }
                done->notify();
                return value;
            };
        }
        template <typename T>
        std::function<void(unsigned repeat, T value, BarrierNotifiable *done)>
        numeric_writer(int offset)
        {
            return [this, offset](unsigned repeat, T value, BarrierNotifiable *done)
            {
                LOG(VERBOSE, "[WiFiMemCfg:%02x-WR] offs: %d, value: %d",
                    SPACE, offset, (uint32_t)value);
                switch(offset)
                {
                    case WiFiConfigHolder.wifi_mode().offset():
                        LOG(VERBOSE, "[WiFiMemCfg:%02x-RD] wifi_mode: %d",
                            SPACE, value);
                        config_->wifi_mode = (wifi_mode_t)value;
                        set_modified(true);
                        break;
                    case WiFiConfigHolder.softap().auth().offset():
                        LOG(VERBOSE, "[WiFiMemCfg:%02x-RD] softap_auth: %d",
                            SPACE, value);
                        config_->softap_auth = (wifi_auth_mode_t)value;
                        set_modified(true);
                        break;
                    case WiFiConfigHolder.softap().channel().offset():
                        LOG(VERBOSE, "[WiFiMemCfg:%02x-RD] softap_channel: %d",
                            SPACE, value);
                        config_->softap_channel = value;
                        set_modified(true);
                        break;
                    case WiFiConfigHolder.sntp().enabled().offset():
                        LOG(VERBOSE, "[WiFiMemCfg:%02x-RD] sntp_enabled: %d",
                            SPACE, value);
                        config_->sntp_enabled = value;
                        set_modified(true);
                        break;
                    default:
                        LOG_ERROR("[WiFiMemCfg:%02x-WR] request for "
                                  "unrecognized offset:%d", SPACE, offset);
                }
                done->notify();
            };
        }
        std::function<void(unsigned repeat, string *value, BarrierNotifiable *done)>
        string_reader(int offset)
        {
            return [this, offset](unsigned repeat, string *value, BarrierNotifiable *done)
            {
                LOG(VERBOSE, "[WiFiMemCfg:%02x-RDSTR] offs: %d", SPACE, offset);
                switch (offset)
                {
                    case WiFiConfigHolder.hostname_prefix().offset():
                        *value = config_->hostname_prefix;
                        LOG(VERBOSE,
                            "[WiFiMemCfg:%02x-RDSTR] hostname_prefix: %s",
                            SPACE, value->c_str());
                        break;
                    case WiFiConfigHolder.station().ssid().offset():
                        *value = config_->station_ssid;
                        LOG(VERBOSE,
                            "[WiFiMemCfg:%02x-RDSTR] station_ssid: %s", SPACE,
                            value->c_str());
                        break;
                    case WiFiConfigHolder.station().password().offset():
                        {
                            string encoded =
                                base64_encode(config_->station_pass);
                            encoded.insert(0, "***");
                            *value = encoded;
                        }
                        LOG(VERBOSE,
                            "[WiFiMemCfg:%02x-RDSTR] station_pass: %s", SPACE,
                            value->c_str());
                        break;
                    case WiFiConfigHolder.softap().ssid().offset():
                        *value = config_->softap_ssid;
                        LOG(VERBOSE, "[WiFiMemCfg:%02x-RDSTR] softap_ssid: %s",
                            SPACE, value->c_str());
                        break;
                    case WiFiConfigHolder.softap().password().offset():
                        {
                            string encoded =
                                base64_encode(config_->softap_pass);
                            encoded.insert(0, "***");
                            *value = encoded;
                        }
                        LOG(VERBOSE, "[WiFiMemCfg:%02x-RDSTR] softap_pass: %s",
                            SPACE, value->c_str());
                        break;
                    case WiFiConfigHolder.sntp().server().offset():
                        *value = config_->sntp_server;
                        LOG(VERBOSE, "[WiFiMemCfg:%02x-RDSTR] sntp_server: %s",
                            SPACE, value->c_str());
                        break;
                    case WiFiConfigHolder.sntp().timezone().offset():
                        *value = config_->timezone;
                        LOG(VERBOSE, "[WiFiMemCfg:%02x-RDSTR] timezone: %s",
                            SPACE, value->c_str());
                        break;
                    default:
                        LOG_ERROR("[WiFiMemCfg:%02x--RSTR] request for "
                                  "unrecognized offset:%d", SPACE, offset);
                        *value = "";
                }
                done->notify();
            };
        }
        std::function<void(unsigned repeat, string value, BarrierNotifiable *done)>
        string_writer(int offset)
        {
            return [this, offset](unsigned repeat, string value, BarrierNotifiable *done)
            {
                // strip off nulls (if found)
                value.erase(
                    std::remove(value.begin(), value.end(), '\0'), value.end());
                LOG(VERBOSE, "[WiFiMemCfg:%02x-WRSTR] offs: %d, value:%s",
                    SPACE, offset, value.c_str());
                switch(offset)
                {
                    case WiFiConfigHolder.hostname_prefix().offset():
                        LOG(VERBOSE,
                            "[WiFiMemCfg:%02x-RD] hostname_prefix: %s", SPACE,
                            value.c_str());
                        str_populate(config_->hostname_prefix, value.c_str());
                        set_modified(true);
                        break;
                    case WiFiConfigHolder.station().ssid().offset():
                        LOG(VERBOSE, "[WiFiMemCfg:%02x-RD] station_ssid: %s",
                            SPACE, value.c_str());
                        str_populate(config_->station_ssid, value.c_str());
                        set_modified(true);
                        break;
                    case WiFiConfigHolder.station().password().offset():
                        if (value.rfind("***", 0) == 0)
                        {
                            string encodedpw = value.substr(3);
                            string value = "";
                            base64_decode(encodedpw, &value);
                        }
                        LOG(VERBOSE, "[WiFiMemCfg:%02x-RD] station_pass: %s",
                            SPACE, value.c_str());
                        str_populate(config_->station_pass, value.c_str());
                        set_modified(true);
                        break;
                    case WiFiConfigHolder.softap().ssid().offset():
                        LOG(VERBOSE, "[WiFiMemCfg:%02x-RD] softap_ssid: %s",
                            SPACE, value.c_str());
                        str_populate(config_->softap_ssid, value.c_str());
                        set_modified(true);
                        break;
                    case WiFiConfigHolder.softap().password().offset():
                        if (value.rfind("***", 0) == 0)
                        {
                            string encodedpw = value.substr(3);
                            string value = "";
                            base64_decode(encodedpw, &value);
                        }
                        LOG(VERBOSE, "[WiFiMemCfg:%02x-RD] softap_pass: %s",
                            SPACE, value.c_str());
                        str_populate(config_->softap_pass, value.c_str());
                        set_modified(true);
                        break;
                    case WiFiConfigHolder.sntp().server().offset():
                        LOG(VERBOSE, "[WiFiMemCfg:%02x-RD] sntp_server: %s",
                            SPACE, value.c_str());
                        str_populate(config_->sntp_server, value.c_str());
                        set_modified(true);
                        break;
                    case WiFiConfigHolder.sntp().timezone().offset():
                        LOG(VERBOSE, "[WiFiMemCfg:%02x-RD] timezone: %s",
                            SPACE, value.c_str());
                        str_populate(config_->timezone, value.c_str());
                        set_modified(true);
                        break;
                    default:
                        LOG_ERROR("[WiFiMemCfg:%02x-WSTR] request for "
                                  "unrecognized offset:%d", SPACE, offset);
                }
                done->notify();
            };
        }
    };
} // namespace esp32cs

#endif // WIFICFG_MEMORY_CONFIG_SPACE_HXX_
