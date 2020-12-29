/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2020 Mike Dunston

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

#include "LCCWiFiManager.h"
#include "LCCStackManager.h"
#include "JsonConstants.h"

#include "sdkconfig.h"

#include <algorithm>
#include <FileSystemManager.h>
#include <esp_sntp.h>
#include <freertos_drivers/esp32/Esp32WiFiManager.hxx>
#include <HttpStringUtils.h>
#include <openlcb/SimpleStack.hxx>
#include <utime.h>

namespace esp32cs
{

static constexpr const char WIFI_STATION_CFG[] = "wifi-sta";
static constexpr const char WIFI_STATION_DNS_CFG[] = "wifi-dns";
static constexpr const char WIFI_STATION_IP_CFG[] = "wifi-ip";
static constexpr const char WIFI_SOFTAP_CFG[] = "wifi-ap";

#ifndef CONFIG_WIFI_STATIC_IP_ADDRESS
#define CONFIG_WIFI_STATIC_IP_ADDRESS ""
#endif

#ifndef CONFIG_WIFI_STATIC_IP_GATEWAY
#define CONFIG_WIFI_STATIC_IP_GATEWAY ""
#endif

#ifndef CONFIG_WIFI_STATIC_IP_SUBNET
#define CONFIG_WIFI_STATIC_IP_SUBNET ""
#endif

#ifndef CONFIG_WIFI_STATIC_IP_DNS
#define CONFIG_WIFI_STATIC_IP_DNS ""
#endif

#ifndef CONFIG_WIFI_SOFTAP_SSID
#define CONFIG_WIFI_SOFTAP_SSID "esp32cs"
#endif

#ifndef CONFIG_WIFI_SOFTAP_PASSWORD
#define CONFIG_WIFI_SOFTAP_PASSWORD "esp32cs"
#endif

#ifndef CONFIG_WIFI_STATION_SSID
#define CONFIG_WIFI_STATION_SSID "esp32cs"
#endif

#ifndef CONFIG_WIFI_STATION_PASSWORD
#define CONFIG_WIFI_STATION_PASSWORD "esp32cs"
#endif

#ifndef CONFIG_SNTP_SERVER
#define CONFIG_SNTP_SERVER "pool.ntp.org"
#endif

#ifndef CONFIG_TIMEZONE
#define CONFIG_TIMEZONE "UTC0"
#endif

#if CONFIG_SNTP

static bool sntp_callback_called_previously = false;
static void sntp_received(struct timeval *tv)
{
  // if this is the first time we have been called, check if the modification
  // timestamp on the LCC_CDI_XML is older than 2020-01-01 and if so reset the
  // modification/access timestamp to current.
  if (!sntp_callback_called_previously)
  {
    struct stat statbuf;
    stat(LCC_CDI_XML, &statbuf);
    time_t mod = statbuf.st_mtime;
    struct tm timeinfo;
    localtime_r(&mod, &timeinfo);
    // check if the timestamp on LCC_CDI_XML is prior to 2020 and if so, force
    // the access/modified timestamps to the current time.
    if (timeinfo.tm_year < (2020 - 1900))
    {
      LOG(INFO, "[SNTP] Updating timestamps on CDI XML files");
      utime(LCC_CDI_XML, NULL);
      utime("/cfg/LCC/train.xml", NULL);
      utime("/cfg/LCC/tmptrain.xml", NULL);
    }
    sntp_callback_called_previously = true;
  }
  time_t new_time = tv->tv_sec;
  LOG(INFO, "[SNTP] Received time update, new localtime: %s", ctime(&new_time));

#if CONFIG_FASTCLOCK_REALTIME
  Singleton<LCCWiFiManager>::instance()->real_time_clock_sync(new_time);
#endif // CONFIG_FASTCLOCK_REALTIME
}
#endif // CONFIG_SNTP

LCCWiFiManager::LCCWiFiManager(openlcb::SimpleStackBase *stack
                             , const esp32cs::Esp32ConfigDef &cfg)
                             : stack_(stack), cfg_(cfg)
{
  string ip = CONFIG_WIFI_STATIC_IP_ADDRESS;
  string gateway = CONFIG_WIFI_STATIC_IP_GATEWAY;
  string netmask = CONFIG_WIFI_STATIC_IP_SUBNET;
  string dns = CONFIG_WIFI_STATIC_IP_DNS;

  // if we have a static ip configuration convert it to the required format.
  if (!ip.empty() && !gateway.empty() && !netmask.empty())
  {
    stationIP_.reset(new openmrn_arduino::ESP32_ADAPTER_IP_INFO_TYPE());
    stationIP_->ip.addr = ipaddr_addr(ip.c_str());
    stationIP_->gw.addr = ipaddr_addr(gateway.c_str());
    stationIP_->netmask.addr = ipaddr_addr(netmask.c_str());
  }

  // if we have a dns override convert it
  if (!dns.empty())
  {
    stationDNS_.u_addr.ip4.addr = ipaddr_addr(dns.c_str());
  }

  LOG(INFO, "[WiFi] Starting WiFiManager");
  wifi_.reset(
    new Esp32WiFiManager(stack_, cfg_.seg().wifi()
                       , (wifi_mode_t)CONFIG_WIFI_MODE
                       , CONFIG_WIFI_HOSTNAME_PREFIX
                       , CONFIG_WIFI_STATION_SSID, CONFIG_WIFI_STATION_PASSWORD
                       , stationIP_.get(), stationDNS_
                       , CONFIG_WIFI_SOFTAP_SSID, CONFIG_WIFI_SOFTAP_PASSWORD
                       , CONFIG_WIFI_SOFTAP_CHANNEL
                       , nullptr /* SoftAP static ip */
                       , CONFIG_SNTP_SERVER, CONFIG_TIMEZONE
                       , false));

#if CONFIG_WIFI_DEBUG_OUTPUT
  wifi_->enable_verbose_logging();
#endif // CONFIG_WIFI_DEBUG_OUTPUT

#if CONFIG_FASTCLOCK_REALTIME
  LOG(INFO, "[FastClock] Creating Real-Time fast clock instance with ID %s"
    , uint64_to_string_hex(CONFIG_FASTCLOCK_REALTIME_ID).c_str());
  realTimeClock_.reset(
    new openlcb::BroadcastTimeServer(stack_->node()
                                  , UINT64_C(CONFIG_FASTCLOCK_REALTIME_ID)));
  realTimeClock_->set_rate_quarters(4);
#endif // CONFIG_FASTCLOCK_REALTIME
}

void LCCWiFiManager::shutdown()
{
  wifi_.reset(nullptr);
}

string LCCWiFiManager::wifi_scan_json(bool ignore_duplicates)
{
  string result = "[";
  SyncNotifiable n;
  wifi_->start_ssid_scan(&n);
  n.wait_for_notification();
  size_t num_found = wifi_->get_ssid_scan_result_count();
  vector<string> seen_ssids;
  for (int i = 0; i < num_found; i++)
  {
    auto entry = wifi_->get_ssid_scan_result(i);
    if (ignore_duplicates)
    {
      if (std::find_if(seen_ssids.begin(), seen_ssids.end()
        , [entry](string &s)
          {
            return s == (char *)entry.ssid;
          }) != seen_ssids.end())
      {
        // filter duplicate SSIDs
        continue;
      }
      seen_ssids.push_back((char *)entry.ssid);
    }
    if (result.length() > 1)
    {
      result += ",";
    }
    LOG(VERBOSE, "auth:%d,rssi:%d,ssid:%s", entry.authmode, entry.rssi, entry.ssid);
    result += StringPrintf("{\"auth\":%d,\"rssi\":%d,\"ssid\":\"%s\"}",
                           entry.authmode, entry.rssi,
                           http::url_encode((char *)entry.ssid).c_str());
  }
  result += "]";
  wifi_->clear_ssid_scan_results();
  return result;
}

#if CONFIG_FASTCLOCK_REALTIME
void LCCWiFiManager::real_time_clock_sync(time_t sync_time)
{
  LOG(INFO, "[FastClock] Time sync: %s", ctime(&sync_time));
  struct tm timeinfo;
  localtime_r(&sync_time, &timeinfo);
  realTimeClock_->set_time(timeinfo.tm_hour, timeinfo.tm_min);
  realTimeClock_->set_date(timeinfo.tm_mon + 1, timeinfo.tm_mday);
  realTimeClock_->set_year(timeinfo.tm_year + 1900);
  if (!realTimeClock_->is_running())
  {
    realTimeClock_->start();
    LOG(INFO, "[FastClock] Starting real-time clock");
  }
  else
  {
    LOG(INFO, "[FastClock] real-time clock synced");
  }
}
#endif // CONFIG_FASTCLOCK_REALTIME

} // namespace esp32cs