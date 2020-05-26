/**********************************************************************
ESP32 HTTP Server

COPYRIGHT (c) 2019-2020 Mike Dunston

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

#ifndef DNSD_H_
#define DNSD_H_

#include <stdint.h>
#include <os/OS.hxx>
#include <utils/constants.hxx>
#include <utils/Singleton.hxx>

namespace http
{

DECLARE_CONST(dnsd_stack_size);
DECLARE_CONST(dnsd_buffer_size);

static constexpr uint16_t DEFAULT_DNS_PORT = 53;

class Dnsd : public Singleton<Dnsd>
{
public:
  Dnsd(uint32_t local_ip, std::string name = "dnsd"
     , uint16_t port = DEFAULT_DNS_PORT);
  ~Dnsd();
  void dns_process_thread();
private:
  uint32_t local_ip_;
  std::string name_;
  uint16_t port_;
  OSThread dns_thread_;
  bool shutdown_{false};
  bool shutdownComplete_{false};

  struct DNSHeader
  {
    uint16_t id;           // identification number
    union {
      struct {
        uint16_t rd  : 1;  // recursion desired
        uint16_t tc  : 1;  // truncated message
        uint16_t aa  : 1;  // authoritive answer
        uint16_t opc : 4;  // message_type
        uint16_t qr  : 1;  // query/response flag
        uint16_t rc  : 4;  // response code
        uint16_t z   : 3;  // its z! reserved
        uint16_t ra  : 1;  // recursion available
      };
      uint16_t flags;
    };
    uint16_t questions;    // number of question entries
    uint16_t answers;      // number of answer entries
    uint16_t authorties;   // number of authority entries
    uint16_t resources;    // number of resource entries
  } __attribute__((packed));

  struct DNSResponse
  {
    uint16_t id;
    uint16_t answer;
    uint16_t classes;
    uint32_t ttl;
    uint16_t length;
    uint32_t address;
  } __attribute__((packed));
};

} // namespace http

#endif // DNSD_H_
