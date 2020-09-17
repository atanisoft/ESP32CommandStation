/** \copyright
 * Copyright (c) 2019-2020, Mike Dunston
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
 * \file Dnsd.h
 *
 * Include file for the DNS server.
 *
 * @author Mike Dunston
 * @date 13 Sept 2019
 */

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
