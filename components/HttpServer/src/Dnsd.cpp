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
 * \file Dnsd.cpp
 *
 * Implementation of the DNS server.
 *
 * @author Mike Dunston
 * @date 13 Sept 2019
 */

#include "Dnsd.h"
#include "freertos_includes.h"

#include <iomanip>
#include <sys/socket.h>
#include <utils/format_utils.hxx>
#include <utils/logging.h>

namespace http
{

static void* dns_thread_start(void* arg)
{
  Singleton<Dnsd>::instance()->dns_process_thread();
  return nullptr;
}

Dnsd::Dnsd(uint32_t ip, string name, uint16_t port)
  : local_ip_(ip), name_(name), port_(port)
  , dns_thread_(name_.c_str(), config_dnsd_priority(), config_dnsd_stack_size()
              , dns_thread_start, nullptr)
  , buffer_(config_dnsd_buffer_size(), 0)
{
  
}

Dnsd::~Dnsd()
{
  shutdown_ = true;
  while(!shutdownComplete_)
  {
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void Dnsd::dns_process_thread()
{
  struct sockaddr_in addr;
  int fd;

  ERRNOCHECK("socket", fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP));

  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons(port_);
  int val = 1;
  ERRNOCHECK("setsockopt_reuseaddr",
             setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &val, sizeof(val)));
  ERRNOCHECK("bind",
             ::bind(fd, (struct sockaddr *) &addr, sizeof(addr)));

  LOG(INFO, "[%s] Listening on port %d, fd %d, using %s for local IP"
    , name_.c_str(), port_, fd, ipv4_to_string(local_ip_).c_str());
  while (!shutdown_)
  {
    sockaddr_in source;
    socklen_t source_len = sizeof(sockaddr_in);
    bzero(buffer_.data(), config_dnsd_buffer_size());
    // This will block the thread until it receives a message on the socket.
    int len = ::recvfrom(fd, buffer_.data(), config_dnsd_buffer_size()
                       , DNS_RECVFROM_FLAGS, (sockaddr *)&source, &source_len);
    if (len >= sizeof(DNSHeader))
    {
      DNSHeader *header = (DNSHeader *)buffer_.data();
      string parsed_domain_name;

      // parse the request to extract the domain name being requested
      if (ntohs(header->questions) == 1 && header->answers == 0 &&
          header->authorties == 0 && header->resources == 0)
      {
        // extract the requested domain name, it is a broken into segments
        // with the period replaced with a zero byte between segments instead
        // of the period. Max length of the domain name segments is 255 bytes
        // including the null markers.
        uint16_t offset = sizeof(DNSHeader);
        while (offset < 255 && offset < len)
        {
          uint8_t segment_len = buffer_[offset++];
          if (segment_len)
          {
            if (parsed_domain_name.length())
            {
              parsed_domain_name += '.';
            }
            parsed_domain_name.append(
              (const char *)(buffer_.data() + offset), segment_len);
            offset += segment_len;
          }
          else
          {
            break;
          }
        }
      }
      if (parsed_domain_name.empty())
      {
        // no domain name to look up, discard the request and get another.
        continue;
      }
      LOG(CONFIG_HTTP_DNS_LOG_LEVEL
        , "[%s <- %s] id: %d, rd:%d, tc:%d, aa:%d, opc:%d, qr:%d, rc:%d, z:%d "
          "ra:%d, q:%d, a:%d, au:%d, res:%d, len:%d, domain:%s"
        , name_.c_str(), inet_ntoa(source.sin_addr), header->id, header->rd
        , header->tc, header->aa, header->opc, header->qr, header->rc
        , header->z, header->ra, header->questions, header->answers
        , header->authorties, header->resources, len
        , parsed_domain_name.c_str());
      // check if it is a request or response, qr = 0 and opc = 0 is request
      if (!header->qr && !header->opc)
      {
        if ((len + sizeof(DNSResponse)) > config_dnsd_buffer_size())
        {
          LOG_ERROR("[%s] Buffer overrun (req:%d, resp:%d, buf:%d), dropping "
                    "request!"
                  , name_.c_str(), len, len + sizeof(DNSResponse)
                  , config_dnsd_buffer_size());
          continue;
        }
        // convert the request to a response by modifying the request header.
        header->qr = 1;
        header->ra = 1;
        header->answers = 1;
        // response payload
        DNSResponse response =
        {
          .id = htons(0xC00C)
        , .answer = htons(1)
        , .classes = htons(1)
        , .ttl = htonl(60)
        , .length = htons(sizeof(uint32_t))
        , .address = htonl(local_ip_)
        };
        // Add the response payload to the buffer for sending
        memcpy(buffer_.data() + len, &response, sizeof(DNSResponse));
        LOG(CONFIG_HTTP_DNS_LOG_LEVEL, "[%s -> %s] %s -> %s", name_.c_str()
          , inet_ntoa(source.sin_addr), parsed_domain_name.c_str()
          , ipv4_to_string(local_ip_).c_str());
        // send the response to the caller
        ERRNOCHECK("sendto", sendto(fd, buffer_.data()
                                  , len + sizeof(DNSResponse), DNS_SENDTO_FLAGS
                                  , (const sockaddr*)&source
                                  , sizeof(sockaddr_in)));
      }
    }
    else
    {
      if (errno == EAGAIN || errno == ECONNRESET || errno == ENOTCONN ||
          errno == ETIMEDOUT)
      {
        continue;
      }
      print_errno_and_exit("recvfrom");
    }
  }
  shutdownComplete_ = true;
}

} // namespace http
