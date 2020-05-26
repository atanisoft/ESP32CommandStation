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

#include "Dnsd.h"
#include <utils/logging.h>
#include <utils/format_utils.hxx>
#include <sys/socket.h>
#include <iomanip>

namespace http
{

static void* dns_thread_start(void* arg)
{
  Singleton<Dnsd>::instance()->dns_process_thread();
  return nullptr;
}

Dnsd::Dnsd(uint32_t ip, string name, uint16_t port)
  : local_ip_(ip), name_(name), port_(port)
  , dns_thread_(name_.c_str(), 1, config_dnsd_stack_size(), dns_thread_start
              , nullptr)
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
  vector<uint8_t> receiveBuffer(config_dnsd_buffer_size(), 0);
  vector<uint8_t> responseBuffer;

  while (!shutdown_)
  {
    receiveBuffer.clear();
    sockaddr_in source;
    socklen_t source_len = sizeof(sockaddr_in);
    // This will block the thread until it receives a message on the socket.
    int len = ::recvfrom(fd, receiveBuffer.data(), config_dnsd_buffer_size(), 0
                       , (sockaddr *)&source, &source_len);
    if(len >= sizeof(DNSHeader))
    {
      DNSHeader *header = (DNSHeader *)receiveBuffer.data();
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
          uint8_t segment_len = receiveBuffer.data()[offset++];
          if (segment_len)
          {
            if (parsed_domain_name.length())
            {
              parsed_domain_name += '.';
            }
            parsed_domain_name.append(
              (const char *)(receiveBuffer.data() + offset), segment_len);
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
        // convert the request to a response by modifying the request header
        // in place (since it gets copied to the response as-is).
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
        responseBuffer.resize(len + sizeof(DNSResponse));
        memcpy(responseBuffer.data(), receiveBuffer.data(), len);
        memcpy(responseBuffer.data() + len, &response, sizeof(DNSResponse));
        LOG(CONFIG_HTTP_DNS_LOG_LEVEL, "[%s -> %s] %s -> %s", name_.c_str()
          , inet_ntoa(source.sin_addr), parsed_domain_name.c_str()
          , ipv4_to_string(local_ip_).c_str());
        ERRNOCHECK("sendto", sendto(fd, responseBuffer.data()
                                  , responseBuffer.size(), 0
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
