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

#ifndef JMRI_CLIENT_FLOW_H_
#define JMRI_CLIENT_FLOW_H_

#include <DCCppProtocol.h>
#include <executor/StateFlow.hxx>
#include <sys/socket.h>

class JmriClientFlow : private StateFlowBase, public DCCPPProtocolConsumer
{
public:
  JmriClientFlow(int fd, uint32_t remote_ip, Service *service)
    : StateFlowBase(service), DCCPPProtocolConsumer(), fd_(fd)
    , remoteIP_(remote_ip)
  {
    LOG(INFO, "[JMRI %s] Connected", name().c_str());
    bzero(buf_, BUFFER_SIZE);

    struct timeval tm;
    tm.tv_sec = 0;
    tm.tv_usec = MSEC_TO_USEC(10);
    ERRNOCHECK("setsockopt_timeout",
        setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tm, sizeof(tm)));

    start_flow(STATE(read_data));
  }

  virtual ~JmriClientFlow()
  {
    LOG(INFO, "[JMRI %s] Disconnected", name().c_str());
    ::close(fd_);
  }
private:
  static const size_t BUFFER_SIZE = 128;
  int fd_;
  uint32_t remoteIP_;
  uint8_t buf_[BUFFER_SIZE];
  size_t buf_used_{0};
  string res_;
  StateFlowTimedSelectHelper helper_{this};

  Action read_data()
  {
    // clear the buffer of data we have sent back
    res_.clear();

    return read_nonblocking(&helper_, fd_, buf_, BUFFER_SIZE
                          , STATE(process_data));
  }

  Action process_data()
  {
    if (helper_.hasError_)
    {
      return delete_this();
    }
    else if (helper_.remaining_ == BUFFER_SIZE)
    {
      return yield_and_call(STATE(read_data));
    }
    else
    {
      buf_used_ = BUFFER_SIZE - helper_.remaining_;
      LOG(VERBOSE, "[JMRI %s] received %zu bytes", name().c_str(), buf_used_);
    }
    res_.append(feed(buf_, buf_used_));
    buf_used_ = 0;
    return yield_and_call(STATE(send_data));
  }

  Action send_data()
  {
    if(res_.empty())
    {
      return yield_and_call(STATE(read_data));
    }
    return write_repeated(&helper_, fd_, res_.data(), res_.length()
                        , STATE(read_data));
  }

  string name()
  {
    return StringPrintf("%s/%d", ipv4_to_string(remoteIP_).c_str(), fd_);
  }
};

#endif // JMRI_CLIENT_FLOW_H_