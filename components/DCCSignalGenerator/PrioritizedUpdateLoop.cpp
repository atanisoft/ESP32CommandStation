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

#include "PrioritizedUpdateLoop.hxx"

#include <algorithm>
#include <dcc/PacketSource.hxx>
#include <utils/constants.hxx>

namespace esp32cs
{

DECLARE_CONST(min_refresh_delay_us);

struct HighPriorityUpdate
{
  void reset(dcc::PacketSource *source, unsigned code)
  {
    this->source = source;
    this->code = code;
  }
  dcc::PacketSource *source;
  unsigned code;
};

PrioritizedUpdateLoop::PrioritizedUpdateLoop(Service *service
                                           , dcc::PacketFlowInterface *track)
  : StateFlow<Buffer<dcc::Packet>, QList<1>>(service)
  , track_(track)
  , exclusiveSourceIndex_(NO_EXCLUSIVE_SOURCE)
{

}

PrioritizedUpdateLoop::~PrioritizedUpdateLoop()
{
  const std::lock_guard<std::mutex> lock(mux_);
  refreshSources_.clear();
  refreshSourceMetrics_.clear();
}

bool PrioritizedUpdateLoop::add_refresh_source(dcc::PacketSource *source
                                             , unsigned priority)
{
  const std::lock_guard<std::mutex> lock(mux_);
  bool highest_priority = true;

  // record the new packet source
  refreshSources_.push_back(source);

  // seed the tracking metrics
  auto &metrics = refreshSourceMetrics_[source];
  metrics.priority = priority;
  metrics.lastPacketTimestamp = 0;

  // recalculate priorities (if needed)
  if (priority >= dcc::UpdateLoopBase::EXCLUSIVE_MIN_PRIORITY)
  {
    unsigned existing_priority = 0;
    // Check if we have an existing exclusive packet source, if so check if the
    // new packet source has a higher priority or not.
    if (exclusiveSourceIndex_ != NO_EXCLUSIVE_SOURCE)
    {
      existing_priority =
        refreshSourceMetrics_[refreshSources_[exclusiveSourceIndex_]].priority;
    }
    if (priority > existing_priority)
    {
      // new source has a higher priority, update index
      exclusiveSourceIndex_ = refreshSources_.size() - 1;
    }
    else
    {
      // There is a higher priority source already, inform the caller that the
      // new source will have lower priority.
      highest_priority = false;
    }
  }
  else if (exclusiveSourceIndex_ != NO_EXCLUSIVE_SOURCE)
  {
    // we have an 
    highest_priority = false;
  }
  return highest_priority;
}

void PrioritizedUpdateLoop::remove_refresh_source(dcc::PacketSource *source)
{
  const std::lock_guard<std::mutex> lock(mux_);
  refreshSources_.erase(
    std::remove(refreshSources_.begin(), refreshSources_.end(), source)
  , refreshSources_.end());
  refreshSourceMetrics_.erase(source);

  // recalculate exclusive packet source priorities for next highest
  size_t max_priority = dcc::UpdateLoopBase::EXCLUSIVE_MIN_PRIORITY;
  size_t max_index = NO_EXCLUSIVE_SOURCE;
  size_t index = 0;
  for (auto s : refreshSources_)
  {
    if (refreshSourceMetrics_[s].priority > max_priority)
    {
      max_priority = refreshSourceMetrics_[s].priority;
      max_index = index;
    }
    index++;
  }
  exclusiveSourceIndex_ = max_index;
}

void PrioritizedUpdateLoop::notify_update(dcc::PacketSource* source
                                        , unsigned code)
{
  Buffer<HighPriorityUpdate> *buf;
  mainBufferPool->alloc(&buf, nullptr);
  HASSERT(buf);
  buf->data()->reset(source, code);

  {
    const std::lock_guard<std::mutex> lock(mux_);
    priorityUpdateSources_.insert(buf, 0);
  }
}

StateFlowBase::Action PrioritizedUpdateLoop::entry()
{
  dcc::PacketSource *next_source = nullptr;
  Buffer<HighPriorityUpdate> *high_priority = nullptr;
  
  {
    const std::lock_guard<std::mutex> lock(mux_);
    if (exclusiveSourceIndex_ != NO_EXCLUSIVE_SOURCE)
    {
      next_source = refreshSources_[exclusiveSourceIndex_];
    }
    else
    {
      high_priority =
        static_cast<Buffer<HighPriorityUpdate>*>(priorityUpdateSources_.next().item);
    }
  }
  uint64_t time_now = esp_timer_get_time();
  unsigned code = 0;
  // if we have a high priority update ready, send it out
  if (high_priority)
  {
    code = high_priority->data()->code;

    // check if the source has been removed
    auto metrics = refreshSourceMetrics_.find(next_source);
    if (metrics == refreshSourceMetrics_.end())
    {
      high_priority->unref();
    }
    else if (metrics->second.lastPacketTimestamp >
             (time_now - config_min_refresh_delay_us()))
    {
      // we sent a packet to this source within the minimum refresh window
      // send this source back to the queue.
      const std::lock_guard<std::mutex> lock(mux_);
      priorityUpdateSources_.insert(high_priority, 0);
    }
    else
    {
      // all checks have been validated, we can use this high priority source
      // for the next packet.
      next_source = high_priority->data()->source;
    }
  }
  // if we don't have a packet source already and we have at least one source
  // to pick from, find one that hasn't been updated recently and use it.
  if (!next_source && !refreshSources_.empty())
  {
    // scan all refresh sources to find which (if any) has not been recently
    // refreshed.
    for (size_t idx = 0; idx < refreshSources_.size(); idx++)
    {
      {
        const std::lock_guard<std::mutex> lock(mux_);
        if (nextRefreshIndex_ > refreshSources_.size())
        {
            nextRefreshIndex_ = 0;
        }
        next_source = refreshSources_[nextRefreshIndex_++];
        code = 0;
      }
      if (refreshSourceMetrics_[next_source].lastPacketTimestamp <
          (time_now - config_min_refresh_delay_us()))
      {
        // enough time has passed for this packet source, we can send to it.
        break;
      }
      else
      {
        // insufficient time has passed, skip this source
        next_source = nullptr;
      }
    }
  }

  if (next_source)
  {
    // we have a new source, get the next packet from the source
    next_source->get_next_packet(code, message()->data());

    // track that we have sent a packet to this source recently
    refreshSourceMetrics_[next_source].lastPacketTimestamp = time_now;
  }
  else
  {
    // no packet source generated a packet, convert the packet to idle.
    message()->data()->set_dcc_idle();
  }

  // transfer the packet to the track interface
  track_->send(transfer_message());

  // exit the stateflow
  return exit();
}
} // namespace esp32cs