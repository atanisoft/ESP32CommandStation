/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2020-2021 Mike Dunston

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
#include <esp_timer.h>
#include <utils/constants.hxx>

namespace esp32cs
{

using dcc::PacketSource;
using dcc::PacketFlowInterface;
using dcc::UpdateLoopBase;

DECLARE_CONST(min_refresh_delay_us);

struct UpdateRequest
{
  void reset(PacketSource *source, unsigned code)
  {
    this->source = source;
    this->code = code;
  }
  PacketSource *source;
  unsigned code;
};

PrioritizedUpdateLoop::PrioritizedUpdateLoop(Service *service,
                                             PacketFlowInterface *track)
  : StateFlow<Buffer<dcc::Packet>, QList<1>>(service),
    track_(track)
{

}

PrioritizedUpdateLoop::~PrioritizedUpdateLoop()
{
  sources_.clear();
  metrics_.clear();
}

bool PrioritizedUpdateLoop::add_refresh_source(PacketSource *source,
                                               unsigned priority)
{
  const std::lock_guard<std::mutex> lock(mux_);

  // record the new packet source
  sources_.push_back(source);

  // seed the tracking metrics
  auto &metrics = metrics_[source];
  metrics.priority = priority;
  metrics.lastPacketTimestamp = 0;

  if (priority > UpdateLoopBase::EXCLUSIVE_MIN_PRIORITY)
  {
    unsigned highest_priority = 0;
    if (exclusiveIndex_ != NO_EXCLUSIVE_SOURCE)
    {
      highest_priority = metrics_[sources_[exclusiveIndex_]].priority;
    }
    if (priority > highest_priority)
    {
      exclusiveIndex_ = sources_.size() - 1;
    }
    else
    {
      return false;
    }
  }
  else if (exclusiveIndex_ != NO_EXCLUSIVE_SOURCE)
  {
    return false;
  }
  return true;
}

void PrioritizedUpdateLoop::remove_refresh_source(PacketSource *source)
{
  const std::lock_guard<std::mutex> lock(mux_);
  sources_.erase(
    std::remove(sources_.begin(), sources_.end(), source), sources_.end());
  metrics_.erase(source);

  // if there are no packet sources there can't be an exclusive so exit early.
  if (sources_.empty())
  {
    exclusiveIndex_ = NO_EXCLUSIVE_SOURCE;
    return;
  }

  // recalculate for packet priority based on exclusive sources
  unsigned highest_priority = UpdateLoopBase::EXCLUSIVE_MIN_PRIORITY;
  unsigned highest_priority_index = NO_EXCLUSIVE_SOURCE;
  for (size_t index = 0; index < sources_.size(); index++)
  {
    const auto &packet_source = sources_[index];
    if (metrics_[packet_source].priority > highest_priority)
    {
      highest_priority = metrics_[packet_source].priority;
      highest_priority_index = index;
    }
  }
  exclusiveIndex_ = highest_priority_index;
}

void PrioritizedUpdateLoop::notify_update(PacketSource* source, unsigned code)
{
  // prepare the high priority update before we lock the queue
  Buffer<UpdateRequest> *buf;
  mainBufferPool->alloc(&buf, nullptr);
  HASSERT(buf);
  buf->data()->reset(source, code);

  // lock the queue and insert the new update packet for processing
  {
    const std::lock_guard<std::mutex> lock(mux_);
    updateSources_.insert(buf, 0);
  }
}

StateFlowBase::Action PrioritizedUpdateLoop::entry()
{
  {
    dcc::PacketSource *source = nullptr;
    uint64_t current_time = esp_timer_get_time();
    uint64_t threshold = current_time - config_min_refresh_delay_us();
    unsigned code = 0;

    const std::lock_guard<std::mutex> lock(mux_);
    // if we have an exclusive source use it as the source otherwise check if
    // there is a priority update to send out.
    if (exclusiveIndex_ != NO_EXCLUSIVE_SOURCE)
    {
      source = sources_[exclusiveIndex_];
    }
    else if (updateSources_.pending())
    {
      Buffer<UpdateRequest> *update =
        static_cast<Buffer<UpdateRequest>*>(updateSources_.next().item);
      code = update->data()->code;
      const auto &update_source = update->data()->source;

      auto metrics = metrics_.find(update_source);
      if (metrics == metrics_.end())
      {
        // priority update source has disappeared, discard and find another
        // packet source.
        update->unref();
      }
      else if (metrics->second.lastPacketTimestamp > threshold)
      {
        // we sent a packet to this source within the minimum refresh window
        // send this source back to the queue.
        updateSources_.insert(update, 0);
      }
      else
      {
        // all checks have been validated, we can use this high priority source
        // for the next packet.
        source = update_source;
      }
    }

    if (source == nullptr)
    {
      // scan all refresh sources to find which (if any) has not been recently
      // refreshed.
      for (size_t idx = 0; idx < sources_.size(); idx++)
      {
        nextIndex_ %= sources_.size();
        auto ref = sources_[nextIndex_++];
        if (ref && metrics_[ref].lastPacketTimestamp < threshold)
        {
          // enough time has passed for this packet source, we can send to it.
          code = 0;
          source = ref;
          break;
        }
      }
    }

    if (source)
    {
      // we have a new source, get the next packet from the source
      source->get_next_packet(code, message()->data());

      // track that we have sent a packet to this source recently
      metrics_[source].lastPacketTimestamp = current_time;
    }
    else
    {
      // no packet source generated a packet, convert the packet to idle.
      message()->data()->set_dcc_idle();
    }
  }

  // transfer the packet to the track interface
  track_->send(transfer_message());

  // exit the stateflow
  return exit();
}

} // namespace esp32cs
