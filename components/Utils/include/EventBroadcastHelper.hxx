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

#ifndef EVENT_BROADCAST_HELPER_HXX_
#define EVENT_BROADCAST_HELPER_HXX_

#include <openlcb/SimpleStack.hxx>
#include <utils/Singleton.hxx>

namespace esp32cs
{

/// Utility class that will send an event out onto the bus.
class EventBroadcastHelper : public Singleton<EventBroadcastHelper>
{
public:
    /// Constructor.
    ///
    /// @param service is the @ref Service that will execute this flow.
    EventBroadcastHelper(openlcb::SimpleCanStack *stack) : stack_(stack)
    {
    }

    void send_event(uint64_t eventID)
    {
        stack_->executor()->add(new CallbackExecutable([&]()
        {
            stack_->send_event(eventID);
        }));
    }
private:
    openlcb::SimpleCanStack *stack_;
};

} // namespace esp32cs

#endif // EVENT_BROADCAST_HELPER_HXX_