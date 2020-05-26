/**********************************************************************
ESP32 COMMAND STATION

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

#ifndef SIMPLE_CALLBACK_HANDLER_H_
#define SIMPLE_CALLBACK_HANDLER_H_

#include <openlcb/CallbackEventHandler.hxx>
#include <openlcb/EventHandler.hxx>
#include <openlcb/Node.hxx>

// Simplified callback handler to automatically register the callbacks
class SimplifiedCallbackEventHandler : public openlcb::CallbackEventHandler
{
  using EventReportHandlerFn =
    openlcb::CallbackEventHandler::EventReportHandlerFn;
  using EventStateHandlerFn =
    openlcb::CallbackEventHandler::EventStateHandlerFn;
  using RegistryEntryBits =
    openlcb::CallbackEventHandler::RegistryEntryBits;
public:
  // Registers an event callback that can be for producer, consumer or both.
  SimplifiedCallbackEventHandler(openlcb::EventId eventID
                               , uint32_t callbackType
                               , openlcb::Node *node
                               , EventReportHandlerFn report_handler
                               , EventStateHandlerFn state_handler
  ) : openlcb::CallbackEventHandler(node, report_handler, state_handler)
  {
    add_entry(eventID, callbackType);
  }

  // Registers a callback for an event as a consumer only.
  SimplifiedCallbackEventHandler(openlcb::EventId id
                               , openlcb::Node *node
                               , std::function<void()> callback
  ) : openlcb::CallbackEventHandler(node
                                  , std::bind(&SimplifiedCallbackEventHandler::report
                                            , this
                                            , std::placeholders::_1
                                            , std::placeholders::_2
                                            , std::placeholders::_3
                                    )
                                  , nullptr)
  {
    add_entry(id, RegistryEntryBits::IS_CONSUMER);
  }

  // hook point for the callback invocation.
  void report(const openlcb::EventRegistryEntry &entry
            , openlcb::EventReport *report
            , BarrierNotifiable *done)
  {
    callback_();
  }
private:
  std::function<void()> callback_{nullptr};
};

#endif // SIMPLE_CALLBACK_HANDLER_H_