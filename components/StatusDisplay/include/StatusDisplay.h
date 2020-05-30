/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2017-2020 Mike Dunston

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

#ifndef STATUS_DISPLAY_H_
#define STATUS_DISPLAY_H_

#include <functional>
#include <set>
#include <string>
#include <esp_event.h>
#include <executor/StateFlow.hxx>
#include <openlcb/NodeBrowser.hxx>
#include <openlcb/SimpleStack.hxx>
#include <openlcb/TractionDefs.hxx>
#include <utils/macros.h>
#include <utils/Uninitialized.hxx>

class StatusDisplay : public StateFlowBase, public Singleton<StatusDisplay>
                    , private Atomic
{
public:
  StatusDisplay(openlcb::SimpleStackBase *, Service *);
  void stop()
  {
    set_terminated();
    timer_.ensure_triggered();
  }
  void clear();
  void info(const std::string&, ...);
  void status(const std::string&, ...);
  void wifi(const std::string&, ...);
  void track_power(const std::string&, ...);

  void node_pong(openlcb::NodeID id);
private:
  STATE_FLOW_STATE(init);
  STATE_FLOW_STATE(initOLED);
  STATE_FLOW_STATE(initLCD);
  STATE_FLOW_STATE(update);

  /// Cache of the text to display on the OLED/LCD
  std::string lines_[CONFIG_DISPLAY_LINE_COUNT];
  bool lineChanged_[CONFIG_DISPLAY_LINE_COUNT];

  uint8_t i2cAddr_;
  bool redraw_{true};
  bool sh1106_{false};
  StateFlowTimer timer_{this};
  uint8_t regZero_{0};
  uint8_t rotatingIndex_{0};
  uint8_t updateCount_{0};
  uint32_t lccExecCount_{0};
  uint32_t lccLastExecCount_{0};
  size_t lccRemoteNodeCount_{0};
  size_t lccLocalNodeCount_{0};
  bool lccNodeRefreshPending_{true};
  uint64_t nextLccNodeCountRefreshTime_{0};
  const uint64_t LCC_NODE_REFRESH_INTERVAL{SEC_TO_USEC(300)};
  std::set<openlcb::NodeID> lccSeenNodes_;
  uninitialized<openlcb::NodeBrowser> lccNodeBrowser_;
  openlcb::SimpleStackBase *stack_;
};

#endif // STATUS_DISPLAY_H_