/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2017-2019 Mike Dunston

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

#include "ESP32CommandStation.h"
#include "Turnouts.h"
#include "S88Sensors.h"
#include "RemoteSensors.h"
#include "HC12Interface.h"
#include "NextionInterface.h"

#include <esp_int_wdt.h>
#include <esp_task_wdt.h>

const char * buildTime = __DATE__ " " __TIME__;

std::vector<uint8_t> restrictedPins;

#if LOCONET_ENABLED
LocoNetESP32Uart locoNet(LOCONET_RX_PIN, LOCONET_TX_PIN, LOCONET_UART, LOCONET_INVERTED_LOGIC, LOCONET_ENABLE_RX_PIN_PULLUP);
#endif

bool otaComplete = false;
bool otaInProgress = false;

// esp32 doesn't have a true restart method exposed so use the watchdog to
// force a restart
void esp32_restart() {
  esp_task_wdt_init(1, true);
  esp_task_wdt_add(NULL);
  while(true);
}

void setup() {
  Serial.begin(115200L);
  Serial.setDebugOutput(true);
  LOG(INFO, "ESP32 Command Station v%s starting up", VERSION);
#ifndef ALLOW_USAGE_OF_RESTRICTED_GPIO_PINS
  restrictedPins.push_back(0);
  restrictedPins.push_back(2);
  restrictedPins.push_back(5);
  restrictedPins.push_back(6);
  restrictedPins.push_back(7);
  restrictedPins.push_back(8);
  restrictedPins.push_back(9);
  restrictedPins.push_back(10);
  restrictedPins.push_back(11);
  restrictedPins.push_back(12);
  restrictedPins.push_back(15);
#endif

#if STATUS_LED_ENABLED
  initStatusLEDs();
#endif

  // set up ADC1 here since we use it for all motor boards
  adc1_config_width(ADC_WIDTH_BIT_12);

  InfoScreen::init();
  InfoScreen::replaceLine(INFO_SCREEN_STATION_INFO_LINE, F("ESP32-CS: v%s"), VERSION);
  InfoScreen::replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("Starting Up"));
#if INFO_SCREEN_STATION_INFO_LINE == INFO_SCREEN_IP_ADDR_LINE
  delay(500);
#endif
#if NEXTION_ENABLED
  nextionInterfaceInit();
#endif
  configStore.init();
#if USE_RMT_FOR_DCC
  dccSignal[DCC_SIGNAL_OPERATIONS] = new SignalGenerator_RMT("OPS", 512, DCC_SIGNAL_OPERATIONS, DCC_SIGNAL_PIN_OPERATIONS);
  dccSignal[DCC_SIGNAL_PROGRAMMING] = new SignalGenerator_RMT("PROG", 10, DCC_SIGNAL_PROGRAMMING, DCC_SIGNAL_PIN_PROGRAMMING);
#else
  dccSignal[DCC_SIGNAL_OPERATIONS] = new SignalGenerator_HardwareTimer("OPS", 512, DCC_SIGNAL_OPERATIONS, DCC_SIGNAL_PIN_OPERATIONS);
  dccSignal[DCC_SIGNAL_PROGRAMMING] = new SignalGenerator_HardwareTimer("PROG", 10, DCC_SIGNAL_PROGRAMMING, DCC_SIGNAL_PIN_PROGRAMMING);
#endif
#if LCC_ENABLED
  lccInterface.init();
#endif
  wifiInterface.begin();
  MotorBoardManager::registerBoard(MOTORBOARD_CURRENT_SENSE_OPS,
                                   MOTORBOARD_ENABLE_PIN_OPS,
                                   MOTORBOARD_TYPE_OPS,
                                   MOTORBOARD_NAME_OPS);
  MotorBoardManager::registerBoard(MOTORBOARD_CURRENT_SENSE_PROG,
                                   MOTORBOARD_ENABLE_PIN_PROG,
                                   MOTORBOARD_TYPE_PROG,
                                   MOTORBOARD_NAME_PROG,
                                   true);
#if INFO_SCREEN_TRACK_POWER_LINE >= 0
  InfoScreen::replaceLine(INFO_SCREEN_TRACK_POWER_LINE, F("TRACK POWER: OFF"));
#endif
  DCCPPProtocolHandler::init();
  OutputManager::init();
  TurnoutManager::init();
  SensorManager::init();
#if S88_ENABLED
  S88BusManager::init();
#endif
  RemoteSensorManager::init();
  LocomotiveManager::init();
#if HC12_RADIO_ENABLED
  HC12Interface::init();
#endif
#if LOCONET_ENABLED
  InfoScreen::replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("LocoNet Init"));
  locoNet.begin();
  locoNet.onPacket(OPC_GPON, [](lnMsg *msg) {
    MotorBoardManager::powerOnAll();
  });
  locoNet.onPacket(OPC_GPOFF, [](lnMsg *msg) {
    MotorBoardManager::powerOffAll();
  });
  locoNet.onPacket(OPC_IDLE, [](lnMsg *msg) {
    LocomotiveManager::emergencyStop();
  });
  locoNet.onPacket(OPC_LOCO_ADR, [](lnMsg *msg) {
    lnMsg response = {0};
    auto loco = LocomotiveManager::getLocomotive(msg->la.adr_lo + (msg->la.adr_hi << 7));
    response.sd.command = OPC_SL_RD_DATA;
    response.sd.mesg_size = 0x0E;
    response.sd.slot = loco->getRegister();
    response.sd.stat = LOCO_IDLE | DEC_MODE_128;
    response.sd.adr = msg->la.adr_lo;
    response.sd.adr2 = msg->la.adr_hi;
    response.sd.dirf = DIRF_F0;
    response.sd.trk = GTRK_MLOK1;
    if(MotorBoardManager::isTrackPowerOn()) {
      response.sd.trk |= GTRK_POWER;
    }
    if(progTrackBusy) {
      response.sd.trk |= GTRK_PROG_BUSY;
    }
    locoNet.send(&response);
  });
  locoNet.onPacket(OPC_LOCO_SPD, [](lnMsg *msg) {
    auto loco = LocomotiveManager::getLocomotiveByRegister(msg->lsp.slot);
    if(loco) {
      loco->setSpeed(msg->lsp.spd);
    } else {
      locoNet.send(OPC_LONG_ACK, OPC_LOCO_SPD, 0);
    }
  });
  locoNet.onPacket(OPC_LOCO_DIRF, [](lnMsg *msg) {
    auto loco = LocomotiveManager::getLocomotiveByRegister(msg->ldf.slot);
    if(loco) {
      loco->setDirection(msg->ldf.dirf & DIRF_DIR);
      loco->setFunction(0, msg->ldf.dirf & DIRF_F0);
      loco->setFunction(1, msg->ldf.dirf & DIRF_F1);
      loco->setFunction(2, msg->ldf.dirf & DIRF_F2);
      loco->setFunction(3, msg->ldf.dirf & DIRF_F3);
      loco->setFunction(4, msg->ldf.dirf & DIRF_F4);
    } else {
      locoNet.send(OPC_LONG_ACK, OPC_LOCO_DIRF, 0);
    }
  });
  locoNet.onPacket(OPC_LOCO_SND, [](lnMsg *msg) {
    auto loco = LocomotiveManager::getLocomotiveByRegister(msg->ls.slot);
    if(loco) {
      loco->setFunction(5, msg->ls.snd & SND_F5);
      loco->setFunction(6, msg->ls.snd & SND_F6);
      loco->setFunction(7, msg->ls.snd & SND_F7);
      loco->setFunction(8, msg->ls.snd & SND_F8);
    } else {
      locoNet.send(OPC_LONG_ACK, OPC_LOCO_SND, 0);
    }
  });
  locoNet.onPacket(OPC_WR_SL_DATA, [](lnMsg *msg) {
    if(msg->pt.slot == PRG_SLOT) {
      if(msg->pt.command == 0x00) {
        // Cancel / abort request, currently ignored
      } else if (progTrackBusy) {
        locoNet.send(OPC_LONG_ACK, OPC_MASK, 0);
      } else {
        uint16_t cv = PROG_CV_NUM(msg->pt);
        uint8_t value = PROG_DATA(msg->pt);
        if((msg->pt.command & DIR_BYTE_ON_SRVC_TRK) == 0 &&
          (msg->pt.command & PCMD_RW) == 1) { // CV Write on PROG
          if(enterProgrammingMode()) {
            locoNet.send(OPC_LONG_ACK, OPC_MASK, 1);
            msg->pt.command = OPC_SL_RD_DATA;
            if(!writeProgCVByte(cv, value)) {
              msg->pt.pstat = PSTAT_WRITE_FAIL;
            } else {
              msg->pt.data7 = value;
              if(value & 0x80) {
                msg->pt.cvh |= CVH_D7;
              }
            }
            leaveProgrammingMode();
            locoNet.send(msg);
          } else {
            locoNet.send(OPC_LONG_ACK, OPC_MASK, 0);
          }
        } else if((msg->pt.command & DIR_BYTE_ON_SRVC_TRK) == 0 &&
          (msg->pt.command & PCMD_RW) == 0) { // CV Read on PROG
          if(enterProgrammingMode()) {
            locoNet.send(OPC_LONG_ACK, OPC_MASK, 1);
            msg->pt.command = OPC_SL_RD_DATA;
            int16_t value = readCV(cv);
            if(value == -1) {
              msg->pt.pstat = PSTAT_READ_FAIL;
            } else {
              msg->pt.data7 = value & 0x7F;
              if(value & 0x80) {
                msg->pt.cvh |= CVH_D7;
              }
            }
            leaveProgrammingMode();
            locoNet.send(msg);
          } else {
            locoNet.send(OPC_LONG_ACK, OPC_MASK, 0);
          }
        } else if ((msg->pt.command & OPS_BYTE_NO_FEEDBACK) == 0) {
          // CV Write on OPS, no feedback
          locoNet.send(OPC_LONG_ACK, OPC_MASK, 0x40);
          uint16_t locoAddr = ((msg->pt.hopsa & 0x7F) << 7) + (msg->pt.lopsa & 0x7F);
          writeOpsCVByte(locoAddr, cv, value);
        } else if ((msg->pt.command & OPS_BYTE_FEEDBACK) == 0) {
          // CV Write on OPS
          locoNet.send(OPC_LONG_ACK, OPC_MASK, 1);
          uint16_t locoAddr = ((msg->pt.hopsa & 0x7F) << 7) + (msg->pt.lopsa & 0x7F);
          writeOpsCVByte(locoAddr, cv, value);
          msg->pt.command = OPC_SL_RD_DATA;
          if(value & 0x80) {
            msg->pt.cvh |= CVH_D7;
          }
          msg->pt.data7 = value & 0x7F;
          locoNet.send(msg);
        } else {
          // not implemented
          locoNet.send(OPC_LONG_ACK, OPC_MASK, OPC_MASK);
        }
      }
    }
  });
  locoNet.onPacket(OPC_INPUT_REP, [](lnMsg *msg) {
    LOG(INFO, "LocoNet INPUT_REPORT %02x : %02x", msg->ir.in1, msg->ir.in2);
  });
  locoNet.onPacket(OPC_SW_REQ, [](lnMsg *msg) {
    LOG(INFO, "LocoNet SW_REQ %02x : %02x", msg->srq.sw1, msg->srq.sw2);
  });
  locoNet.onPacket(OPC_SW_REP, [](lnMsg *msg) {
    LOG(INFO, "LocoNet SW_REP %02x : %02x", msg->srp.sn1, msg->srp.sn2);
  });
#endif

#if ENERGIZE_OPS_TRACK_ON_STARTUP
  MotorBoardManager::powerOnAll();
#endif

  LOG(INFO, "ESP32 Command Station READY!");
  InfoScreen::replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("ESP32-CS READY!"));
}

void loop() {
  if(otaComplete) {
    LOG(INFO, "OTA binary has been received, preparing to reboot!");
    delay(250);
    esp32_restart();
  }
  MotorBoardManager::check();
  LocomotiveManager::update();
  if(!otaInProgress) {
    InfoScreen::update();
#if LCC_ENABLED
    lccInterface.update();
#endif
#if HC12_RADIO_ENABLED
    HC12Interface::update();
#endif
  }
}
