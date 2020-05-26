/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2019 Mike Dunston

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

#include "DCCProgrammer.h"

#include <dcc/ProgrammingTrackBackend.hxx>
#include <dcc/DccDebug.hxx>
#include <DuplexedTrackIf.h>
#include <utils/Uninitialized.hxx>

// number of attempts the programming track will make to read/write a CV
static constexpr uint8_t PROG_TRACK_CV_ATTEMPTS = 3;

static bool enterServiceMode()
{
  BufferPtr<ProgrammingTrackRequest> req =
    invoke_flow(Singleton<ProgrammingTrackBackend>::instance()
              , ProgrammingTrackRequest::ENTER_SERVICE_MODE);
  return req->data()->resultCode == 0;
}

static void leaveServiceMode()
{
  invoke_flow(Singleton<ProgrammingTrackBackend>::instance()
            , ProgrammingTrackRequest::EXIT_SERVICE_MODE);
}

static bool sendServiceModeDecoderReset()
{
  BufferPtr<ProgrammingTrackRequest> req =
    invoke_flow(Singleton<ProgrammingTrackBackend>::instance()
              , ProgrammingTrackRequest::SEND_RESET, 15);
  return (req->data()->resultCode == 0);
}

static bool sendServiceModePacketWithAck(dcc::Packet pkt)
{
  BufferPtr<ProgrammingTrackRequest> req =
   invoke_flow(Singleton<ProgrammingTrackBackend>::instance()
             , ProgrammingTrackRequest::SEND_PROGRAMMING_PACKET, pkt
             , 15);
  return req->data()->hasAck_;
}

static bool executeProgTrackWriteRequest(dcc::Packet pkt)
{
  if (enterServiceMode())
  {
    LOG(VERBOSE, "[PROG] Resetting DCC Decoder");
    if (!sendServiceModeDecoderReset())
    {
      leaveServiceMode();
      return false;
    }
    LOG(VERBOSE, "[PROG] Sending DCC packet: %s", dcc::packet_to_string(pkt).c_str());
    if (!sendServiceModePacketWithAck(pkt))
    {
      leaveServiceMode();
      return false;
    }
    LOG(VERBOSE, "[PROG] Resetting DCC Decoder (after PROG)");
    if (!sendServiceModeDecoderReset())
    {
      leaveServiceMode();
      return false;
    }
    leaveServiceMode();
    return true;
  }
  return false;
}

int16_t readCV(const uint16_t cv)
{
  int16_t value = -1;
  if (enterServiceMode())
  {
    for(int attempt = 0; attempt < PROG_TRACK_CV_ATTEMPTS && value == -1; attempt++) {
      LOG(INFO, "[PROG %d/%d] Attempting to read CV %d", attempt+1, PROG_TRACK_CV_ATTEMPTS, cv);
      // reset cvValue to all bits OFF
      value = 0;
      for(uint8_t bit = 0; bit < 8; bit++) {
        LOG(VERBOSE, "[PROG %d/%d] CV %d, bit [%d/7]", attempt+1, PROG_TRACK_CV_ATTEMPTS, cv, bit);
        dcc::Packet pkt;
        pkt.start_dcc_svc_packet();
        pkt.add_dcc_prog_command(0x78, cv, 0xE8 + bit);
        if (sendServiceModePacketWithAck(pkt))
        {
          LOG(VERBOSE, "[PROG %d/%d] CV %d, bit [%d/7] ON", attempt+1, PROG_TRACK_CV_ATTEMPTS, cv, bit);
          value &= (1 << bit);
        } else {
          LOG(VERBOSE, "[PROG %d/%d] CV %d, bit [%d/7] OFF", attempt+1, PROG_TRACK_CV_ATTEMPTS, cv, bit);
        }
      }
      dcc::Packet pkt;
      pkt.set_dcc_svc_verify_byte(cv, value);
      if (sendServiceModePacketWithAck(pkt))
      {
        LOG(INFO, "[PROG %d/%d] CV %d, verified as %d", attempt+1, PROG_TRACK_CV_ATTEMPTS, cv, value);
      }
      else
      {
        LOG(WARNING, "[PROG %d/%d] CV %d, could not be verified", attempt+1, PROG_TRACK_CV_ATTEMPTS, cv);
        value = -1;
      }
    }
    LOG(INFO, "[PROG] CV %d value is %d", cv, value);
    leaveServiceMode();
  }
  else
  {
    LOG_ERROR("[PROG] Failed to enter programming mode!");
  }
  return value;
}

bool writeProgCVByte(const uint16_t cv, const uint8_t value)
{
  bool writeVerified = false;
  dcc::Packet pkt, verifyPkt;
  pkt.set_dcc_svc_write_byte(cv, value);
  verifyPkt.set_dcc_svc_verify_byte(cv, value);
  
  for(uint8_t attempt = 1;
      attempt <= PROG_TRACK_CV_ATTEMPTS && !writeVerified;
      attempt++)
  {
    LOG(INFO, "[PROG %d/%d] Attempting to write CV %d as %d", attempt
      , PROG_TRACK_CV_ATTEMPTS, cv, value);

    if (executeProgTrackWriteRequest(pkt) &&
        executeProgTrackWriteRequest(verifyPkt))
    {
      // write byte and verify byte were successful
      writeVerified = true;
    }

    if (!writeVerified)
    {
      LOG(WARNING, "[PROG %d/%d] CV %d write value %d could not be verified."
        , attempt, PROG_TRACK_CV_ATTEMPTS, cv, value);
    }
    else
    {
      LOG(INFO, "[PROG %d/%d] CV %d write value %d verified.", attempt
        , PROG_TRACK_CV_ATTEMPTS, cv, value);
    }
  }
  return writeVerified;
}

bool writeProgCVBit(const uint16_t cv, const uint8_t bit, const bool value)
{
  bool writeVerified = false;
  dcc::Packet pkt, verifyPkt;
  pkt.set_dcc_svc_write_bit(cv, bit, value);
  verifyPkt.set_dcc_svc_verify_bit(cv, bit, value);

  for(uint8_t attempt = 1;
      attempt <= PROG_TRACK_CV_ATTEMPTS && !writeVerified;
      attempt++) {
    LOG(INFO, "[PROG %d/%d] Attempting to write CV %d bit %d as %d", attempt
      , PROG_TRACK_CV_ATTEMPTS, cv, bit, value);
    if (executeProgTrackWriteRequest(pkt) &&
        executeProgTrackWriteRequest(verifyPkt))
    {
      // write byte and verify byte were successful
      writeVerified = true;
    }

    if (!writeVerified)
    {
      LOG(WARNING, "[PROG %d/%d] CV %d write bit %d could not be verified."
        , attempt, PROG_TRACK_CV_ATTEMPTS, cv, bit);
    }
    else
    {
      LOG(INFO, "[PROG %d/%d] CV %d write bit %d verified.", attempt
        , PROG_TRACK_CV_ATTEMPTS, cv, bit);
    }
  }
  return writeVerified;
}

void writeOpsCVByte(const uint16_t locoAddress, const uint16_t cv
                  , const uint8_t cvValue)
{
  auto track = Singleton<esp32cs::DuplexedTrackIf>::instance();
  auto b = get_buffer_deleter(track->alloc());
  if (b)
  {
    LOG(INFO, "[OPS] Updating CV %d to %d for loco %d", cv, cvValue
      , locoAddress);

    b->data()->start_dcc_packet();
    if(locoAddress > 127)
    {
      b->data()->add_dcc_address(dcc::DccLongAddress(locoAddress));
    }
    else
    {
      b->data()->add_dcc_address(dcc::DccShortAddress(locoAddress));
    }
    b->data()->add_dcc_pom_write1(cv, cvValue);
    b->data()->packet_header.rept_count = 3;
    track->send(b.get());
  }
  else
  {
    LOG_ERROR("[OPS] Failed to retrieve DCC Packet for programming request");
  }
}

void writeOpsCVBit(const uint16_t locoAddress, const uint16_t cv
                 , const uint8_t bit, const bool value)
{
  auto track = Singleton<esp32cs::DuplexedTrackIf>::instance();
  auto b = get_buffer_deleter(track->alloc());
  if (b)
  {
    LOG(INFO, "[OPS] Updating CV %d bit %d to %d for loco %d", cv, bit, value
      , locoAddress);
    b->data()->start_dcc_packet();
    if(locoAddress > 127)
    {
      b->data()->add_dcc_address(dcc::DccLongAddress(locoAddress));
    }
    else
    {
      b->data()->add_dcc_address(dcc::DccShortAddress(locoAddress));
    }
    // TODO add_dcc_pom_write_bit(cv, bit, value)
    b->data()->add_dcc_prog_command(0xe8, cv - 1
                                  , (uint8_t)(0xF0 + bit + value * 8));
    b->data()->packet_header.rept_count = 3;
    track->send(b.get());
  }
  else
  {
    LOG_ERROR("[OPS] Failed to retrieve DCC Packet for programming request");
  }
}
