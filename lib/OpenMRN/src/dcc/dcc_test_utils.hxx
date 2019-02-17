#ifndef _DCC_DCC_TEST_UTILS_HXX_
#define _DCC_DCC_TEST_UTILS_HXX_

#include "dcc/DccDebug.hxx"
#include "dcc/Packet.hxx"

void PrintTo(const DCCPacket &pkt, ::std::ostream *os)
{
    *os << dcc::packet_to_string(pkt);
}

namespace dcc
{
void PrintTo(const Packet &pkt, ::std::ostream *os)
{
    *os << packet_to_string(pkt);
}
}

string PrintToString(const dcc::Packet &pkt)
{
    return dcc::packet_to_string(pkt);
}

std::vector<uint8_t> dcc_from(uint8_t d0, uint8_t d1, int d2 = -1, int d3 = -1, int d4 = -1)
{
    std::vector<uint8_t> ret;
    ret.push_back(d0);
    ret.push_back(d1);
    if (d2 >= 0)
    {
        ret.push_back(d2);
    }
    else if (d2 == -2)
    {
        ret.push_back(d0 ^ d1);
    }
    if (d3 >= 0)
    {
        ret.push_back(d3);
    }
    else if (d3 == -2)
    {
        ret.push_back(d0 ^ d1 ^ ret[2]);
    }
    if (d4 >= 0)
    {
        ret.push_back(d4);
    }
    else if (d4 == -2)
    {
        ret.push_back(d0 ^ d1 ^ ret[2] ^ ret[3]);
    }
    return ret;
}

dcc::Packet packet_from(uint8_t hdr, std::vector<uint8_t> payload)
{
    dcc::Packet pkt;
    pkt.header_raw_data = hdr;
    pkt.dlc = payload.size();
    memcpy(pkt.payload, &payload[0], pkt.dlc);
    return pkt;
}

MATCHER_P2(PacketIs, hdr, payload,
    std::string(" is a packet of ") + PrintToString(packet_from(hdr, payload)))
{
    dcc::Packet pkt = packet_from(hdr, payload);
    return (pkt.header_raw_data == arg.header_raw_data && pkt.dlc == arg.dlc &&
        memcmp(pkt.payload, arg.payload, pkt.dlc) == 0);
}

#endif // _DCC_DCC_TEST_UTILS_HXX_
