/** \copyright
 * Copyright (c) 2015, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \file TractionCvSpace.hxx
 *
 * A memory config protocol compatible Memory Space for reading and writing CVs
 * of DCC locomotives.
 *
 * @author Balazs Racz
 * @date 16 May 2015
 */

#include "openlcb/TractionCvSpace.hxx"
#include "dcc/ProgrammingTrackBackend.hxx"
#include "openlcb/TractionDefs.hxx"

// We try this many times to write a CV using railcom if we keep getting an
// unknown railcom response. After that we try to verify with a read.
static const int WRITE_RETRY_COUNT_ON_UNKNOWN = 10;
// We try at most this many times to read a CV if we keep getting an unknown
// railcom response. After that we report an error.
static const int READ_RETRY_COUNT_ON_UNKNOWN = 10;

// Maximum time we keep trying to read or write a given CV.
static const int RAILCOM_POM_OP_TIMEOUT_MSEC = 2000;


namespace openlcb
{

TractionCvSpace::TractionCvSpace(MemoryConfigHandler *parent,
                                 dcc::PacketFlowInterface *track,
                                 dcc::RailcomHubFlow *railcom_hub,
                                 uint8_t space_id)
    : StateFlowBase(parent->service())
    , parent_(parent)
    , track_(track)
    , railcomHub_(railcom_hub)
    , errorCode_(ERROR_NOOP)
    , spaceId_(space_id)
    , timer_(this)
{
    parent_->registry()->insert(nullptr, spaceId_, this);
    // We purposefully do not start the state flow until a request comes in.
}

TractionCvSpace::~TractionCvSpace()
{
    parent_->registry()->erase(nullptr, spaceId_, this);
    if (errorCode_ == ERROR_PENDING)
    {
        timer_.cancel();
    }
}

bool TractionCvSpace::set_node(Node *node)
{
    if (!node)
    {
        return false;
    }
    NodeID id = node->node_id();
    /* NOTE(balazs.racz): It is difficult to figure out the DCC address given
     * an abstract NMRAnet node. Here we hardcode the reserved DCC node ID
     * space, which is not a great solution. */
    if ((id & 0xFFFF00000000ULL) == TractionDefs::NODE_ID_DCC)
    {
        uint16_t new_address = id & 0xFFFFU;
        if (dccAddress_ != new_address)
        {
            dccAddress_ = new_address;
            errorCode_ = ERROR_NOOP;
        }
        return true;
    }
    else
    {
        return false;
    }
}

const unsigned TractionCvSpace::MAX_CV;

size_t TractionCvSpace::read(const address_t source, uint8_t *dst, size_t len,
    errorcode_t *error, Notifiable *again)
{
    if (source == OFFSET_CV_INDEX) {
        lastIndexedNode_ = dccAddress_;
        uint8_t* lastcv = (uint8_t*)&lastIndexedCv_;
        if (len > 0) dst[0] = lastcv[3];
        if (len > 1) dst[1] = lastcv[2];
        if (len > 2) dst[2] = lastcv[1];
        if (len > 3) dst[3] = lastcv[0];
        return std::min(len, size_t(4));
    }
    uint32_t cv = -1;
    if (source == OFFSET_CV_VALUE || source == OFFSET_CV_VERIFY_RESULT)
    {
        if (dccAddress_ != lastIndexedNode_)
        {
            *error = Defs::ERROR_PERMANENT;
            return 0;
        }
        // Translate from user-visible CV to wire protocol CV.
        cv = lastIndexedCv_ - 1;
        // fall through to regular processing
    }
    if (source < OFFSET_CV_INDEX)
    {
        cv = source;
    }
    LOG(INFO, "cv read %" PRIu32, cv);
    if (cv > MAX_CV)
    {
        *error = MemoryConfigDefs::ERROR_OUT_OF_BOUNDS;
        errorCode_ = ERROR_NOOP;
        return 0;
    }
    if (cv == cvNumber_)
    {
        if (errorCode_ == ERROR_OK)
        {
            *dst = cvData_;
            errorCode_ = ERROR_NOOP;
            return 1;
        }
        else if (errorCode_ == _ERROR_TIMEOUT)
        {
            *error = Defs::ERROR_OPENLCB_TIMEOUT;
            errorCode_ = ERROR_NOOP;
            return 0;
        }
        else if (errorCode_ == _ERROR_BUSY)
        {
            *error = Defs::ERROR_OUT_OF_ORDER;
            errorCode_ = ERROR_NOOP;
            return 0;
        }
    }
    done_ = again;
    cvNumber_ = cv;
    errorCode_ = ERROR_NOOP;
    cvData_ = 0;
    numTry_ = 0;
    if ((source == OFFSET_CV_VALUE) || (source <= MAX_CV))
    {
        start_flow(STATE(try_read1));
    }
    else if (source == OFFSET_CV_VERIFY_RESULT)
    {
        start_flow(STATE(pgm_verify));
    }
    else
    {
        DIE("Have not started the flow but will return AGAIN.");
    }
    *error = ERROR_AGAIN;
    deadline_ =
        os_get_time_monotonic() + MSEC_TO_NSEC(RAILCOM_POM_OP_TIMEOUT_MSEC);
    return 0;
}

StateFlowBase::Action TractionCvSpace::pgm_verify()
{
    return invoke_subflow_and_wait(
        Singleton<ProgrammingTrackBackend>::instance(),
        STATE(pgm_verify_wait_flush),
        ProgrammingTrackRequest::ENTER_SERVICE_MODE);
}

StateFlowBase::Action TractionCvSpace::pgm_verify_wait_flush()
{
    auto b = get_buffer_deleter(
        full_allocation_result(Singleton<ProgrammingTrackBackend>::instance()));
    if (b->data()->resultCode != 0)
    {
        // Failed to enter service mode. Maybe we are in ESTOP.
        errorCode_ = _ERROR_BUSY;
        done_->notify();
        return exit();
    }
    return sleep_and_call(&timer_, MSEC_TO_NSEC(10), STATE(pgm_verify_reset));
}

StateFlowBase::Action TractionCvSpace::pgm_verify_reset()
{
    return invoke_subflow_and_wait(
        Singleton<ProgrammingTrackBackend>::instance(),
        STATE(pgm_verify_packet), ProgrammingTrackRequest::SEND_RESET, 15);
}

StateFlowBase::Action TractionCvSpace::pgm_verify_packet()
{
    auto b = get_buffer_deleter(
        full_allocation_result(Singleton<ProgrammingTrackBackend>::instance()));

    dcc::Packet pkt;
    pkt.set_dcc_svc_verify_byte(lastIndexedCv_, lastVerifyValue_);
    pkt.set_dcc_svc_verify_bit(0, 7, 0);
    /*pkt.start_dcc_packet();
    pkt.payload[0] = 0b01110100 | ((lastIndexedCv_ >> 8) & 0b11);
    pkt.payload[1] = (lastIndexedCv_ & 0xff);
    pkt.payload[2] = lastVerifyValue_;
    pkt.dlc = 3;
    pkt.add_dcc_checksum();*/
    return invoke_subflow_and_wait(
        Singleton<ProgrammingTrackBackend>::instance(), STATE(pgm_verify_done),
        ProgrammingTrackRequest::SEND_PROGRAMMING_PACKET, pkt, 15);
}

StateFlowBase::Action TractionCvSpace::pgm_verify_done()
{
    auto b = get_buffer_deleter(
        full_allocation_result(Singleton<ProgrammingTrackBackend>::instance()));
    if (b->data()->hasAck_)
    {
        cvData_ = 1;
    }
    else
    {
        cvData_ = 0;
    }
    errorCode_ = ERROR_OK;
    return invoke_subflow_and_wait(
        Singleton<ProgrammingTrackBackend>::instance(),
        STATE(pgm_verify_reset_done), ProgrammingTrackRequest::SEND_RESET, 15);
}

StateFlowBase::Action TractionCvSpace::pgm_verify_reset_done()
{
    auto b = get_buffer_deleter(
        full_allocation_result(Singleton<ProgrammingTrackBackend>::instance()));
    return invoke_subflow_and_wait(
        Singleton<ProgrammingTrackBackend>::instance(), STATE(pgm_verify_exit),
        ProgrammingTrackRequest::EXIT_SERVICE_MODE);
}

StateFlowBase::Action TractionCvSpace::pgm_verify_exit()
{
    auto b = get_buffer_deleter(
        full_allocation_result(Singleton<ProgrammingTrackBackend>::instance()));
    return async_done();
}

StateFlowBase::Action TractionCvSpace::try_read1()
{
    return allocate_and_call(track_, STATE(fill_read1_packet));
}
StateFlowBase::Action TractionCvSpace::try_write1()
{
    return allocate_and_call(track_, STATE(fill_write1_packet));
}

StateFlowBase::Action TractionCvSpace::fill_read1_packet()
{
    auto *b = get_allocation_result(track_);
    b->data()->start_dcc_packet();
    /** @TODO(balazs.racz) here we make bad assumptions about how to decide
     * between long and short addresses */
    if (dccAddress_ >= 0x80)
    {
        b->data()->add_dcc_address(dcc::DccLongAddress(dccAddress_));
    }
    else
    {
        b->data()->add_dcc_address(dcc::DccShortAddress(dccAddress_));
    }
    b->data()->add_dcc_pom_read1(cvNumber_);
    b->data()->feedback_key = reinterpret_cast<uintptr_t>(this);
    railcomHub_->register_port(this);
    errorCode_ = ERROR_PENDING;
    track_->send(b);
    return sleep_and_call(&timer_, MSEC_TO_NSEC(500), STATE(read_returned));
}

StateFlowBase::Action TractionCvSpace::read_returned()
{
    LOG(WARNING, "railcom read returned status %d value %d", errorCode_, cvData_);
    switch (errorCode_) {
    case ERROR_PENDING:
        errorCode_ = _ERROR_TIMEOUT;
        railcomHub_->unregister_port(this);
        break;
    case ERROR_OK:
        break;
    default:
    case ERROR_UNKNOWN_RESPONSE:
        if (numTry_ >= READ_RETRY_COUNT_ON_UNKNOWN) {
            errorCode_ = _ERROR_TIMEOUT;
            break;
        }
        numTry_++;
        return call_immediately(STATE(try_read1));
    case _ERROR_BUSY:
        if (os_get_time_monotonic() > deadline_) {
            errorCode_ = _ERROR_TIMEOUT;
            break;
        }
        return call_immediately(STATE(try_read1));
    }
    return async_done();
}

size_t TractionCvSpace::write(address_t destination, const uint8_t *src,
                              size_t len, errorcode_t *error, Notifiable *again)
{
    if (destination == OFFSET_CV_INDEX) {
        lastIndexedNode_ = dccAddress_;
        uint8_t* lastcv = (uint8_t*)&lastIndexedCv_;
        if (len > 0) lastcv[3] = src[0];
        if (len > 1) lastcv[2] = src[1];
        if (len > 2) lastcv[1] = src[2];
        if (len > 3) lastcv[0] = src[3];
        return std::min(len, size_t(4));
    }
    if (destination == OFFSET_CV_VERIFY_VALUE)
    {
        lastVerifyValue_ = src[0];
        return 1;
    }
    if (destination == OFFSET_CV_VALUE) {
        if (dccAddress_ != lastIndexedNode_) {
            *error = Defs::ERROR_TEMPORARY;
            return 0;
        }
        // Translate from user-visible CV to wire protocol CV.
        destination = lastIndexedCv_ - 1;
        // fall through to regular processing
    }
    LOG(INFO, "cv write %" PRIu32 " := %d", destination, *src);
    if (destination > MAX_CV)
    {
        *error = MemoryConfigDefs::ERROR_OUT_OF_BOUNDS;
        errorCode_ = ERROR_NOOP;
        return 0;
    }
    if (errorCode_ == ERROR_OK && destination == cvNumber_)
    {
        errorCode_ = ERROR_NOOP;
        return 1;
    }
    if (errorCode_ == _ERROR_TIMEOUT && destination == cvNumber_)
    {
        *error = Defs::ERROR_TEMPORARY | 1;
        errorCode_ = ERROR_NOOP;
        return 0;
    }
    done_ = again;
    cvNumber_ = destination;
    cvData_ = *src;
    numTry_ = 0;
    errorCode_ = ERROR_NOOP;
    start_flow(STATE(try_write1));
    *error = ERROR_AGAIN;
    deadline_ = os_get_time_monotonic() + MSEC_TO_NSEC(RAILCOM_POM_OP_TIMEOUT_MSEC);
    return 0;
}

StateFlowBase::Action TractionCvSpace::fill_write1_packet()
{
    auto *b = get_allocation_result(track_);
    b->data()->start_dcc_packet();
    /** @todo(balazs.racz) here we make bad assumptions about how to decide
     * between long and short addresses */
    if (dccAddress_ >= 0x80)
    {
        b->data()->add_dcc_address(dcc::DccLongAddress(dccAddress_));
    }
    else
    {
        b->data()->add_dcc_address(dcc::DccShortAddress(dccAddress_));
    }
    b->data()->add_dcc_pom_write1(cvNumber_, cvData_);
    b->data()->feedback_key = reinterpret_cast<uintptr_t>(this);
    // POM write packets need to appear at least twice on non-back-to-back
    // packets by the standard. We make 4 back to back packets and that
    // fulfills the requirement.
    b->data()->packet_header.rept_count = 3;
    railcomHub_->register_port(this);
    errorCode_ = ERROR_PENDING;
    track_->send(b);
    return sleep_and_call(&timer_, MSEC_TO_NSEC(500), STATE(write_returned));
}

StateFlowBase::Action TractionCvSpace::write_returned()
{
    LOG(WARNING, "railcom write returned status %d value %d", errorCode_, cvData_);
    switch (errorCode_)
    {
        case ERROR_PENDING:
            errorCode_ = _ERROR_TIMEOUT;
            railcomHub_->unregister_port(this);
            break;
        case ERROR_OK:
            break;
        default:
        case ERROR_UNKNOWN_RESPONSE:
            if (numTry_ >= WRITE_RETRY_COUNT_ON_UNKNOWN)
            {
                // We switch from writing to reading if we had tried many enough
                // times. Maybe the write was actually successful.
                return call_immediately(STATE(try_read1));
            }
            numTry_++;
            return call_immediately(STATE(try_write1));
        case _ERROR_BUSY:
            if (os_get_time_monotonic() > deadline_)
            {
                errorCode_ = _ERROR_TIMEOUT;
                break;
            }
            /// @todo(balazs.racz) keep a timestamp to not keep trying forever.
            return call_immediately(STATE(try_write1));
    }
    return async_done();
}

void TractionCvSpace::record_railcom_status(unsigned code)
{
    errorCode_ = code;
    timer_.trigger();
    railcomHub_->unregister_port(this);
}

void TractionCvSpace::send(Buffer<dcc::RailcomHubData> *b, unsigned priority)
{
    AutoReleaseBuffer<dcc::RailcomHubData> ar(b);
    if (errorCode_ != ERROR_PENDING)
        return;
    const dcc::Feedback &f = *b->data();
    if (f.feedbackKey != (reinterpret_cast<uintptr_t>(this)) || f.channel == 0xff)
    {
        // Skip railcom from other packets; also skip the railcom-based
        // occupancy information packets.
        return;
    }
    LOG(INFO, "CV railcom feedback ch=%d: %s", f.channel, railcom_debug(f).c_str());
    if (!f.ch2Size)
    {
        return record_railcom_status(ERROR_NO_RAILCOM_CH2_DATA);
    }
    dcc::parse_railcom_data(f, &interpretedResponse_);
    unsigned new_status = ERROR_PENDING;
    for (const auto& e : interpretedResponse_) {
        if (e.railcom_channel != 2) continue;
        switch(e.type) {
        case dcc::RailcomPacket::BUSY:
            if (new_status == ERROR_PENDING) {
                new_status = _ERROR_BUSY;
            }
            break;
        case dcc::RailcomPacket::NACK:
            if (new_status == ERROR_PENDING) {
                // We ignore NACK responses because certain versions ofthe
                // standard had NACK interpreted the same way as ACK as well as
                // it was being used as a filler. There are locomotives out
                // there who do this.
                // new_status = ERROR_NACK;
            }
            break;
        case dcc::RailcomPacket::ACK:
            if (new_status == ERROR_PENDING) {
                new_status = ERROR_OK;
            }
            break;
        case dcc::RailcomPacket::GARBAGE:
            if (new_status == ERROR_PENDING) {
                new_status = ERROR_GARBAGE;
            }
            break;
        case dcc::RailcomPacket::MOB_POM:
            cvData_ = e.argument;
            new_status = ERROR_OK;
            break;
        default:
            if (new_status == ERROR_PENDING) {
                new_status = ERROR_UNKNOWN_RESPONSE;
            }
            break;
        }
    }
    return record_railcom_status(new_status);
}

} // namespace openlcb
