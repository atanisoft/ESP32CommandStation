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

#ifndef _OPENLCB_TRACTIONCVSPACE_HXX_
#define _OPENLCB_TRACTIONCVSPACE_HXX_

#include "openlcb/MemoryConfig.hxx"
#include "executor/StateFlow.hxx"
#include "dcc/PacketFlowInterface.hxx"
#include "dcc/RailCom.hxx"
#include "dcc/RailcomHub.hxx"

namespace openlcb
{

/// Memory Space implementation for a CV (configuration variable) access on a
/// Railcom-enabled command station. Reads are turned into DCC extended packets
/// for Programming-on-Main mode CV reads, and the Railcom feedback is
/// evaluated for the result of the memory read operation. Writes are turned
/// into POM-mode CV write packets, and the Railcom feedback is evaluated for
/// success acknowledgement.
///
/// A single instance of this class works for all DCC locomotives, assuming
/// that the memory configuration handler was registered for all virtual nodes
/// of the given interface.
///
/// Restrictions: the DCC locmotives are required to have theid NodeID
/// allocated as TractionDefs::NODE_ID_DCC + address. There is a hard-coded
/// assumption that node IDs lower, than 128 are shoprt addresses, and higher
/// addresses are long addresses.
///
/// @TODO(balazs.racz) Wire up a link to the traction service to somehow remove
/// these restrictions.
class TractionCvSpace : private MemorySpace,
                        private dcc::RailcomHubPortInterface,
                        public StateFlowBase
{
public:
    TractionCvSpace(MemoryConfigHandler *parent,
                    dcc::PacketFlowInterface *track,
                    dcc::RailcomHubFlow *railcom_hub, uint8_t space_id);

    ~TractionCvSpace();

private:
    static const unsigned MAX_CV = 1023;

    bool set_node(Node *node) OVERRIDE;

    bool read_only() OVERRIDE
    {
        return false;
    }

    address_t max_address() OVERRIDE
    {
        return OFFSET_CV_VALUE;
    }

    size_t write(address_t destination, const uint8_t *data, size_t len,
                 errorcode_t *error, Notifiable *again) OVERRIDE;

    size_t read(address_t source, uint8_t *dst, size_t len, errorcode_t *error,
                Notifiable *again) OVERRIDE;

    // State flow states.
    Action try_read1();
    Action fill_read1_packet();
    Action read_returned();

    Action try_write1();
    Action fill_write1_packet();
    Action write_returned();

    Action pgm_verify();
    Action pgm_verify_wait_flush();
    Action pgm_verify_reset();
    Action pgm_verify_packet();
    Action pgm_verify_done();
    Action pgm_verify_reset_done();
    Action pgm_verify_exit();

    // Railcom feedback
    void send(Buffer<dcc::RailcomHubData> *b, unsigned priority) OVERRIDE;
    void record_railcom_status(unsigned code);

    MemoryConfigHandler *parent_;
    dcc::PacketFlowInterface *track_;
    dcc::RailcomHubFlow *railcomHub_;
    uint16_t dccAddress_;
    uint16_t cvNumber_;
    uint8_t cvData_; //< data to read or write.
    uint8_t errorCode_ : 4;
    uint8_t numTry_ : 4;
    enum
    {
        ERROR_NOOP = 0,
        ERROR_PENDING = 1,
        ERROR_OK = 2,
        _ERROR_BUSY = 3,
        ERROR_NACK = 7,
        ERROR_NO_RAILCOM_CH2_DATA = 4,
        ERROR_GARBAGE = 5,
        ERROR_UNKNOWN_RESPONSE = 6,
        _ERROR_TIMEOUT = 8,
    };

public:
    enum
    {
        OFFSET_CV_INDEX = 0x7F000000,
        OFFSET_CV_VALUE = 0x7F000004,
        OFFSET_CV_VERIFY_VALUE = 0x7F000005,
        OFFSET_CV_VERIFY_RESULT = 0x7F000006,
    };

private:
    /// Helper function for completing asynchronous processing.
    Action async_done()
    {
        done_->notify();
        done_ = nullptr;
        return exit();
    }

    uint8_t spaceId_;
    /// Stores the last node for which the CV index was written.
    uint16_t lastIndexedNode_;
    /// Stores the CV value that we're checking the CV against to verify.
    uint8_t lastVerifyValue_;
    /// Stores the last CV index (for indirect CV lookup).
    uint32_t lastIndexedCv_;

    Notifiable *done_; //< notify when transfer is done
    StateFlowTimer timer_;
    long long deadline_;  //< time when we should give up and return error.
    vector<dcc::RailcomPacket> interpretedResponse_;
};

} // namespace openlcb

#endif // _OPENLCB_TRACTIONCVSPACE_HXX_
