/** \copyright
 * Copyright (c) 2014, Balazs Racz
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
 * \file async_datagram_test_helper.hxx
 *
 * Unit tests fixture classes for exercising datagram functionality.
 *
 * @author Balazs Racz
 * @date 23 Feb 2014
 */

#ifndef _UTILS_ASYNC_DATAGRAM_TEST_HELPER_HXX_
#define _UTILS_ASYNC_DATAGRAM_TEST_HELPER_HXX_

#include "utils/async_if_test_helper.hxx"
#include "openlcb/Datagram.hxx"
#include "openlcb/DatagramCan.hxx"

namespace openlcb {

/// Test base class for OpenLCB unittests that need a datagram handler.
class AsyncDatagramTest : public AsyncNodeTest
{
protected:
    AsyncDatagramTest() : datagram_support_(ifCan_.get(), 10, 2)
    {
    }

    CanDatagramService datagram_support_;
};

Pool* const g_incoming_datagram_allocator = mainBufferPool;

/// Test base class for OpenLCB unittests that simulate two physical or virtual
/// nodes talking to each other with the canbus protocol.
///
/// The two modes of operation are as follows:
///
/// 1) The test base class creates one interface on the canbus, and adds two
/// virtual nodes to this interface. These virtual nodes will be able to send
/// datagrams to each other via local loopback without generating any CANbus
/// traffic visible outside. This mode does not test the CAN frame
/// fragmentation and parsing code.
///
/// 2) the test base class creates two independent CAN interface objects that
/// will be both wired to the same CAN hub (aka virtual CAN bus). The two
/// interfaces will each have one virtual node (but separate alias allocation,
/// local and remote alias cache and local nodes map structure). Talking
/// between these nodes will cause the datagram to be fragmented and sent onto
/// the CAN bus. Using this mode will require tests to make expectations on the
/// canbus traffic, or ignore all canbus packets from the test correctness
/// perspective.
class TwoNodeDatagramTest : public AsyncDatagramTest
{
protected:
    enum
    {
        OTHER_NODE_ID = TEST_NODE_ID + 0x100,
        OTHER_NODE_ALIAS = 0x225,
    };

    /// @param separate_if defines which mode the test base should operate
    /// in. false = mode 1 (one interface, two virtual nodes); true = mode 2
    /// (two interfaces).
    void setup_other_node(bool separate_if)
    {
        if (separate_if)
        {
            otherIfCan_.reset(
                new IfCan(&g_executor, &can_hub0, 10, 10, 5));
            otherIfCan_->add_addressed_message_support();
            otherNodeIf_ = otherIfCan_.get();
            otherDatagramSupport_.reset(
                new CanDatagramService(otherNodeIf_, 10, 2));
            otherNodeDatagram_ = otherDatagramSupport_.get();
        }
        else
        {
            otherNodeIf_ = ifCan_.get();
            otherNodeDatagram_ = &datagram_support_;
        }
        run_x([this]() {
            otherNodeIf_->local_aliases()->add(OTHER_NODE_ID, OTHER_NODE_ALIAS);
        });
        expect_packet(":X19100225N02010D000103;"); // node up
        otherNode_.reset(new DefaultNode(otherNodeIf_, OTHER_NODE_ID));
        wait();
    }

    /// Adds the necessary expectations for the address lookup reuqests on the
    /// CANbus by the first datagram being sent from node_ to otherNode_. Not
    /// needed for mode 1 operation.
    void expect_other_node_lookup()
    {
        expect_packet(":X1070222AN02010D000103;"); // looking for DST node
        expect_packet(":X10701225N02010D000103;"); // found dst node
        // expect_packet(":X1949022AN02010D000103;"); // hard-looking for DST node
        // expect_packet(":X19170225N02010D000103;"); // node ID verified
    }

    std::unique_ptr<DefaultNode> otherNode_;
    // Second objects if we want a bus-traffic test.
    std::unique_ptr<IfCan> otherIfCan_;
    IfCan* otherNodeIf_;
    std::unique_ptr<CanDatagramService> otherDatagramSupport_;
    CanDatagramService* otherNodeDatagram_;
};

} // namespace


#endif // _UTILS_ASYNC_DATAGRAM_TEST_HELPER_HXX_
