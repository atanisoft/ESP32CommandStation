// Helper classes for writing unittests testing the entire asynchronous
// stack. Allows to send incoming messages (in gridconnect format) and set
// expectations on messages produced.
//
// Only include this file in unittests.

#ifndef _UTILS_ASYNC_IF_TEST_HELPER_HXX_
#define _UTILS_ASYNC_IF_TEST_HELPER_HXX_

#include "openlcb/AliasAllocator.hxx"
#include "openlcb/IfCan.hxx"
#include "openlcb/EventService.hxx"
#include "openlcb/DefaultNode.hxx"
#include "openlcb/NodeInitializeFlow.hxx"
#include "executor/CallableFlow.hxx"
#include "nmranet_config.h"
#include "utils/GridConnectHub.hxx"
#include "utils/test_main.hxx"

using ::testing::AtLeast;
using ::testing::AtMost;
using ::testing::Eq;
using ::testing::Field;
using ::testing::Invoke;
using ::testing::IsNull;
using ::testing::Mock;
using ::testing::NiceMock;
using ::testing::NotNull;
using ::testing::Pointee;
using ::testing::Return;
using ::testing::StrCaseEq;
using ::testing::StrictMock;
using ::testing::WithArg;
using ::testing::SaveArg;
using ::testing::_;

class TrainTestHelper;

///@todo(balazs.racz) remove
// void (*g_invoke)(Notifiable *) = &InvokeNotification;

HubFlow gc_hub0(&g_service);
CanHubFlow can_hub0(&g_service);
GCAdapterBase *g_gc_adapter = nullptr;

HubFlow gc_hub1(&g_service);
CanHubFlow can_hub1(&g_service);
GCAdapterBase *g_gc_adapter1 = nullptr;

openlcb::InitializeFlow g_init_flow(&g_service);

/** Helper class for setting expectation on the CANbus traffic in unit
 * tests. */
class MockSend : public HubPort
{
public:
    MockSend()
        : HubPort(&g_service)
    {
    }

    MOCK_METHOD1(mwrite, void(const string &s));

    virtual Action entry()
    {
        string s(message()->data()->data(), message()->data()->size());
        mwrite(s);
        return release_and_exit();
    }
};

void InvokeNotification(Notifiable *done)
{
    done->notify();
}

static void print_packet(const string &pkt)
{
    fprintf(stderr, "%s\n", pkt.c_str());
}

/// Test fixture base class for tests that need a CAN-based interface(but not
/// necessary specific to OpenLCB). The direct use of this class is useful only
/// when other CAN-based protocol flows need to be tested. The OpenLCB test
/// fixtures are inheriting from this class.
class AsyncCanTest : public testing::Test
{
public:
    /// Initializes test case with CAN0. Note: if a child test implements
    /// SetUpTestCase themself, they must ensure to call this function.
    static void SetUpTestCase()
    {
        g_gc_adapter =
            GCAdapterBase::CreateGridConnectAdapter(&gc_hub0, &can_hub0, false);
    }

    /// De-Initializes test case with CAN0. Note: if a child test implements
    /// TearDownTestCase themself, they must ensure to call this function.
    static void TearDownTestCase()
    {
        delete g_gc_adapter;
    }

protected:
    AsyncCanTest()
    {
        gc_hub0.register_port(&canBus_);
    }

    ~AsyncCanTest()
    {
        gc_hub0.unregister_port(&canBus_);
        if (printer_.get())
        {
            gc_hub0.unregister_port(printer_.get());
        }
    }

    /** Delays the current thread until we are certain that all asynchrnous
        processing has completed. */
    void wait()
    {
        wait_for_main_executor();
    }

    /** Delays the current thread until all asynchronous processing and all
     * pending timers have completed. */
    void twait()
    {
        wait_for_main_executor();
        while (!g_executor.active_timers()->empty())
        {
            usleep(20000);
            wait_for_main_executor();
        }
        wait_for_main_executor();
    }

#ifdef __EMSCRIPTEN__
    void usleep(unsigned long usecs) {
        long long deadline = usecs;
        deadline *= 1000;
        deadline += os_get_time_monotonic();
        while (os_get_time_monotonic() < deadline) {
            os_emscripten_yield();
        }
    }
#endif

/** Adds an expectation that the code will send a packet to the CANbus.

    Example:
    expect_packet(":X1954412DN05010101FFFF0000;");

    @param gc_packet the packet in GridConnect format, including the leading
    : and trailing ;
*/
#define expect_packet(gc_packet)                                               \
    EXPECT_CALL(canBus_, mwrite(StrCaseEq(gc_packet)))

    /** Ignores all produced packets.
     *
     *  Tihs can be used in tests where the expectations are tested in a higher
     *  level than monitoring the CANbus traffic.
    */
    void expect_any_packet()
    {
        if (!printer_) { print_all_packets(); }
        EXPECT_CALL(canBus_, mwrite(_)).Times(AtLeast(0));
        //.WillRepeatedly(WithArg<0>(Invoke(print_packet)));
    }

    /** Prints all packets sent to the canbus until the end of the current test
     * function.
    */
    void print_all_packets()
    {
        HASSERT(!printer_ && "cannot have more than one print_all_packets call");
        NiceMock<MockSend> *m = new NiceMock<MockSend>();
        EXPECT_CALL(*m, mwrite(_)).Times(AtLeast(0)).WillRepeatedly(
            WithArg<0>(Invoke(print_packet)));
        gc_hub0.register_port(m);
        printer_.reset(m);
    }

    /** Injects a packet to the interface. This acts as if a different node on
        the CANbus had sent that packet.

        Example:
        send_packet(":X195B4001N05010101FFFF0000;");

        @param gc_packet the packet in GridConnect format, including the leading
        : and trailing ;
    */
    void send_packet(const string &gc_packet)
    {
        Buffer<HubData> *packet;
        mainBufferPool->alloc(&packet);
        packet->data()->assign(gc_packet);
        packet->data()->skipMember_ = &canBus_;
        gc_hub0.send(packet);
    }

    /// Clears all existing expectations on the CAN-bus packets. Usually means
    /// that all previously expressed expectations must have been met by this
    /// point.
    ///
    /// @param strict if true, does not allow any unknown/unexpected/spurious
    /// packets to have arrived; i.e., fails the test if there was a packet
    /// coming with no expectation.
    ///
    void clear_expect(bool strict = false)
    {
        Mock::VerifyAndClear(&canBus_);
        if (strict) {
            EXPECT_CALL(canBus_, mwrite(_)).Times(0);
        }
    }

/** Injects an incoming packet to the interface and expects that the node
    will send out a response packet for it.

    As a side effect, clears all pending expectations on the CANbus.

    Example:
    send_packet_and_expect_response(":X198F4001N05010101FFFF0000;",
                                    ":X194C412DN05010101FFFF0000;");

    @param pkt is the packet to inject, in GridConnect format.
    @param resp is the response to expect, also in GridConnect format.
*/
#define send_packet_and_expect_response(pkt, resp)                             \
    do                                                                         \
    {                                                                          \
        expect_packet(resp);                                                   \
        send_packet_and_flush_expect(pkt);                                     \
    } while (0)

    /// Sends a packet to the canbus, waits for the executor to clear, and then
    /// verifies all previous expectations.
    ///
    /// @param pkt gridconnnect-formatted packet to send.
    ///
    void send_packet_and_flush_expect(const string &pkt)
    {
        send_packet(pkt);
        wait();
        Mock::VerifyAndClear(&canBus_);
    }

    /// Helper object for setting expectations on the packets sent on the bus.
    NiceMock<MockSend> canBus_;
    /// Object for debug-printing every packet (if requested).
    std::unique_ptr<HubPort> printer_;
};

/// Test fixture base class for a second CAN-bus. Usage: use multiple
/// inheritance in the test fixture to merge both AsyncCanTest and
/// AsyncCan1Test.
class AsyncCan1Test {
protected:
  AsyncCan1Test() { gc_hub1.register_port(&canBus1_); }
  ~AsyncCan1Test() {
    wait_for_main_executor();
    gc_hub1.unregister_port(&canBus1_);
  }
  
    /// Initializes test case with CAN1. Note: if a child test implements
    /// SetUpTestCase themself, they must ensure to call this function.
  static void SetUpTestCase() {
    g_gc_adapter1 =
        GCAdapterBase::CreateGridConnectAdapter(&gc_hub1, &can_hub1, false);
  }

    /// De-Initializes test case with CAN1. Note: if a child test implements
    /// TearDownTestCase themself, they must ensure to call this function.
  static void TearDownTestCase() {
    delete g_gc_adapter1;
  }

#define expect_packet1(gc_packet) \
  EXPECT_CALL(canBus1_, mwrite(StrCaseEq(gc_packet)))

  /// Send a gridconnect packet to the second CAN port.
  ///
  /// @param gc_packet gridconnect formatted packet string.
  ///
  void send_packet1(const string &gc_packet) {
    Buffer<HubData> *packet;
    mainBufferPool->alloc(&packet);
    packet->data()->assign(gc_packet);
    packet->data()->skipMember_ = &canBus1_;
    gc_hub1.send(packet);
  }

  /// Helper object for setting expectations on the packets sent on the bus.
  StrictMock<MockSend> canBus1_;
};

namespace openlcb
{

static const NodeID TEST_NODE_ID = 0x02010d000003ULL;

class LocalIf : public If
{
public:
    LocalIf(int local_nodes_count)
        : If(&g_executor, local_nodes_count)
    {
    }

    void add_owned_flow(Executable *e) override
    {
        ownedFlows_.emplace_back(e);
    }

    void delete_local_node(Node *node) override
    {
        remove_local_node_from_map(node);
    }

    bool matching_node(NodeHandle expected, NodeHandle actual) override
    {
        if (expected.id && actual.id)
        {
            return expected.id == actual.id;
        }
        // Cannot reconcile.
        LOG(VERBOSE, "Cannot reconcile expected and actual NodeHandles for "
                     "equality testing.");
        return false;
    }

private:
    std::vector<std::unique_ptr<Destructable>> ownedFlows_;
};

/** Test fixture base class with helper methods for exercising the asynchronous
 * interface code.
 *
 * Usage:
 *
 * Inherit your test fixture class from AsyncIfTest.
 */
class AsyncIfTest : public AsyncCanTest
{
protected:
    static int local_alias_cache_size;
    static int local_node_count;
    static int remote_alias_cache_size;

    AsyncIfTest()
        : pendingAliasAllocation_(false)
    {
        ifCan_.reset(new IfCan(&g_executor, &can_hub0, local_alias_cache_size, remote_alias_cache_size, local_node_count));
        run_x([this]() { ifCan_->local_aliases()->add(TEST_NODE_ID, 0x22A); });
        ifCan_->set_alias_allocator(
            new AliasAllocator(TEST_NODE_ID, ifCan_.get()));

    }

    ~AsyncIfTest()
    {
        wait();
        if (pendingAliasAllocation_)
        {
            ifCan_->alias_allocator()->TEST_finish_pending_allocation();
            wait();
        }
    }

    friend class ::TrainTestHelper;

    /** Creates an alias allocator flow, and injects an already allocated
     *  alias. */
    void create_allocated_alias()
    {
        inject_allocated_alias(0x33A, true);
        aliasSeed_ = 0x44C;
        pendingAliasAllocation_ = false;
    }

    void inject_allocated_alias(NodeAlias alias, bool repeat = false)
    {
        if (!ifCan_->alias_allocator()) {
            ifCan_->set_alias_allocator(
                new AliasAllocator(TEST_NODE_ID, ifCan_.get()));
        }
        run_x([this, alias, repeat]() {
            ifCan_->alias_allocator()->TEST_add_allocated_alias(alias, repeat);
        });
    }

    void expect_next_alias_allocation(NodeAlias a = 0)
    {
        pendingAliasAllocation_ = true;
        if (!a)
        {
            ifCan_->alias_allocator()->seed_ = aliasSeed_;
            a = aliasSeed_;
            aliasSeed_++;
        }
        EXPECT_CALL(canBus_, mwrite(StringPrintf(":X17020%03XN;", a)))
            .Times(1)
            .RetiresOnSaturation();
        EXPECT_CALL(canBus_, mwrite(StringPrintf(":X1610D%03XN;", a)))
            .Times(1)
            .RetiresOnSaturation();
        EXPECT_CALL(canBus_, mwrite(StringPrintf(":X15000%03XN;", a)))
            .Times(1)
            .RetiresOnSaturation();
        EXPECT_CALL(canBus_, mwrite(StringPrintf(":X14003%03XN;", a)))
            .Times(1)
            .RetiresOnSaturation();

        EXPECT_CALL(canBus_, mwrite(StringPrintf(":X10700%03XN;", a)))
            .Times(AtMost(1))
            .RetiresOnSaturation();
    }


    BarrierNotifiable *get_notifiable()
    {
        bn_.reset(&n_);
        return &bn_;
    }

    void wait_for_notification()
    {
        n_.wait_for_notification();
    }

    SyncNotifiable n_;
    BarrierNotifiable bn_;

    /// The interface under test.
    std::unique_ptr<IfCan> ifCan_;
    /** Temporary object used to send aliases around in the alias allocator
     *  flow. */
    AliasInfo testAlias_;
    /// The next alias we will make the allocator create.
    NodeAlias aliasSeed_;
    /// true if we have a pending async alias allocation task.
    bool pendingAliasAllocation_;
};

int AsyncIfTest::local_alias_cache_size = 10;
int AsyncIfTest::local_node_count = 9;
int AsyncIfTest::remote_alias_cache_size = 10;

/// Base class for test cases with one virtual node on a CANbus interface.
class AsyncNodeTest : public AsyncIfTest
{
protected:
    AsyncNodeTest()
        : eventService_(ifCan_.get())
    {
        EXPECT_CALL(canBus_, mwrite(":X1910022AN02010D000003;")).Times(1);
        ownedNode_.reset(new DefaultNode(ifCan_.get(), TEST_NODE_ID));
        node_ = ownedNode_.get();
        ifCan_->add_addressed_message_support();
        wait();
        Mock::VerifyAndClear(&canBus_);
        // AddEventHandlerToIf(ifCan_.get());
    }

    ~AsyncNodeTest()
    {
        wait_for_event_thread();
    }

    void wait_for_event_thread()
    {
        while (EventService::instance->event_processing_pending())
        {
#ifdef __EMSCRIPTEN__
            os_emscripten_yield();
#else
            usleep(100);
#endif
        }
        AsyncIfTest::wait();
    }

    EventService eventService_;
    std::unique_ptr<DefaultNode> ownedNode_;
    Node *node_;
};

/// Test handler for receiving incoming openlcb Message objects from a bus. The
/// incoming messages need GoogleMock expectations.
///
/// Usage: see file src/openlcb/IfCan.cxxtest
class MockMessageHandler : public MessageHandler
{
public:
    MOCK_METHOD2(handle_message,
                 void(GenMessage *message, unsigned priority));
    virtual void send(Buffer<GenMessage> *message, unsigned priority)
    {
        handle_message(message->data(), priority);
        message->unref();
    }
};

/// GoogleMock matcher on a Payload being equal to a given 64-bit value in
/// network byte order. (Typically an event id.)
MATCHER_P(IsBufferValue, id, "")
{
    uint64_t value = htobe64(id);
    if (arg.size() != 8)
        return false;
    if (memcmp(&value, arg.data(), 8))
        return false;
    return true;
}

/// GoogleMock matcher on a Payload being equal to a given string (C or C++
/// style).
MATCHER_P(IsBufferValueString, expected, "")
{
    string s(expected);
    return arg == s;
}

/// GoogleMock matcher on a Payload being equal to a given 48-bit value in
/// network byte order (typically a Node ID).
MATCHER_P(IsBufferNodeValue, id, "")
{
    uint64_t value = htobe64(id);
    if (arg->used() != 6)
        return false;
    uint8_t *expected = reinterpret_cast<uint8_t *>(&value) + 2;
    uint8_t *actual = static_cast<uint8_t *>(arg->start());
    if (memcmp(expected, actual, 6))
    {
        for (int i = 0; i < 6; ++i)
        {
            if (expected[i] != actual[i])
            {
                LOG(INFO, "mismatch at position %d, expected %02x actual %02x",
                    i, expected[i], actual[i]);
            }
        }
        return false;
    }
    return true;
}

/// GoogleMock matcher on a Payload being equal to a given 6-byte string.
MATCHER_P(IsBufferNodeValueString, id, "")
{
    uint64_t value = htobe64(id);
    if (arg.size() != 6)
        return false;
    uint8_t *expected = reinterpret_cast<uint8_t *>(&value) + 2;
    uint8_t *actual = static_cast<uint8_t *>(arg->start());
    if (memcmp(expected, actual, 6))
    {
        for (int i = 0; i < 6; ++i)
        {
            if (expected[i] != actual[i])
            {
                LOG(INFO, "mismatch at position %d, expected %02x actual %02x",
                    i, expected[i], actual[i]);
            }
        }
        return false;
    }
    return true;
}

} // namespace openlcb

#endif // _UTILS_ASYNC_IF_TEST_HELPER_HXX_
