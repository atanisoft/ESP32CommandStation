/** @copyright
 * Copyright (c) 2017, Stuart W Baker
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
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
 * @file DccAccyProducer.hxx
 *
 * Producer class that represents 2044 consecutive bits out of DCC accessory
 * control Well-Known Event ID space.  This is inteded for use in throttle
 * implementations.
 *
 * @author Stuart Baker
 * @date 17 June 2017
 */

#ifndef _OPENLCB_DCCACCYPRODUCER_HXX_
#define _OPENLCB_DCCACCYPRODUCER_HXX_

#include <vector>

#include "executor/CallableFlow.hxx"
#include "openlcb/NonAuthoritativeEventProducer.hxx"
#include "openlcb/TractionDefs.hxx"
#include "utils/Uninitialized.hxx"

namespace openlcb
{
/// C++ Namespace for collecting all commands that can be sent to the
/// DccAccyProducer flow.
struct DccAccyProducerCommands
{
    /// QUERY command.
    enum Query
    {
        QUERY, ///< QUERY command
    };

    /// SET command.
    enum Set
    {
        SET, ///< SET command
    };
};

/// Request structure used to send requests to the DccAccyProducer
/// class. Contains parametrized reset calls for properly supporting
/// @ref StateFlowBase::invoke_subflow_and_wait() syntax.
struct DccAccyProducerInput : public CallableFlowRequestBase
{
    /// Possible subflow commands
    enum Command
    {
        CMD_QUERY, ///< state query
        CMD_SET, ///< state set/change
    };

    /// Reset method for @ref CMD_QUERY.
    /// @param address DCC accessory decoder address
    void reset(const DccAccyProducerCommands::Query &,
               const uint16_t address)
    {
        cmd_ = CMD_QUERY;
        address_ = address;
    }

    /// Reset method for @ref CMD_SET.
    /// @param address DCC accessory decoder address
    /// @param value desired value to command
    void reset(const DccAccyProducerCommands::Set &, const uint16_t address,
               bool value)
    {
        cmd_ = CMD_SET;
        address_ = address;
        value_ = value;
    }

    Command cmd_; ///< subflow command
    uint16_t address_; ///< DCC accessory address
    bool value_; ///< DCC accessory address value
};

/// DCC accessory address event producer for the Well-Known DCC Accessory range.
/// We implement the event mapping scheme referred to as "Vendor B" in the
/// OpenLCB EventIdentifiersTN document.
class DccAccyProducer : public CallableFlow<DccAccyProducerInput>
{
public:
    /// DCC accessory activation values
    enum Route
    {
        NORMAL = false, ///< normal route
        REVERSE = true, ///< reverse route
    };

    /// highest possible DCC address supported
    static constexpr uint16_t MAX_ADDRESS = 2044;

    /// Constructor.  Creates a new DCC Accessory range producer.
    ///
    /// @param node the node that the producer will be bound to
    /// @param dcc_state_callback Callback method for delivering the results of
    ///                           a consumer identified or event report.  The
    ///                           first unsigned parameter represents the bit
    ///                           offset for the range and the second bool
    ///                           parameter indicates the state as indicated by
    ///                           @ref Route.
    DccAccyProducer(Node *node,
               std::function<void(unsigned, bool)> dcc_state_callback = nullptr)
        : CallableFlow<DccAccyProducerInput>(node->iface())
        , dccStateCallback_(dcc_state_callback)
        , writer_()
        , bn_()
    {
        once_.once();
        AtomicHolder h(&instancesLock_);
        if (instances_->empty())
        {
            uint32_t max_address = MAX_ADDRESS;
            eventProducer_.emplace(
                      node,
                      TractionDefs::ACTIVATE_BASIC_DCC_ACCESSORY_EVENT_BASE + 8,
                      max_address, state_callback);
        }
        instances_->push_back(this);
    }

    /// Destructor.  Remove "this" instance from @ref instances_ vector
    ~DccAccyProducer()
    {
        AtomicHolder h(&instancesLock_);
        for (unsigned i = 0; i < instances_->size(); ++i)
        {
            if (instances_->at(i) == this)
            {
                instances_->erase(instances_->begin() + i);
                break;
            }
        }
        if (instances_->empty())
        {
            eventProducer_.reset();
        }
    }

private:
    using Command = DccAccyProducerInput::Command;

    /** Called once.
     */
    static void once_routine()
    {
        instances_.emplace();
    }

    /// Entry point to sub-flow that dispatches the next state based on the
    /// incoming command.
    ///
    /// @return next state appropriate to the command, else
    ///         Defs::ERROR_INVALID_ARGS on error
    Action entry() override
    {
        HASSERT(input()->address_ > 0 && input()->address_ <= MAX_ADDRESS);
        switch (input()->cmd_)
        {
            case Command::CMD_QUERY:
                return call_immediately(STATE(query));
            case Command::CMD_SET:
                return call_immediately(STATE(set));
            default:
                return return_with_error(Defs::ERROR_INVALID_ARGS);
        }
    }

    /// Query the last known state of the accessory address.
    ///
    /// @return waits for the query message to go out and advances to the
    ///         parent flow's next state once the memory holding the message is
    ///         freed.
    Action query()
    {
        eventProducer_->send_query_consumer(input()->address_ - 1, &writer_,
                                            bn_.reset(this));
        return wait_and_return_ok();
    }

    /// Set the state of the accessory address.
    ///
    /// @return waits for the query message to go out and advances to the
    ///         parent flow's next state once the memory holding the message is
    ///         freed.
    Action set()
    {
        eventProducer_->set(input()->address_ - 1, input()->value_, &writer_,
                            bn_.reset(this));
        return wait_and_return_ok();
    }

    /// Helper method for accessing the subflow input data
    ///
    /// @return pointer to the subflow data
    DccAccyProducerInput *input()
    {
        return message()->data();
    }

    /// Callback called when there is a notification of event (DCC accessory
    /// address) state.  Pass to the next level up after accounting for DCC
    /// address range starting at 1.
    ///
    /// @param bit bit index within event pair range
    /// @param value value of the event pair
    static void state_callback(unsigned bit, bool value)
    {
        /// @todo (Stuart Baker) should this be protected by a mutex with the
        /// constructor and destructor?
        for (unsigned i = 0; i < instances_->size(); ++i)
        {
            if (instances_->at(i)->dccStateCallback_)
            {
                instances_->at(i)->dccStateCallback_(bit + 1, value);
            }
        }
    }

    /// Singleton instance of a BitRangeNonAuthoritativeEventP for the DCC
    /// accessory address Well-Known Event ID range.
    static uninitialized<BitRangeNonAuthoritativeEventP> eventProducer_;

    /// Vector of all the subscribers to the DCC accessory address Well-Known
    /// Event ID Range
    static uninitialized<std::vector<DccAccyProducer*>> instances_;

    /// This lock protects the instances_ vector;
    static Atomic instancesLock_;

    /// one time execution helper
    static OSThreadOnce once_;

    /// Callback method that will be invoked when a consumer identified
    /// message is received with a known state.
    std::function<void(unsigned, bool)> dccStateCallback_;

    WriteHelper writer_; ///< statically allocated buffer
    BarrierNotifiable bn_; ///< notfiable for unblocking the next state

    DISALLOW_COPY_AND_ASSIGN(DccAccyProducer);
};

} // namespace openlcb

#endif // _OPENLCB_DCCACCYPRODUCER_HXX_
