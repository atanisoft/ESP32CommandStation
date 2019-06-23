//#define LOGLEVEL VERBOSE
#include "utils/logging.h"

#include <algorithm>
#include <vector>
#include <endian.h>

#include "openlcb/EventService.hxx"

#include "openlcb/EventServiceImpl.hxx"
#include "openlcb/EventHandler.hxx"
#include "openlcb/EventHandlerTemplates.hxx"
#include "openlcb/EventHandlerContainer.hxx"
#include "openlcb/Defs.hxx"
#include "openlcb/EndianHelper.hxx"

namespace openlcb
{

/*static*/
EventService *EventService::instance = nullptr;

EventService::EventService(ExecutorBase *e) : Service(e)
{
    HASSERT(instance == nullptr);
    instance = this;
    impl_.reset(new Impl(this));
}

EventService::EventService(If *iface) : Service(iface->executor())
{
    HASSERT(instance == nullptr);
    instance = this;
    impl_.reset(new Impl(this));
    register_interface(iface);
}

EventService::~EventService()
{
    HASSERT(instance == this);
    instance = nullptr;
}

void EventService::register_interface(If *iface)
{
    impl()->ownedFlows_.emplace_back(new InlineEventIteratorFlow(
        iface, this, EventService::Impl::MTI_VALUE_EVENT,
        EventService::Impl::MTI_MASK_EVENT));
    impl()->ownedFlows_.emplace_back(new EventIteratorFlow(
        iface, this, EventService::Impl::MTI_VALUE_GLOBAL,
        EventService::Impl::MTI_MASK_GLOBAL));
    impl()->ownedFlows_.emplace_back(new EventIteratorFlow(
        iface, this, EventService::Impl::MTI_VALUE_ADDRESSED_ALL,
        EventService::Impl::MTI_MASK_ADDRESSED_ALL));
}

EventService::Impl::Impl(EventService *service) : callerFlow_(service)
{
#ifdef TARGET_LPC11Cxx
    registry.reset(new VectorEventHandlers());
#else
    registry.reset(new TreeEventHandlers());
#endif
}

EventService::Impl::~Impl()
{
}

StateFlowBase::Action EventCallerFlow::entry()
{
    EventHandlerCall *c = message()->data();
    if (c->epoch != EventRegistry::instance()->get_epoch())
    {
        // Event registry was invalidated since this call was scheduled. Ignore.
        return call_immediately(STATE(call_done));
    }
    n_.reset(this);
    (c->registry_entry->handler->*(c->fn))(*c->registry_entry, c->rep, &n_);
    return wait_and_call(STATE(call_done));
}

StateFlowBase::Action EventCallerFlow::call_done()
{
    return release_and_exit();
}

EventIteratorFlow::EventIteratorFlow(If *async_if, EventService *event_service,
                                     unsigned mti_value, unsigned mti_mask)
    : IncomingMessageStateFlow(async_if)
    , eventService_(event_service)
    , iterator_(event_service->impl()->registry->create_iterator())
#ifdef DEBUG_EVENT_PERFORMANCE
    , mtiValue_(mti_value)
#endif
{
    iface()->dispatcher()->register_handler(this, mti_value, mti_mask);
}

EventIteratorFlow::~EventIteratorFlow()
{
    iface()->dispatcher()->unregister_handler_all(this);
    delete iterator_;
}

/// Returns true if there are outstanding events that are not yet handled.
bool EventService::event_processing_pending()
{
    for (auto &f : impl()->ownedFlows_)
    {
        if (!f->is_waiting())
            return true;
    }
    return false;
}

void DecodeRange(EventReport *r)
{
    uint64_t e = r->event;
    if (e & 1)
    {
        r->mask = (e ^ (e + 1)) >> 1;
    }
    else
    {
        r->mask = (e ^ (e - 1)) >> 1;
    }
    r->event &= ~r->mask;
}

StateFlowBase::Action EventIteratorFlow::entry()
{
    // at this point: we have the mutex.
    LOG(VERBOSE, "GlobalFlow::HandleEvent");
#ifdef DEBUG_EVENT_PERFORMANCE
    currentProcessStart_ = os_get_time_monotonic();
#endif
    EventReport *rep = &eventReport_;
    rep->src_node = nmsg()->src;
    rep->dst_node = nmsg()->dstNode;
    if ((nmsg()->mti & Defs::MTI_EVENT_MASK) == Defs::MTI_EVENT_MASK)
    {
        if (nmsg()->payload.size() != 8)
        {
            LOG(INFO, "Invalid input event message, payload length %d",
                (unsigned)nmsg()->payload.size());
            return release_and_exit();
        }
        rep->event = NetworkToEventID(nmsg()->payload.data());
        rep->mask = 0;
    }
    else
    {
        // Message without event payload.
        rep->event = 0;
        /// @TODO(balazs.racz) refactor this into a global constant.
        rep->mask = 0xFFFFFFFFFFFFFFFFULL;
    }

    switch (nmsg()->mti)
    {
        case Defs::MTI_EVENT_REPORT:
            fn_ = &EventHandler::handle_event_report;
            break;
        case Defs::MTI_CONSUMER_IDENTIFY:
            fn_ = &EventHandler::handle_identify_consumer;
            break;
        case Defs::MTI_CONSUMER_IDENTIFIED_RANGE:
            DecodeRange(rep);
            fn_ = &EventHandler::handle_consumer_range_identified;
            break;
        case Defs::MTI_CONSUMER_IDENTIFIED_UNKNOWN:
            rep->state = EventState::UNKNOWN;
            fn_ = &EventHandler::handle_consumer_identified;
            break;
        case Defs::MTI_CONSUMER_IDENTIFIED_VALID:
            rep->state = EventState::VALID;
            fn_ = &EventHandler::handle_consumer_identified;
            break;
        case Defs::MTI_CONSUMER_IDENTIFIED_INVALID:
            rep->state = EventState::INVALID;
            fn_ = &EventHandler::handle_consumer_identified;
            break;
        case Defs::MTI_CONSUMER_IDENTIFIED_RESERVED:
            rep->state = EventState::RESERVED;
            fn_ = &EventHandler::handle_consumer_identified;
            break;
        case Defs::MTI_PRODUCER_IDENTIFY:
            fn_ = &EventHandler::handle_identify_producer;
            break;
        case Defs::MTI_PRODUCER_IDENTIFIED_RANGE:
            DecodeRange(rep);
            fn_ = &EventHandler::handle_producer_range_identified;
            break;
        case Defs::MTI_PRODUCER_IDENTIFIED_UNKNOWN:
            rep->state = EventState::UNKNOWN;
            fn_ = &EventHandler::handle_producer_identified;
            break;
        case Defs::MTI_PRODUCER_IDENTIFIED_VALID:
            rep->state = EventState::VALID;
            fn_ = &EventHandler::handle_producer_identified;
            break;
        case Defs::MTI_PRODUCER_IDENTIFIED_INVALID:
            rep->state = EventState::INVALID;
            fn_ = &EventHandler::handle_producer_identified;
            break;
        case Defs::MTI_PRODUCER_IDENTIFIED_RESERVED:
            rep->state = EventState::RESERVED;
            fn_ = &EventHandler::handle_producer_identified;
            break;
        case Defs::MTI_EVENTS_IDENTIFY_ADDRESSED:
            if (!rep->dst_node)
            {
                LOG(INFO, "Invalid addressed identify all message, destination "
                          "node not found");
                return release_and_exit();
            }
        // fall through
        case Defs::MTI_EVENTS_IDENTIFY_GLOBAL:
            fn_ = &EventHandler::handle_identify_global;
            // Reduces the priority so that we let the priority 3 event messages
            // be processed before the global identify events makes any
            // progress.
            set_priority(4);
            break;
        default:
            DIE("Unexpected message arrived at the global event handler.");
    } //    case
    // The incoming message is not needed anymore.
    incomingDone_ = message()->new_child();
    release();

    eventRegistryEpoch_ = eventService_->impl()->registry->get_epoch();
    iterator_->init_iteration(rep);
    return yield_and_call(STATE(iterate_next));
}

StateFlowBase::Action EventIteratorFlow::iterate_next()
{
    if (eventRegistryEpoch_ != eventService_->impl()->registry->get_epoch())
    {
        // Iterators are invalidated. We need to start over. This may cause
        // duplicate delivery of the same events.
        iterator_->clear_iteration();
        eventRegistryEpoch_ = eventService_->impl()->registry->get_epoch();
        iterator_->init_iteration(&eventReport_);
    }

    EventRegistryEntry *entry = iterator_->next_entry();
    if (!entry)
    {
        if (incomingDone_)
        {
            incomingDone_->notify();
            incomingDone_ = nullptr;
        }

#ifdef DEBUG_EVENT_PERFORMANCE
        long long len = os_get_time_monotonic() - currentProcessStart_;
        numProcessNsec_ += len;
        countEvents_++;
        if (countEvents_ >= REPORT_COUNT)
        {
            //long msec = numProcessNsec_ / 1000000;
            //printf("event perf for mti %04x: %ld msec for %d events\n",
            //       mtiValue_, msec, REPORT_COUNT);
            countEvents_ = 0;
            numProcessNsec_ = 0;
        }

#endif

        return exit();
    }
    return dispatch_event(entry);
}

StateFlowBase::Action EventIteratorFlow::dispatch_event(const EventRegistryEntry *entry)
{
    Buffer<EventHandlerCall> *b;
    /* This could be made an asynchronous allocation. Then the pool could be
     * made fixed size. */
    eventService_->impl()->callerFlow_.pool()->alloc(&b, nullptr);
    HASSERT(b);
    b->data()->reset(entry, eventRegistryEpoch_, &eventReport_, fn_);
    n_.reset(this);
    b->set_done(&n_);
    eventService_->impl()->callerFlow_.send(b, priority());
    return wait();
}

StateFlowBase::Action
InlineEventIteratorFlow::dispatch_event(const EventRegistryEntry *entry)
{
    currentEntry_ = entry;
    if (eventRegistryEpoch_ != eventService_->impl()->registry->get_epoch())
    {
        // Will restart iteration.
        return call_immediately(STATE(iterate_next));
    }
    n_.reset(this);
    // It is required to hold on to a child to call abort_if_almost_done.
    auto *c = n_.new_child();
    (currentEntry_->handler->*(fn_))(*currentEntry_, &eventReport_, &n_);
    if (n_.abort_if_almost_done())
    {
        // Aborted. Event handler did not do any asynchronous action.
        return call_immediately(STATE(iterate_next));
    }
    else
    {
        c->notify();
        return wait_and_call(STATE(iterate_next));
    }
}

} /* namespace openlcb */
