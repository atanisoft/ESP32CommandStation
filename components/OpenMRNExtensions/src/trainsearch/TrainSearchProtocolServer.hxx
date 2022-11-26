/*
 * SPDX-FileCopyrightText: 2014-2016 Balazs Racz
 * SPDX-FileCopyrightText: 2020-2022 Mike Dunston (atanisoft)
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef TSP_SERVER_HXX_
#define TSP_SERVER_HXX_

#include "locodb/LocoDatabaseEntry.hxx"
#include "locomgr/LocoManager.hxx"
#include "trainsearch/Defs.hxx"
#include "utils/StringUtils.hxx"

#include <openlcb/EventHandlerTemplates.hxx>
#include <openlcb/TractionTrain.hxx>
#include <utils/constants.hxx>

namespace trainsearch
{

#ifndef TSP_LOG_LEVEL
#ifdef CONFIG_TSP_LOGGING_LEVEL
#define TSP_LOG_LEVEL CONFIG_TSP_LOGGING_LEVEL
#else
#define TSP_LOG_LEVEL VERBOSE
#endif // CONFIG_TSP_LOGGING_LEVEL
#endif // TSP_LOG_LEVEL

DECLARE_CONST(trainsearch_allocate_delay_ms);
DECLARE_CONST(trainsearch_new_node_check_interval_ms);

/// Implementation of the Train Search Protocol specification.
///
/// https://github.com/openlcb/documents/blob/master/drafts/TrainSearchS.pdf
class TrainSearchProtocolServer : public openlcb::SimpleEventHandler
{
public:
  using Node = openlcb::Node;
  using NodeID = openlcb::NodeID;
  using MessageHandler = openlcb::MessageHandler;
  using If = openlcb::If;
  using EventId = openlcb::EventId;
  using Defs = openlcb::Defs;
  using GenMessage = openlcb::GenMessage;
  using TractionDefs = openlcb::TractionDefs;
  using EventRegistry = openlcb::EventRegistry;
  using WriteHelper = openlcb::WriteHelper;

  /// Constructor.
  TrainSearchProtocolServer()
    : flow_(this, Singleton<locomgr::LocoManager>::instance())
  {
  }

  /// Destructor.
  ~TrainSearchProtocolServer()
  {
    set_enabled(false);
  }

  /// Enables/Disables the @ref TrainSearchProtocolServer based on persistent
  /// configuration settings.
  ///
  /// @param enabled When true the @ref TrainSearchProtocolServer will listen for and
  /// respond to OpenLCB Events related to train search.
  void set_enabled(bool enabled)
  {
    if (enabled)
    {
      EventRegistry::instance()->register_handler(
          EventRegistryEntry(this, TrainSearchDefs::EVENT_SEARCH_BASE,
                             USER_ARG_FIND),
          TrainSearchDefs::TRAIN_FIND_MASK);
      EventRegistry::instance()->register_handler(
          EventRegistryEntry(this, IS_TRAIN_EVENT, USER_ARG_ISTRAIN), 0);
    }
    else
    {
      EventRegistry::instance()->unregister_handler(this);
    }
  }

  /// Handles identifying locomotives that are valid for this node.
  ///
  /// @param registry_entry @ref EventRegistryEntry used for this event.
  /// @param event @ref EventReport that was received.
  /// @param done @ref BarrierNotifiable to notify when processing has
  /// completed.
  void handle_identify_global(const EventRegistryEntry &registry_entry,
                              EventReport *event,
                              BarrierNotifiable *done) override
  {
    AutoNotify an(done);

    if (event && event->dst_node)
    {
      auto *locoMgr = Singleton<locomgr::LocoManager>::instance();
      // Identify addressed
      if (!locoMgr->is_known_train_node(event->dst_node))
      {
        LOG(VERBOSE, "ignoring unknown train node: %s",
            utils::node_id_to_string(event->dst_node->node_id()).c_str());
        return;
      }
      if (registry_entry.user_arg == USER_ARG_FIND)
      {
        event->event_write_helper<1>()->WriteAsync(event->dst_node,
            Defs::MTI_PRODUCER_IDENTIFIED_RANGE, WriteHelper::global(),
            openlcb::eventid_to_buffer(TrainSearchDefs::EVENT_SEARCH_BASE),
            done->new_child());
      }
      else if (registry_entry.user_arg == USER_ARG_ISTRAIN)
      {
        event->event_write_helper<1>()->WriteAsync(event->dst_node,
            Defs::MTI_PRODUCER_IDENTIFIED_UNKNOWN, WriteHelper::global(),
            openlcb::eventid_to_buffer(IS_TRAIN_EVENT), done->new_child());
      }
    }
    else
    {
      // Identify global
      if (pendingGlobalIdentify_ ||
          registry_entry.user_arg == USER_ARG_ISTRAIN)
      {
        // We have not started processing the global identify yet. Swallow this
        // one.
        LOG(VERBOSE, "discarding duplicate global identify");
        return;
      }
      // We do a synchronous alloc here but there isn't a much better choice.
      auto *b = flow_.alloc();
      // Can't do this -- see handleidentifyproducer.
      // b->set_done(done->new_child());
      pendingGlobalIdentify_ = true;
      b->data()->reset(REQUEST_GLOBAL_IDENTIFY);
      flow_.send(b);
    }
  }

  /// Handles event producer notices.
  ///
  /// @param registry_entry @ref EventRegistryEntry used for this event.
  /// @param event @ref EventReport that was received.
  /// @param done @ref BarrierNotifiable to notify when processing has
  /// completed.
  void handle_identify_producer(const EventRegistryEntry &registry_entry,
                                EventReport *event,
                                BarrierNotifiable *done) override
  {
    AutoNotify an(done);

    auto *b = flow_.alloc();
    // This would be nice in that we would prevent allocating more buffers
    // while the previous request is serviced. However, thereby we also block
    // the progress on the event handler flow, which will cause deadlocks,
    // since servicing the request involves sending events out that will be
    // looped back.
    //
    // b->set_done(done->new_child());
    b->data()->reset(event->event);
    if (event->event == IS_TRAIN_EVENT)
    {
      pendingIsTrain_ = true;
    }
    flow_.send(b);
  };

 private:

  /// Send this in the event_ field if there is a global identify
  /// pending. This is not a valid EventID, because the upper byte is 0.
  static constexpr EventId REQUEST_GLOBAL_IDENTIFY = 0x0001000000000000U;

  //// Local copy of the well-known event ID produced by all train nodes.
  static constexpr uint64_t IS_TRAIN_EVENT = TractionDefs::IS_TRAIN_EVENT;

  /// Argument provided to global identify event flow for searching for trains.
  static constexpr unsigned USER_ARG_FIND = 1;

  /// Argument provided to global identify event flow for identifying a
  /// specific train that may be handled by this node.
  static constexpr unsigned USER_ARG_ISTRAIN = 2;

  /// Search request structure.
  struct SearchRequest
  {
    /// Sets the request event ID.
    /// @param event @ref EventId to process.
    void reset(EventId event)
    {
      event_ = event;
      allocate_ = event_ & TrainSearchDefs::ALLOCATE;
    }

    /// @ref EventId to process.
    EventId event_;

    /// True if we should allocate a new train node if not already active.
    bool allocate_ : 1;
  };

  /// Processes train search requests
  class TrainSearchProtocolFlow : public StateFlow<Buffer<SearchRequest>, QList<1> >
  {
  public:
  
    /// Constructor.
    ///
    /// @param parent @ref TrainSearchProtocolServer that owns this request
    /// flow.
    TrainSearchProtocolFlow(TrainSearchProtocolServer *parent,
        locomgr::LocoManager* locoManager)
        : StateFlow(locoManager->train_service()),
        messageFlow_(locoManager->iface()->global_message_write_flow()),
        parent_(parent), locoManager_(locoManager)
    {

    }

    /// Primary entry point for processing requests.
    Action entry() override
    {
      LOG(TSP_LOG_LEVEL, "event:%s",
          utils::event_id_to_string(message()->data()->event_).c_str());
      if (message()->data()->event_ == REQUEST_GLOBAL_IDENTIFY)
      {
        LOG(TSP_LOG_LEVEL, "global REQUEST_GLOBAL_IDENTIFY");
        isGlobal_ = true;
        if (!parent_->pendingGlobalIdentify_)
        {
          LOG(TSP_LOG_LEVEL, "duplicate global REQUEST_GLOBAL_IDENTIFY");
          // Duplicate global identify, or the previous one was already handled.
          return release_and_exit();
        }
        parent_->pendingGlobalIdentify_ = false;
      }
      else if (message()->data()->event_ == IS_TRAIN_EVENT)
      {
        LOG(TSP_LOG_LEVEL, "global IS_TRAIN_EVENT");
        isGlobal_ = true;
        if (!parent_->pendingIsTrain_)
        {
          LOG(TSP_LOG_LEVEL, "duplicate global IS_TRAIN_EVENT");
          // Duplicate is_train, or the previous one was already handled.
          return release_and_exit();
        }
        parent_->pendingIsTrain_ = false;
      }
      else
      {
        LOG(TSP_LOG_LEVEL, "!REQUEST_GLOBAL_IDENTIFY && !IS_TRAIN_EVENT");
        isGlobal_ = false;
      }
      LOG(TSP_LOG_LEVEL, "starting iteration");
      iterateId_ = 0;
      hasMatches_ = false;
      return call_immediately(STATE(iterate));
    }

    /// Entry point for iterating the @ref LocoDatabase for matching entries.
    Action iterate()
    {
      LOG(TSP_LOG_LEVEL, "iterate: %zu", iterateId_);
      if (!Singleton<locodb::LocoDatabase>::instance()->is_valid_train(iterateId_))
      {
        LOG(TSP_LOG_LEVEL, "iterate: finished");
        return sleep_and_call(
          &timer_, MSEC_TO_NSEC(config_trainsearch_allocate_delay_ms()),
          STATE(maybe_allocate_node));
      }
      if (isGlobal_)
      {
        if (message()->data()->event_ == REQUEST_GLOBAL_IDENTIFY &&
            parent_->pendingGlobalIdentify_)
        {
          LOG(TSP_LOG_LEVEL, "iterate: REQUEST_GLOBAL_IDENTIFY reset count");
          // Another notification arrived. Start iteration from 0.
          iterateId_ = 0;
          parent_->pendingGlobalIdentify_ = false;
          return again();
        }
        if (message()->data()->event_ == IS_TRAIN_EVENT &&
            parent_->pendingIsTrain_)
        {
          LOG(TSP_LOG_LEVEL, "iterate: IS_TRAIN_EVENT reset count");
          // Another notification arrived. Start iteration from 0.
          iterateId_ = 0;
          parent_->pendingIsTrain_ = false;
          return again();
        }
        LOG(TSP_LOG_LEVEL, "iterate: send_response %zu: %s", iterateId_,
            utils::event_id_to_string(message()->data()->event_).c_str());
        return allocate_and_call(messageFlow_, STATE(send_response));
      }
      LOG(TSP_LOG_LEVEL, "iterate: try_traindb_lookup");
      return call_immediately(STATE(try_traindb_lookup));
    }

    /// This state attempts to look up the entry in the train database.
    Action try_traindb_lookup()
    {
      auto entry =
        Singleton<locodb::LocoDatabase>::instance()->get_entry(iterateId_);
      if (entry && 
          TrainSearchDefs::match_query_to_node(
            message()->data()->event_, entry.get()))
      {
        LOG(TSP_LOG_LEVEL, "try_traindb_lookup: MATCH: %zu:%s", iterateId_,
            entry->identifier().c_str());
        hasMatches_ = true;
        return allocate_and_call(messageFlow_, STATE(send_response));
      }
      LOG(TSP_LOG_LEVEL, "try_traindb_lookup: NOT MATCHED:%zu", iterateId_);
      return yield_and_call(STATE(iterate_next));
    }

    /// Sends a result response to the callers.
    Action send_response()
    {
      auto *b = get_allocation_result(messageFlow_);
      b->set_done(bn_.reset(this));
      auto next_node_id = locoManager_->get_train_node_id(iterateId_);
      if (message()->data()->event_ == REQUEST_GLOBAL_IDENTIFY)
      {
        b->data()->reset(
          Defs::MTI_PRODUCER_IDENTIFIED_RANGE, next_node_id,
          openlcb::eventid_to_buffer(TrainSearchDefs::EVENT_SEARCH_BASE));
        // send is_train event too.
        auto *bb = messageFlow_->alloc();
        bb->data()->reset(
          Defs::MTI_PRODUCER_IDENTIFIED_UNKNOWN, next_node_id,
          openlcb::eventid_to_buffer(IS_TRAIN_EVENT));
        bb->set_done(bn_.new_child());
        bb->data()->set_flag_dst(GenMessage::WAIT_FOR_LOCAL_LOOPBACK);
        messageFlow_->send(bb);
      }
      else if (message()->data()->event_ == IS_TRAIN_EVENT)
      {
        b->data()->reset(
          Defs::MTI_PRODUCER_IDENTIFIED_UNKNOWN, next_node_id,
          openlcb::eventid_to_buffer(message()->data()->event_));
      }
      else
      {
        b->data()->reset(
          Defs::MTI_PRODUCER_IDENTIFIED_VALID, next_node_id,
          openlcb::eventid_to_buffer(message()->data()->event_));
      }
      b->data()->set_flag_dst(GenMessage::WAIT_FOR_LOCAL_LOOPBACK);
      messageFlow_->send(b);

      return wait_and_call(STATE(iterate_next));
    }

    /// Moves to the next train identifier.
    Action iterate_next()
    {
      iterateId_++;
      return call_immediately(STATE(iterate));
    }

    Action maybe_allocate_node()
    {
      if (!hasMatches_ && !isGlobal_ && message()->data()->allocate_)
      {
        LOG(TSP_LOG_LEVEL,
            "No matches were found in Locomotive Roster, attempting to "
            "allocate a new locomotive.");
        return yield_and_call(STATE(allocate_new_locomotive));
      }
      LOG(TSP_LOG_LEVEL,
          "No matches were found in Locomotive Roster, giving up.");
      return release_and_exit();
    }

    /// This state is called when we reach the end of the @ref LocoDatabase
    /// registered entries and need to allocate a new locomotive.
    Action allocate_new_locomotive()
    {
      locodb::DriveMode mode;
      unsigned address =
        TrainSearchDefs::query_to_address(message()->data()->event_, &mode);
      newNodeId_ = locoManager_->create_train_node(mode, address);
      if (!newNodeId_)
      {
        LOG(WARNING, "Decided to allocate node but failed. type=%d addr=%u",
            static_cast<int>(mode), address);
        return release_and_exit();
      }
      return call_immediately(STATE(wait_for_node));
    }

    /// Yields until the new node is initialized and we are allowed to send
    /// traffic out from it.
    Action wait_for_node()
    {
      Node *n = locoManager_->iface()->lookup_local_node(newNodeId_);
      HASSERT(n);
      if (n->is_initialized())
      {
        return allocate_and_call(messageFlow_, STATE(send_new_node_response));
      }
      else
      {
        return sleep_and_call(&timer_,
          MSEC_TO_NSEC(config_trainsearch_new_node_check_interval_ms()),
          STATE(wait_for_node));
      }
    }

    /// Sends the matching entry from the @ref LocoDatabase to the caller.
    Action send_new_node_response()
    {
      auto *b = get_allocation_result(messageFlow_);
      b->data()->reset(Defs::MTI_PRODUCER_IDENTIFIED_VALID, newNodeId_,
                       openlcb::eventid_to_buffer(message()->data()->event_));
      messageFlow_->send(b);
      return release_and_exit();
    }

  private:
    /// @ref MessageHandler to send events to.
    MessageHandler *messageFlow_;

    /// @ref TrainSearchProtocolServer that owns this flow.
    TrainSearchProtocolServer *parent_;

    /// @ref LocoManager to use for looking up locomotive details.
    locomgr::LocoManager *locoManager_;

    /// True if we found any matches during the search.
    bool hasMatches_ : 1;

    /// True if the current search has to touch every node.
    bool isGlobal_ : 1;

    /// Holder for train identifier to load.
    size_t iterateId_;

    /// Holder for newly allocated train node.
    NodeID newNodeId_;

    /// @ref BarrierNotifiable to use for controlling iteration of locomotive
    /// database entries.
    BarrierNotifiable bn_;

    /// @ref StatFlowTimer used for handling state transitions when delays are
    /// used.
    StateFlowTimer timer_{this};
  };

  /// Set to true when a global identify message is received. When a global
  /// identify starts processing, it shall be set to false. If a global
  /// identify request arrives with no pendingGlobalIdentify_, that is a
  /// duplicate request that can be ignored.
  uint8_t pendingGlobalIdentify_{false};

  /// Same as pendingGlobalIdentify_ for the IS_TRAIN event producer.
  uint8_t pendingIsTrain_{false};

  /// State flow that handles the processing of any search requests.
  TrainSearchProtocolFlow flow_;
};

class SingleNodeTrainSearchProtocolServer : public openlcb::SimpleEventHandler
{
public:
  using Node = openlcb::Node;
  using Defs = openlcb::Defs;
  using EventRegistry = openlcb::EventRegistry;

  SingleNodeTrainSearchProtocolServer(Node *node, locodb::LocoDatabaseEntry *db_entry)
      : node_(node), dbEntry_(db_entry)
  {
    EventRegistry::instance()->register_handler(
        EventRegistryEntry(this, TrainSearchDefs::EVENT_SEARCH_BASE),
        TrainSearchDefs::TRAIN_FIND_MASK);
  }

  ~SingleNodeTrainSearchProtocolServer()
  {
    EventRegistry::instance()->unregister_handler(this);
  }

  void handle_identify_global(const EventRegistryEntry &registry_entry,
                              EventReport *event,
                              BarrierNotifiable *done) override
  {
    AutoNotify an(done);

    if (event && event->dst_node)
    {
      // Identify addressed
      if (event->dst_node != node_) return;
    }

    static_assert(((TrainSearchDefs::EVENT_SEARCH_BASE >>
                    TrainSearchDefs::TRAIN_FIND_MASK) &
                   1) == 1,
                  "The lowermost bit of the EVENT_SEARCH_BASE must be 1 or "
                  "else the event produced range encoding must be updated.");
    event->event_write_helper<1>()->WriteAsync(
        event->dst_node, Defs::MTI_PRODUCER_IDENTIFIED_RANGE,
        openlcb::WriteHelper::global(),
        openlcb::eventid_to_buffer(TrainSearchDefs::EVENT_SEARCH_BASE),
        done->new_child());
  }

  void handle_identify_producer(const EventRegistryEntry &registry_entry,
                                EventReport *event,
                                BarrierNotifiable *done) override
  {
    AutoNotify an(done);

    if (TrainSearchDefs::match_query_to_node(event->event, dbEntry_))
    {
      event->event_write_helper<1>()->WriteAsync(
          node_, Defs::MTI_PRODUCER_IDENTIFIED_VALID,
          openlcb::WriteHelper::global(),
          openlcb::eventid_to_buffer(event->event),
          done->new_child());
    }
  };

 private:
  Node* node_;
  locodb::LocoDatabaseEntry* dbEntry_;
};

} // namespace trainsearch

#endif  // TSP_SERVER_HXX_