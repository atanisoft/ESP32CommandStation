/** \copyright
 * Copyright (c) 2013, Stuart W Baker
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
 * \file EventHandlerContainer.hxx
 *
 * Defines containers of event handlers that are able to iterate through the
 * registered event handlers that ought to be called for a given incoming
 * event.
 *
 * @author Balazs Racz
 * @date 3 November 2013
 */

#ifndef _OPENLCB_EVENTHANDLERCONTAINER_HXX_
#define _OPENLCB_EVENTHANDLERCONTAINER_HXX_

#include <algorithm>
#include <vector>
#include <forward_list>
#include <endian.h>

#ifndef LOGLEVEL
//#define LOGLEVEL VERBOSE
#endif

#include "utils/Atomic.hxx"
#include "utils/logging.h"
#include "utils/SortedListMap.hxx"
#include "openlcb/EventHandler.hxx"
#include "openlcb/EventHandlerTemplates.hxx"

namespace openlcb
{

/// Abstract class for representing iteration through a container for event
/// handlers.
class EventIterator {
protected:
    /// Creates an EventIterator.
    EventIterator() {}

public:
    virtual ~EventIterator() {}

    /** Steps the iteration.
     * @returns the next entry or NULL if the iteration is done.
     * May be called many times after the iteratin is ended and should
     * consistently return NULL. */
    virtual EventRegistryEntry* next_entry() = 0;

    /** Starts the iteration. If the iteration is not done yet, call
     * clear_iteration first.
     *
     * @param event is the event report to reset the iteration for. */
    virtual void init_iteration(EventReport* event) = 0;

    /** Stops iteration and resets iteration variables. */
    virtual void clear_iteration() = 0;
};

/// EventIterator that produces every single entry in a given container (which
/// can be any STL-compatible container with begin() and end() methods).
template<class C> class FullContainerIterator : public EventIterator {
public:
    FullContainerIterator(C* container)
        : container_(container) {
        clear_iteration();
    }
    EventRegistryEntry* next_entry() OVERRIDE {
        if (it_ == container_->end()) return nullptr;
        EventRegistryEntry* h = &*it_;
        ++it_;
        return h;
    }
    void clear_iteration() OVERRIDE {
        it_ = container_->end();
    }
    void init_iteration(EventReport*) OVERRIDE {
        it_ = container_->begin();
    }

private:
    typename C::iterator it_;
    C* container_;
};

/// EventRegistry implementation that keeps all event handlers in a vector and
/// forwards every single call to each event handler.
class VectorEventHandlers : public EventRegistry {
 public:
    VectorEventHandlers() {}

    // Creates a new event iterator. Caller takes ownership of object.
    EventIterator* create_iterator() OVERRIDE {
        return new FullContainerIterator<HandlersList>(&handlers_);
    }

  void register_handler(const EventRegistryEntry& entry, unsigned mask) OVERRIDE {
    // @TODO(balazs.racz): need some kind of locking here.
    handlers_.push_front(entry);
    set_dirty();
  }
  void unregister_handler(EventHandler *handler) OVERRIDE
  {
      // @TODO(balazs.racz): need some kind of locking here.
      struct HandlerEquals
      {
          HandlerEquals(EventHandler *h) : h_(h)
          {
          }
          bool operator()(const EventRegistryEntry &e)
          {
              return e.handler == h_;
          }

      private:
          EventHandler *h_;
      } predicate(handler);
      handlers_.remove_if(predicate);
      set_dirty();
  }

 private:
  typedef std::forward_list<EventRegistryEntry> HandlersList;
  HandlersList handlers_;
};

/// EventRegistry implementation that keeps event handlers in a SortedListMap
/// and filters the event handler calls based on the registered event handler
/// arguments (id/mask).
class TreeEventHandlers : public EventRegistry, private Atomic {
public:
    TreeEventHandlers();

    EventIterator* create_iterator() OVERRIDE;
    void register_handler(const EventRegistryEntry &entry,
                          unsigned mask) OVERRIDE;
    void unregister_handler(EventHandler* handler) OVERRIDE;

private:
    class Iterator;
    friend class Iterator;

    /// Comparison operator for event registry entries.
    struct cmpop
    {
        bool operator()(const EventRegistryEntry &d, uint64_t k)
        {
            return d.event < k;
        }
        bool operator()(uint64_t k, const EventRegistryEntry &d)
        {
            return k < d.event;
        }
        bool operator()(const EventRegistryEntry &a, const EventRegistryEntry &b)
        {
            return a.event < b.event;
        }
    };

    typedef SortedListSet<EventRegistryEntry, cmpop> OneMaskMap;
    typedef std::map<uint8_t, OneMaskMap> MaskLookupMap;
    /** The registered handlers. The offset in the first map tell us how many
     * bits wide the registration is (it is the mask value in the register
     * call).*/
    MaskLookupMap handlers_;
};

}; /* namespace openlcb */

#endif  // _OPENLCB_EVENTHANDLERCONTAINER_HXX_
