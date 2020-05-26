/** @copyright
 * Copyright (c) 2017, Stuart W Baker
 * All rights reserved
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
 * @file MDNS.hxx
 *
 * A simple abstraction to publish/lookup mDNS sevices.
 *
 * @author Stuart Baker
 * @date 30 January 2017
 */

#ifndef _OS_MDNS_HXX_
#define _OS_MDNS_HXX_

#if defined (__linux__)
#include <netdb.h>
#include <stdio.h>
#include <semaphore.h>
#include <avahi-client/client.h>
#include <avahi-client/lookup.h>
#include <avahi-client/publish.h>
#include <avahi-common/error.h>
#include <avahi-common/malloc.h>
#include <avahi-common/simple-watch.h>

#include "os/OS.hxx"

#elif defined(__FreeRTOS__)
#include <netdb.h>
#endif

struct addrinfo;

#include "utils/macros.h"


/** MDNS abstraction object.
 */
class MDNS
#if defined (__linux__)
    : public OSThread
#endif
{
public:
    /** Constructor.
     */
    MDNS()
#if defined (__linux__)
        : 
          OSThread()
        , sem_()
        , group_(nullptr)
        , simplePoll_(nullptr)
        , client_(nullptr)
#endif
    {
#if defined (__linux__)
        start("mDNS Server", 0, 2048);
        printf("mDNS client start\n");        
        sem_.wait();
#endif
    }

    /** Destructor.
     */
    ~MDNS()
    {
#if defined(__linux__)
        shutdown();
#endif
    }

#if defined(__linux__)
    /// Requests that all exported services be cancelled, helper thread to exit,
    /// and all memory cleaned up. Blocks until the exit is completed.
    void shutdown();
#endif

    /** Publish an mDNS name.
     * @param name local "username" or "nodename" of the service
     * @param service service name, example: "_openlcb._tcp"
     * @param port port number
     */
    void publish(const char *name, const char *service, uint16_t port);

    /** Commit the mDNS publisher.
     */
    void commit()
    {
#if defined (__linux__)
        avahi_entry_group_commit(group_);
#endif
    }

    /** Lookup an mDNS name.
     * @param service servicename to lookup
     * @param hints hints about limiting the types of services that will respond
     * @param addrinfo structure containing one or more service addressess that
     *                 match the enquery, else nullptr if no matching service
     *                 found
     * @return 0 upon success, or appropriate EAI_* error on failure, use
     *         ::freeaddrinfo() to free up memory allocated to the non nullptr
     *         *addr returned
     */
    static int lookup(const char *service, struct addrinfo *hints,
                      struct addrinfo **addr);

    /** Start continuous scan for mDNS service name.
     * @param service servicename to scan
     */
    static void scan(const char *service);

private:
#if defined (__linux__)
    struct LookupUserdata
    {
        LookupUserdata(const struct addrinfo *h)
            : count(0)
            , done(0)
            , c(nullptr)
            , sp(nullptr)
            , hints(h)
            , addr(nullptr)
        {
        }

        unsigned count : 31;
        unsigned done  :  1;
        AvahiClient *c;
        AvahiSimplePoll *sp;
        const struct addrinfo *hints;
        struct addrinfo *addr;
    };

    /** Avahi service resolve callback
     * @param r Avahi service resolver
     * @param interface network interface result has come in on
     * @param protocol protocol of the result
     * @param event the type of callback
     * @param name name of service
     * @param type type of service
     * @param domain domain of service
     * @param host_name name of service host
     * @param address ip address of service host
     * @param port port number of service host
     * @param flags flags associated with the lookup
     * @param userdata user specified context
     */
    static void resolve_callback(AvahiServiceResolver *r,
                                 AvahiIfIndex interface, AvahiProtocol protocol,
                                 AvahiResolverEvent event, const char *name,
                                 const char *type, const char *domain,
                                 const char *host_name,
                                 const AvahiAddress *address, uint16_t port,
                                 AvahiStringList *txt,
                                 AvahiLookupResultFlags flags, void* userdata);

    /** Avahi browse callback.
     * @param b Avahi service browser
     * @param interface network interface result has come in on
     * @param protocol protocol of the result
     * @param event the type of callback
     * @param name name of service
     * @param type type of service
     * @param domain domain of service
     * @param flags flags associated with the lookup
     * @param userdata user specified context
     */
    static void browse_callback(AvahiServiceBrowser *b, AvahiIfIndex interface,
                                AvahiProtocol protocol, AvahiBrowserEvent event,
                                const char *name, const char *type,
                                const char *domain,
                                AvahiLookupResultFlags flags, void* userdata);

    /** Avahi group callback.
     * @param g Avahi entry group
     * @param state Avahi entry group state
     * @param userdata user specified context
     */
    static void entry_group_callback(AvahiEntryGroup *g,
                                     AvahiEntryGroupState state, void *userdata);

    /** Avahi client callback.
     * @param c Avahi client
     * @param state Avahi client state
     * @param userdata user specified context
     */
    static void client_callback(AvahiClient *c, AvahiClientState state,
                                void * userdata);

    /** mdns polling thread.
     * @return should never return
     */
    void *entry() override;

    /** Synchronize the startup of AVAHI */
    OSSem sem_;

    /** AVAHI group */
    AvahiEntryGroup *group_;

    /** AVAHI polling client */
    AvahiSimplePoll *simplePoll_;

    /** AVAHI client */
    AvahiClient *client_;
#endif

    DISALLOW_COPY_AND_ASSIGN(MDNS);
};

#endif /* _OS_MDNS_HXX_ */
