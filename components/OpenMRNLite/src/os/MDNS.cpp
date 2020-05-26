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
 * @file MDNS.cxx
 *
 * A simple abstraction to publish/lookup mDNS sevices.
 *
 * @author Stuart Baker
 * @date 9 April 2017
 */

#include "os/MDNS.hxx"

/** Turn on/off debug print statements */
#define MDNS_DEBUG 0

#if !defined (__linux__)
void mdns_publish(const char *name, const char *service, uint16_t port) __attribute__ ((weak));
int mdns_lookup(const char *service, struct addrinfo *hints,
                struct addrinfo **addr) __attribute__ ((weak));
void mdns_scan(const char *service) __attribute__ ((weak));

/** Publish an mDNS name.
 * @param name local "username" or "nodename" of the service
 * @param service service name, example: "_openlcb._tcp"
 * @param port port number
 */
void mdns_publish(const char *name, const char *service, uint16_t port)
{
    HASSERT(0);
}

/** Lookup an mDNS name.
 * @param service servicename to lookup
 * @param hints hints about limiting the types of services that will respond
 * @param addrinfo structure containing one or more service addressess that
 *                 match the enquery, else nullptr if no matching service found
 * @return 0 upon success, or appropriate EAI_* error on failure
 */
int mdns_lookup(const char *service, struct addrinfo *hints,
                struct addrinfo **addr)
{
#if defined(__FreeRTOS__)
    string mdns_name(service);
    mdns_name.append(".local");
    return ::getaddrinfo(nullptr, mdns_name.c_str(), hints, addr);
#else
    DIE("Your OS does not support mDNS");
#endif
}

/** Start continuous scan for mDNS service name.
 * @param service servicename to scan
 */
void mdns_scan(const char *service)
{
    HASSERT(0);
}
#endif

/*
 * MDNS::publish()
 */
void MDNS::publish(const char *name, const char *service, uint16_t port)
{
#if defined (__linux__)
    name = avahi_strdup(name);

    if (!group_)
    {
        group_ = avahi_entry_group_new(client_, entry_group_callback, this);
        if (!group_)
        {
            fprintf(stderr, "avahi_entry_group_new() failed: %s\n",
                avahi_strerror(avahi_client_errno(client_)));
            return;
        }
        HASSERT(group_);
    }

    int result = avahi_entry_group_add_service(group_, AVAHI_IF_UNSPEC,
                                               AVAHI_PROTO_UNSPEC,
                                               (AvahiPublishFlags)0, name,
                                               service, NULL, NULL,
                                               port, NULL);
    
    if (result != 0)
    {
#if MDNS_DEBUG
        fprintf(stderr, "Error exporting mDNS name (%d) %s\n", result,
                avahi_strerror(result));
#endif
        return;
    }
    HASSERT(result == 0);
#else
    mdns_publish(name, service, port);
#endif
}

/*
 * MDNS::lookup()
 */
int MDNS::lookup(const char *service, struct addrinfo *hints,
                 struct addrinfo **addr)
{
#if defined (__linux__)
    LookupUserdata lu(hints);
    AvahiServiceBrowser *sb = nullptr;
    int error;
    int result = 0;
    int protocol;


    switch (hints->ai_family)
    {
        case AF_INET:
            protocol = AVAHI_PROTO_INET;
            break;
        case AF_INET6:
            protocol = AVAHI_PROTO_INET6;
            break;
        case AF_UNSPEC:
            protocol = AVAHI_PROTO_UNSPEC;
            break;
        default:
            result = EAI_FAMILY;
            goto fail;
    }

    *addr = nullptr;

    lu.sp = avahi_simple_poll_new();
    if (!lu.sp)
    {
        result = EAI_MEMORY;
        goto fail;
    }

    lu.c = avahi_client_new(avahi_simple_poll_get(lu.sp),
                            (AvahiClientFlags)0, client_callback,
                            nullptr, &error);

    if (!lu.c)
    {
        result = EAI_MEMORY;
        goto fail;
    }

    sb = avahi_service_browser_new(lu.c, AVAHI_IF_UNSPEC, protocol, service,
                                   nullptr, (AvahiLookupFlags)0,
                                   browse_callback, (void*)&lu);

    if (!sb)
    {
        result = EAI_MEMORY;
        goto fail;
    }

    avahi_simple_poll_loop(lu.sp);

    if (lu.addr)
    {
        *addr = lu.addr;
        printf("lu.addr\n");
    }
    else
    {
        *addr = nullptr;
        result = EAI_NONAME;
    }

fail:
    if (sb)
    {
        avahi_service_browser_free(sb);
    }

    if (lu.c)
    {
        avahi_client_free(lu.c);
    }

    if (lu.sp)
    {
        avahi_simple_poll_free(lu.sp);
    }

    return result;
#else
    return mdns_lookup(service, hints, addr);
#endif
}

/*
 * MDNS::scan()
 */
void MDNS::scan(const char *service)
{
#if defined (__linux__)
#else
    mdns_scan(service);
#endif
}

#if defined (__linux__)
/*
 * MDNS::resolve_callback()
 */
void MDNS::resolve_callback(AvahiServiceResolver *r,
                            AvahiIfIndex interface, AvahiProtocol protocol,
                            AvahiResolverEvent event, const char *name,
                            const char *type, const char *domain,
                            const char *host_name,
                            const AvahiAddress *address, uint16_t port,
                            AvahiStringList *txt,
                            AvahiLookupResultFlags flags, void* userdata)
{
    HASSERT(r);
    LookupUserdata *lu = static_cast<LookupUserdata*>(userdata);

    /* Called whenever a service has been resolved successfully or timed out */
    switch (event)
    {
        case AVAHI_RESOLVER_FAILURE:
#if MDNS_DEBUG
            fprintf(stderr, "(Resolver) Failed to resolve service '%s' of type "
                            "'%s' in domain '%s': %s\n", name, type, domain,
                    avahi_strerror(avahi_client_errno(
                                       avahi_service_resolver_get_client(r))));
#endif
            break;
        case AVAHI_RESOLVER_FOUND:
        {
            struct addrinfo *ai;
            struct sockaddr *sa;

            ai = (struct addrinfo*)malloc(sizeof(struct addrinfo));
            sa = (struct sockaddr*)malloc(sizeof(struct sockaddr));

            memset(ai, 0, sizeof(struct addrinfo));
            memset(sa, 0, sizeof(struct sockaddr));

            ai->ai_addr = sa;

            /// @todo (Stuart Baker) should these really be hard coded?
            ai->ai_protocol = IPPROTO_TCP;
            ai->ai_socktype = SOCK_STREAM;

            if (host_name)
            {
                ai->ai_canonname = (char*)malloc(strlen(host_name) + 1);
                strcpy(ai->ai_canonname, host_name);
            }

            switch (protocol)
            {
                case AVAHI_PROTO_INET:
                {
                    struct sockaddr_in *sa_in = (struct sockaddr_in*)sa;
                    ai->ai_family = AF_INET;
                    ai->ai_addrlen = sizeof(struct sockaddr_in);
                    sa_in->sin_family = AF_INET;
                    sa_in->sin_port = htons(port);
                    sa_in->sin_addr.s_addr = address->data.ipv4.address;
                    break;
                }
                case AVAHI_PROTO_INET6:
                {
                    struct sockaddr_in6 *sa_in = (struct sockaddr_in6*)sa;
                    ai->ai_family = AF_INET6;
                    ai->ai_addrlen = sizeof(struct sockaddr_in6);
                    sa_in->sin6_flowinfo = 0;
                    sa_in->sin6_family = AF_INET6;
                    sa_in->sin6_port = htons(port);
                    memcpy(&sa_in->sin6_addr.s6_addr,
                           address->data.ipv6.address,
                           sizeof(address->data.ipv6.address));
                    break;
                }
                default:
                    HASSERT(0);
            }

            HASSERT(lu->hints->ai_family == ai->ai_family);

            if (!lu->addr)
            {
                lu->addr = ai;
            }
            else
            {
                struct addrinfo *current = lu->addr;
                while (current->ai_next)
                {
                    current = current->ai_next;
                }
                current->ai_next = ai;
            }
#if MDNS_DEBUG
            char a[AVAHI_ADDRESS_STR_MAX], *t;
            char nil = '\0';
            fprintf(stderr, "Service '%s' of type '%s' in domain '%s':\n",
                    name, type, domain);
            avahi_address_snprint(a, sizeof(a), address);
            t = txt ? &nil : avahi_string_list_to_string(txt);
            fprintf(stderr,
                    "\t%s:%u (%s)\n"
                    "\tTXT=%s\n"
                    "\tcookie is %u\n"
                    "\tis_local: %i\n"
                    "\tour_own: %i\n"
                    "\twide_area: %i\n"
                    "\tmulticast: %i\n"
                    "\tcached: %i\n",
                    host_name, port, a, t,
                    avahi_string_list_get_service_cookie(txt),
                    !!(flags & AVAHI_LOOKUP_RESULT_LOCAL),
                    !!(flags & AVAHI_LOOKUP_RESULT_OUR_OWN),
                    !!(flags & AVAHI_LOOKUP_RESULT_WIDE_AREA),
                    !!(flags & AVAHI_LOOKUP_RESULT_MULTICAST),
                    !!(flags & AVAHI_LOOKUP_RESULT_CACHED));
            if (t != &nil)
            {
                avahi_free(t);
            }
#endif
        }
    }
    if (--lu->count == 0)
    {
        if (lu->done)
        {
            avahi_simple_poll_quit(lu->sp);
        }
    }
    avahi_service_resolver_free(r);
}

/*
 * MDNS::browse_callback()
 */
void MDNS::browse_callback(AvahiServiceBrowser *b, AvahiIfIndex interface,
                           AvahiProtocol protocol, AvahiBrowserEvent event,
                           const char *name, const char *type,
                           const char *domain,
                           AvahiLookupResultFlags flags, void* userdata)
{
    LookupUserdata *lu = static_cast<LookupUserdata*>(userdata);
    AvahiClient *c = lu->c;

    switch (event)
    {
        case AVAHI_BROWSER_FAILURE:
            break;
        case AVAHI_BROWSER_NEW:
#if MDNS_DEBUG
            fprintf(stderr, "(Browser) NEW: service '%s' of type '%s' "
                            "in domain '%s'\n", name, type, domain);
#endif
            /* We ignore the returned resolver object. In the callback
               function we free it. If the server is terminated before
               the callback function is called the server will free
               the resolver for us. */
            if (!avahi_service_resolver_new(c, interface, protocol, name, type,
                                            domain, AVAHI_PROTO_UNSPEC,
                                            (AvahiLookupFlags)0,
                                            resolve_callback, userdata))
            {
 #if MDNS_DEBUG
                fprintf(stderr, "Failed to resolve service '%s': %s\n",
                        name, avahi_strerror(avahi_client_errno(c)));
#endif
            }
            else
            {
            }
                ++lu->count;
            break;
        case AVAHI_BROWSER_REMOVE:
            break;
        case AVAHI_BROWSER_ALL_FOR_NOW:
        case AVAHI_BROWSER_CACHE_EXHAUSTED:
#if MDNS_DEBUG
            fprintf(stderr, "(Browser) %s\n",
                    event == AVAHI_BROWSER_CACHE_EXHAUSTED ? "CACHE_EXHAUSTED" :
                                                             "ALL_FOR_NOW");
#endif
            lu->done = 1;
            if (lu->count == 0)
            {
                avahi_simple_poll_quit(lu->sp);
            }
            break;
    }
}

/*
 * MDNS::entry_group_callback()
 */
void MDNS::entry_group_callback(AvahiEntryGroup *g, AvahiEntryGroupState state,
                                void *userdata)
{
    MDNS *mdns = static_cast<MDNS *>(userdata);
    mdns->group_ = g;
}

/*
 * MDNS::client_callback()
 */
void MDNS::client_callback(AvahiClient *c, AvahiClientState state,
                           void * userdata)
{
    switch (state)
    {
        default:
            break;
        case AVAHI_CLIENT_S_RUNNING:
            printf("mDNS client running\n");
            break;
    }
}

/*
 * MDNS::entry()
 */
void *MDNS::entry()
{
    int error;

    simplePoll_ = avahi_simple_poll_new();
    HASSERT(simplePoll_);

    client_ = avahi_client_new(avahi_simple_poll_get(simplePoll_),
                               (AvahiClientFlags)0, client_callback, this,
                               &error);
    if (!client_)
    {
#if MDNS_DEBUG

        fprintf(stderr, "Error creating AvaHi client (%d) %s\nmDNS export will "
                        "not be functional.\n", error, avahi_strerror(error));
#endif
        return nullptr;
    }
    printf("mDNS client created\n");
    HASSERT(client_);

    sem_.post();
    avahi_simple_poll_loop(simplePoll_);

    printf("mdns_thread exit\n");

    if (group_)
    {
        avahi_entry_group_reset(group_);
        avahi_entry_group_free(group_);
        group_ = nullptr;
    }

    if (client_)
    {
        avahi_client_free(client_);
        client_ = nullptr;
    }

    avahi_simple_poll_free(simplePoll_);
    simplePoll_ = nullptr;

    sem_.post();
    return nullptr;
}

void MDNS::shutdown()
{
    if (simplePoll_)
    {
        avahi_simple_poll_quit(simplePoll_);
        sem_.wait();
        HASSERT(!simplePoll_);
    }
}

#endif
