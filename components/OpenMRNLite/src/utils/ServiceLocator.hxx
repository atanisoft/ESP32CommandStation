/** \copyright
 * Copyright (c) 2019, John Socha-Leialoha
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
 * \file ServiceLocator.hxx
 * This file implements a very simple service locator where you can register
 * and retrieve pointers by name.
 *
 * @author John Socha-Leialoha
 * @date 15 September 2019
 */

#ifndef _UTILS_SERVICELOCATOR_HXX_
#define _UTILS_SERVICELOCATOR_HXX_

#include <map>
#include <memory>
#include <string>
#include "utils/Atomic.hxx"

using std::shared_ptr;
using std::string;

/**
 * The sole purpose of this templated class is to provide access to a string
 * that is specific to a type. We use this as a key in the dictionary that
 * maps "type" names into pointers.
 */
template <typename ServiceType>
class RegisterableService
{
private:
    /**
     * Get a name that has the derived type as part of the name
     */
    static string get_type_name(const char *alias)
    {
        string name(__PRETTY_FUNCTION__);
        if (alias != nullptr)
        {
            name += "_";
            name += alias;
        }

        return name;
    }

    template<typename LocatorType> friend class ServiceLocator;
};

/**
 * Provides the shared storage for registrations
 */
class ServiceLocatorImpl
{
public:
    /**
     * Remove all of the registerations for all types. This is mainly useful
     * for tests.
     */
    static void clear()
    {
        ServiceLocatorImpl::services.clear();
    }

private:
    static std::map<string, shared_ptr<void>> services;
    static Atomic lock_;

    /**
     * Register a pointer with a name. The creator of the class still owns the
     * instance.
     * @param name of the service that you want to register
     * @param service is a pointer that you want to register
     */
    static void register_service(string name, shared_ptr<void> &service)
    {
        AtomicHolder l(&ServiceLocatorImpl::lock_);
        ServiceLocatorImpl::services[name] = service;
    }

    /**
     * Retrieves a pointer to the registered service, by name.
     * @param name of the service to retrieve
     * @return the retrieved pointer, which is not guaranteed to be of the
     * requested type
     */
    static shared_ptr<void> get_service(string name)
    {
        AtomicHolder l(&ServiceLocatorImpl::lock_);
        auto it = ServiceLocatorImpl::services.find(name);
        if (it == ServiceLocatorImpl::services.end())
        {
            return nullptr;
        }

        return it->second;
    }

    template<typename ServiceType> friend class ServiceLocator;
};

template <typename ServiceType>
class ServiceLocator
{
public:
    /**
     * Get the service that has been registereed for this type
     * @param alias allows you to retrieve an instance with a specific name
     * @return the service, or nullptr if no such service
     */
    static shared_ptr<ServiceType> get_service(const char *alias = nullptr)
    {
        string name = RegisterableService<ServiceType>::get_type_name(alias);
        shared_ptr<void> service = ServiceLocatorImpl::get_service(name);
        return std::static_pointer_cast<ServiceType>(service);
    }

    /**
     * Register a service instance with the service locator. Because this is
     * just registering the pointer, the creator still owns the lifetime.
     * @param service instance that you want to register
     * @param alias allows you to register a mapping with a specific name
     */
    static void register_service(
        shared_ptr<ServiceType> &service, const char *alias = nullptr)
    {
        string name = RegisterableService<ServiceType>::get_type_name(alias);
        shared_ptr<void> temp(service);
        ServiceLocatorImpl::register_service(name, temp);
    }
};

#endif // _UTILS_SERVICELOCATOR_HXX_

