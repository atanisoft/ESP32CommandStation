/** \copyright
 * Copyright (c) 2014, Balazs Racz
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
 * \file nmranet_constants.cxx
 * Specifies the default values for configuration constants related to NMRAnet.
 *
 * @author Balazs Racz
 * @date 10 Mar 2015
 */

#include "utils/constants.hxx"

/** Number of entries in the remote alias cache */
DEFAULT_CONST(remote_alias_cache_size, 10);

/** Number of entries in the local alias cache */
DEFAULT_CONST(local_alias_cache_size, 3);

/** Maximum number of local nodes */
DEFAULT_CONST(local_nodes_count, 2);

/** Number of datagram registry entries. This is how many datagram handlers can
 * be registered (e.g. memory config protocol is one). */
DEFAULT_CONST(num_datagram_registry_entries, 2);

/** Number of datagram clients. This is how many datagram send operations can
 * happen concurrently. */
DEFAULT_CONST(num_datagram_clients, 2);

/** Maximum number of memory spaces that can be registered for the MemoryConfig
 * datagram handler. */
DEFAULT_CONST(num_memory_spaces, 5);

/** Set to CONSTANT_TRUE if you want to export an "all memory" memory space
 * from the SimpleStack. Note that this should not be enabled in production,
 * because there is no protection against segfaults in it. */
DEFAULT_CONST_FALSE(enable_all_memory_space);

/** Set to CONSTANT_TRUE if you want the nodes to send out producer / consumer
 * identified messages at boot time. This is required by the OpenLCB
 * standard. */
DEFAULT_CONST_TRUE(node_init_identify);
