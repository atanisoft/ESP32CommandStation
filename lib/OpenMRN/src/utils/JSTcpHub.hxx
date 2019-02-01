/** \copyright
 * Copyright (c) 2015, Balazs Racz
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
 * \file JSTcpHub.hxx
 *
 * A node.js compatible server component that exports a TCP listening server
 * with a GridConnect hub.
 *
 * @author Balazs Racz
 * @date 13 Sep 2015
 */

#ifndef _UTILS_JSTCPHUB_HXX_
#define _UTILS_JSTCPHUB_HXX_

#ifdef __EMSCRIPTEN__

#include <emscripten.h>
#include <emscripten/val.h>

#include "utils/Hub.hxx"
#include "utils/JSHubPort.hxx"

class JSTcpHub
{
public:
    JSTcpHub(CanHubFlow *hflow, int port)
        : canHub_(hflow)
    {
        EM_ASM_(
            {
                var net = require('net');
                var server = net.createServer(function(c)
                    {
                        console.log('client connected');
                        c.setEncoding('utf-8');
                        c.setTimeout(0);
                        c.setKeepAlive(true);
                        var client_port =
                            new Module.JSHubPort($1, function(data)
                                {
                                    c.write(data);
                                });
                        c.on('close', function()
                            {
                                console.log('client disconnected');
                                client_port.abandon();
                            });
                        c.on('error', function()
                            {
                                console.log('client error -- disconnected');
                                client_port.abandon();
                            });
                        c.on('data', function(data)
                            {
                                client_port.recv(data);
                            });
                    });
                server.listen($0, function()
                    {
                        console.log('listening on port ' + $0);
                    });
            },
            port, (unsigned long)canHub_);
    }

private:
    CanHubFlow *canHub_;
};

#endif // __EMSCRIPTEN__
#endif // _UTILS_JSTCPHUB_HXX_
