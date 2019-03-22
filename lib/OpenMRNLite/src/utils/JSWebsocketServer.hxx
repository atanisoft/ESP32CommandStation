/** \copyright
 * Copyright (c) 2019, Balazs Racz
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
 * \file JSWebsocketServer.hxx
 *
 * Javascript hub server that exports an HTTP server with a websocket module
 * that allows HTML5 clients to connect via websocket protocol and be part
 * ofthe CAN network.
 *
 * @author Balazs Racz
 * @date 19 Feb 2019
 */

#include <emscripten.h>
#include <emscripten/bind.h>
#include <emscripten/val.h>

#include <memory>

#include "utils/GridConnectHub.hxx"

class JSWebsocketServer
{
public:
    JSWebsocketServer(CanHubFlow *hflow, int port, string static_dir)
        : canHub_(hflow)
    {
        if (!static_dir.empty())
        {
            string script = "Module.static_dir = '" + static_dir + "';\n";
            emscripten_run_script(script.c_str());
        }
        EM_ASM_(
            {
                var WebSocketServer = require('websocket').server;
                var http = require('http');
                var ecstatic = require('ecstatic');
                if (Module.static_dir)
                {
                    var serverImpl =
                        ecstatic({root : Module.static_dir, gzip : true});
                }
                else
                {
                    var serverImpl = function(request, response){
                        // process HTTP request. Since we're writing just
                        // WebSockets server we don't have to implement
                        // anything.
                    };
                }
                var server = http.createServer(serverImpl);
                console.log('try to listen on ', $0);
                server.listen($0, function()
                    {
                        console.log(
                            'websocket server: listening on port ' + $0);
                    });
                console.log('ws: listen done ', $0);

                // create the server
                wsServer = new WebSocketServer({httpServer : server});

                // WebSocket server
                wsServer.on('request', function(request)
                    {
                        var connection = request.accept(null, request.origin);
                        var client_port = new Module.JSHubPort($1,
                            function(gc_text)
                            {
                                var json = JSON.stringify(
                                    {type : 'gc_can_frame', data : gc_text});
                                connection.sendUTF(json);
                            });
                        console.log('websocket client ',
                            client_port.get_port_num(), 'connected');
                        connection.on('message', function(message)
                            {
                                try
                                {
                                    var json = JSON.parse(message.utf8Data);
                                }
                                catch (e)
                                {
                                    console.log(
                                        'This doesnt look like a valid JSON: ',
                                        message.data, ' raw msg ', message);
                                    return;
                                }
                                if (json.type === 'gc_can_frame')
                                {
                                    // Send can frame data to the hub port
                                    client_port.recv(json.data);
                                }
                                else
                                {
                                    console.log('Unknown type ', message.type);
                                }
                            });
                        connection.on('close', function(connection)
                            {
                                console.log('websocket client ',
                                    client_port.get_port_num(), 'disconnected');
                                client_port.pause();
                            });
                        connection.on('resume', function(connection)
                            {
                                console.log('websocket client ',
                                    client_port.get_port_num(), ' resumed');
                                client_port.resume();
                            });
                        connection.on('pause', function(connection)
                            {
                                console.log('websocket client ',
                                    client_port.get_port_num(), ' paused');
                                client_port.pause();
                            });
                        var ignevent = function(evname)
                        {
                            connection.on(evname, function(connection)
                                {
                                    console.log(
                                        'websocket ingoring event ', evname);
                                });
                        };
                        ignevent('drain');
                    });
            },
            port, (unsigned long)canHub_);
    }

private:
    CanHubFlow *canHub_;
};
