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
 * \file JSWebsocketClient.hxx
 *
 * A node.js compatible server component that exports a TCP listening server
 * with a GridConnect hub.
 *
 * @author Balazs Racz
 * @date 13 Sep 2015
 */

#ifndef _UTILS_JSWEBSOCKETCLIENT_HXX_
#define _UTILS_JSWEBSOCKETCLIENT_HXX_

#ifdef __EMSCRIPTEN__

#include <emscripten.h>
#include <emscripten/val.h>

#include "utils/Hub.hxx"
#include "utils/JSHubPort.hxx"

class JSWebsocketClient
{
public:
    JSWebsocketClient(CanHubFlow* hflow, string server_js)
        : canHub_(hflow) {
        string script = "Module.ws_server = " + server_js + ";\n";
        emscripten_run_script(script.c_str());
        EM_ASM_(
            {
                var server_address = Module.ws_server;
                try {
                    var WS = window.WebSocket || window.MozWebSocket;
                } catch (err) {
                    var WS = require('websocket').w3cwebsocket;
                }
                /*var is_node = window ? false : true;
                if (!is_node) {
                    console.log('using browser');
                } else {
                    console.log('using nodejs');
                    }*/
                var reconnect;
                reconnect = function() {
                console.log('NNNN connecting to ws server: ', server_address);
                var connection = new WS(server_address);
                var portnum;
                var client_port = new Module.JSHubPort($0,
                            function(gc_text)
                            {
                                var json = JSON.stringify(
                                    {type : 'gc_can_frame', data : gc_text});
                                if (connection.readyState == 1) {
                                    connection.send(json);
                                } else {
                                    console.log('Not sending frame ', gc_text,
                                                ' because connection',portnum,'state = ',
                                                connection.readyState);
                                }
                            });
                portnum = client_port.get_port_num();
                connection.onopen = function() {
                    console.log('ws connection',portnum,'established. starting stack.');
                    try {
                        Module.startStack();
                    } catch (e) {
                        if (e === 'SimulateInfiniteLoop') {
                            console.log('stack started.');
                        } else {
                            console.log('unknown exception ', e);
                            throw e;
                        }
                    }
                };
                connection.onerror = function (eee) {
                    console.log('Connection',portnum,'error: ', eee);
                    //client_port.abandon();
                    //connection.close();
                    //setTimeout(reconnect, 1000);
                };
                connection.onclose = function () {
                    console.log('Websocket',portnum,'closed.');
                    client_port.abandon();
                    connection.close();
                    delete connection;
                    console.log('Scheduling reconnect.');
                    setTimeout(reconnect, 1000);
                };
                connection.onmessage = function(message) {
                    try
                    {
                        var json = JSON.parse(message.data);
                    }
                    catch (e)
                    {
                        console.log(
                            'This doesnt look like a valid JSON: ',
                            message.data);
                        return;
                    }
                    if (json.type === 'gc_can_frame')
                    {
                        // Send can frame data to the hub port
                        client_port.recv(json.data);
                    } else {
                        console.log('Received data of unknown type: ',
                                    json.type);
                    }
                };
                };
                reconnect();
            }, (unsigned long)canHub_);
    }

private:
    CanHubFlow *canHub_;
};

#endif // __EMSCRIPTEN__
#endif // _UTILS_JSWEBSOCKETCLIENT_HXX_
