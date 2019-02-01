/** \copyright
 * Copyright (c) 2016, Balazs Racz
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
 * \file JSSerialPort.hxx
 *
 * A node.js compatible device talking to a CANUSB adapter via the serial port.
 *
 * @author Balazs Racz
 * @date 22 Jan 2016
 */

#ifndef _UTILS_JSSERIALPORT_HXX_
#define _UTILS_JSSERIALPORT_HXX_

#ifdef __EMSCRIPTEN__

#include <emscripten.h>
#include <emscripten/val.h>

#include "utils/Hub.hxx"
#include "utils/JSHubPort.hxx"

class JSSerialPort
{
public:
    JSSerialPort(CanHubFlow *hflow, string device)
        : canHub_(hflow)
    {
        string script = "Module.serial_device = '" + device + "';\n";
        emscripten_run_script(script.c_str());
        EM_ASM_(
            {
                var serial_module = require('serialport');
                var SerialPort = serial_module.SerialPort;
                var portdev = Module.serial_device;
                console.log('Opening ' + portdev);
                var openerror = function(error) {
                    console.log(
                        'Failed to open serial port ' + portdev + ': ' + error);
                    console.log('Known serial ports:');
                    serial_module.list(function(err, ports)
                        {
                            ports.forEach(function(port)
                                {
                                    console.log('port "' + port.comName +
                                        '" pnp id: ' + port.pnpId +
                                        ' manufacturer: ' + port.manufacturer);
                                });
                        });
                };
                var c;
                try {
                    c = new SerialPort(
                        Module.serial_device, {baudrate : 115200});
                } catch(err) {
                    openerror(err);
                    return;
                }
                c.on("open", function(error) {
                    if (error) {
                        openerror(error);
                        return;
                    }
                    console.log('opened ' + c);
                    var client_port = new Module.JSHubPort(
                        $0, function(data) { c.write(data); });
                    c.on('close', function() {
                        console.log('serial port ' + portdev + ' closed.');
                        client_port.abandon();
                    });
                    c.on('error', function(err) {
                        console.log('error on serial port ' + portdev + ': ' + err);
                        c.close();
                        client_port.abandon();
                    });
                    c.on('data', function(data) { client_port.recv(data.toString()); });
                });
            },
            (unsigned long)canHub_);
    }

    static void list_ports() {
        EM_ASM(
            {
                console.log('Known serial ports:');
                require('serialport').list(function(err, ports)
                    {
                        ports.forEach(function(port)
                            {
                                console.log('port "' + port.comName +
                                    '" pnp id: ' + port.pnpId +
                                    ' manufacturer: ' + port.manufacturer);
                            });
                    });
            });
    }

private:
    CanHubFlow *canHub_;
};

#endif // __EMSCRIPTEN__
#endif // _UTILS_JSTCPCLIENT_HXX_
