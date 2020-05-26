/** @copyright
 * Copyright (c) 2017, Stuart W Baker
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
 * @file WifiDefs.cxx
 * This file provides weak (reference) definitions for the Wi-Fi credentials.
 *
 * @author Stuart W Baker
 * @date 3 February 2017
 */

// The following defaults are a template for what needs to be defined in a
// given application for Wi-Fi to work.  Do not modify this file.  Copy these
// globals into an application specific file and remove the weak attribute.
// They are defined here weak so that a build can complete without failure
// but are not expected to work as is.

extern "C" {
/// Name of wifi accesspoint to connect to.
char __attribute__((weak)) WIFI_SSID[] = "YourSSIDHere";
/// Password of wifi connection. If empty, use no encryption.
char __attribute__((weak)) WIFI_PASS[] = "pass-or-empty-for-open";
/// Hostname at which the OpenLCB hub is at.
char __attribute__((weak)) WIFI_HUB_HOSTNAME[] = "10.0.0.7";
/// Port number of the OpenLCB hub.
int __attribute__((weak)) WIFI_HUB_PORT = 12021;
}

