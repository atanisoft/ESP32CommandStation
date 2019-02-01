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
 * \file DefaultCdi.cxx
 *
 * Weak definitions of the CDI symbols for SimpleStack. These must be in a
 * different compilation unit to avoid gcc optimizing away the weak linker
 * symbol dereference.
 *
 * @author Balazs Racz
 * @date 18 Jun 2015
 */

#include <unistd.h>
#include <stdint.h>

namespace openlcb {

extern const uint16_t __attribute__((weak)) CDI_EVENT_OFFSETS[] = {0};

extern const char __attribute__((weak)) CDI_DATA[] =
R"cdi(<?xml version="1.0"?>
<cdi xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://openlcb.org/schema/cdi/1/1/cdi.xsd">
<identification/>
<acdi/>
<segment origin='0' space='252'>
 <group>
 <name>Manufacturer Information</name>
 <description>Manufacturer-provided fixed node description</description>
 <int size='1'>
 <name>Version</name>
 </int>
 <string size='41'>
 <name>Manufacturer Name</name>
 </string>
 <string size='41'>
 <name>Node Type</name>
 </string>
 <string size='21'>
 <name>Hardware Version</name>
 </string>
 <string size='21'>
 <name>Software Version</name>
 </string>
 </group>
</segment>
<segment origin='0' space='251'>
 <group>
 <name>User Identification</name>
 <description>Lets the user add his own description</description>
 <int size='1'>
 <name>Version</name>
 </int>
 <string size='63'>
 <name>Node Name</name>
 </string>
 <string size='64'>
 <name>Node Description</name>
 </string>
 </group>
</segment>
</cdi>
)cdi";


} // namespace openlcb
