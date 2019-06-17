#######################################################################
# ESP32 COMMAND STATION
#
# COPYRIGHT (c) 2017-2019 Mike Dunston
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see http://www.gnu.org/licenses
#######################################################################

Import("env")
import gzip
import os
import struct
import cStringIO

from file2header import convert

def build_index_html_h(source, target, env):
    inputfile = '%s/data/index.html' % env.subst('$PROJECT_DIR')
    outputfile = '%s/include/index_html.h' % env.subst('$PROJECT_DIR')
    #print "Converting %s\n\t -> %s" % (inputfile, outputfile)
    if os.path.exists(inputfile):
        if os.path.getmtime(inputfile) < os.path.getmtime(outputfile):
            return
    convert(inputfile, outputfile)

env.AddPreAction('$BUILD_DIR/src/Interfaces/WebServer.cpp.o', build_index_html_h)
