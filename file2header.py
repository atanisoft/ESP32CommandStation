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

import sys
import os
import gzip
import struct
import cStringIO


def convert(inputfile, outputfile):
    ''' 
    convert and compress HTML input file into C include file
    '''
    print "Attempting to compress %s" % inputfile
    gzFile = cStringIO.StringIO()
    with open(inputfile) as f, gzip.GzipFile(mode='wb', fileobj=gzFile) as gz:
        gz.writelines(f)
    gzFile.seek(0, os.SEEK_END)
    gzLen = gzFile.tell()
    gzFile.seek(0, os.SEEK_SET)
    print 'Compressed %s file is %d bytes' % (outputfile, gzLen)
    with open(outputfile, 'w') as f:
        f.write("#pragma once\n")
        f.write("const size_t indexHtmlGz_size = {};\n".format(gzLen))
        f.write("const uint8_t indexHtmlGz[] PROGMEM = {\n");
        while True:
            block = gzFile.read(16)
            if len(block) < 16:
                if len(block):
                    f.write("\t")
                    for b in block:
                        # Python 2/3 compat
                        if type(b) is str:
                            b = ord(b)
                        f.write("0x{:02X}, ".format(b))
                    f.write("\n")
                break
            f.write("\t0x{:02X}, 0x{:02X}, 0x{:02X}, 0x{:02X}, "
                    "0x{:02X}, 0x{:02X}, 0x{:02X}, 0x{:02X}, "
                    "0x{:02X}, 0x{:02X}, 0x{:02X}, 0x{:02X}, "
                    "0x{:02X}, 0x{:02X}, 0x{:02X}, 0x{:02X},\n"
                    .format(*struct.unpack("BBBBBBBBBBBBBBBB", block)))
        f.write("};\n")
