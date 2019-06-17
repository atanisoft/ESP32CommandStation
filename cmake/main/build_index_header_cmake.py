#######################################################################
# DCC COMMAND STATION FOR ESP32
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
import getopt

# add the parrent directory to the search path...
THIS_SRCFILE = os.path.abspath(os.path.expanduser(__file__))
if os.path.islink(THIS_SRCFILE):
    THIS_SRCFILE = os.readlink(THIS_SRCFILE)
path = os.path.dirname(THIS_SRCFILE) + "/../.."
print "PATH is %s" % path
sys.path.insert(0, path)

from file2header import convert


def main(argv):
    inputfile = '../../data/index.html'
    outputfile= '../../include/index_html1.h'
    if len(argv) > 0:
        try:
            opts, args = getopt.getopt(argv, "hi:o:", ["ifile=","ofile="])
        except getopt.GetoptError:
            print 'build_index_header_cmake.py -i <inputfile> -o <outputfile>'
            sys.exit(2)
        for opt, arg in opts:
            if opt == '-h':
                print 'build_index_header_cmake.py -i <inputfile> -o <outputfile>'
                sys.exit(3)
            elif opt in ("-i", "--ifile"):
                inputfile = arg
            elif opt in ("-o", "--ofile"):
                outputfile = arg
    if not os.path.exists(inputfile):
        print "could not find intput file: %s" % inputfile
        sys.exit(4)

    convert(inputfile, outputfile)



if __name__ == "__main__":
    main(sys.argv[1:])
