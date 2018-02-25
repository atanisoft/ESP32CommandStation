Import("env")
import gzip
import os
import struct
import cStringIO

def build_index_html_h(source, target, env):
    print "Attempting to compress %s/data/index.html" % env.subst('$PROJECT_DIR')
    gzFile = cStringIO.StringIO()
    with open('%s/data/index.html' % env.subst('$PROJECT_DIR')) as f, gzip.GzipFile(mode='wb', fileobj=gzFile) as gz:
        gz.writelines(f)
    gzFile.seek(0, os.SEEK_END)
    gzLen = gzFile.tell()
    gzFile.seek(0, os.SEEK_SET)
    print 'Compressed index.html.gz file is %d bytes' % gzLen
    with open('%s/index_html.h' % env.subst('$BUILD_DIR'), 'w') as f:
        f.write("#ifndef _INDEX_HTML_GZ_H_\n#define _INDEX_HTML_GZ_H_\n")
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
        f.write("#endif\n")

env.AddPreAction('$BUILD_DIR/src/WebServer.cpp.o', build_index_html_h)
env.Append(
    CPPPATH = [
        "$BUILD_DIR"
    ]
)
