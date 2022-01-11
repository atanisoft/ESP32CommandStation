#!/bin/bash
set -e

# Set chip type
if [ "x$1" == "xesp32s3" ] || [ "x$2" == "xesp32s3" ]; then
    CHIP_TYPE=esp32s3
else
    CHIP_TYPE=esp32
fi 

# Startup message
echo ""
if [ "x$1" == "xmenuconfig" ] || [ "x$2" == "xmenuconfig" ]; then
    echo "Building with custom configuration for chip $CHIP_TYPE."
    CONFIG=menuconfig
else
    echo "Building with default configuration $CHIP_TYPE.. To change it, run this script with the 'menuconfig' option (i.e. ./build.sh menuconfig)"
    CONFIG=
fi
echo ""

# Move to parent dir in order to allow copying the codebase inside the container
cd ..

# build the container
docker build . -f buildchain/Dockerfile -t esp32cs/buildchain

# Build and copy artifacts to the "artifacts" folder
docker run -v $PWD/buildchain/artifacts:/artifacts -it esp32cs/buildchain /home/espressif/make_build.sh $CHIP_TYPE $CONFIG

echo ""
echo "Build complete. To flash, discard the prevoius message and instead use: ./flash.sh serial_port"
echo ""
