#!/bin/bash
set -e

# Startup message
echo ""
if [ "x$1" == "xmenuconfig" ]; then
    echo "Building with custom configuration."
else
    echo "Building with default configuration. To change it, run this script with the 'menuconfig' option (i.e. ./build.sh menuconfig)"
fi
echo ""

# Move to parent fir in order to allow copying the codebase inside the container
cd ..

# build the container
docker build . -f buildchain/Dockerfile -t esp32cs/buildchain

# Build and copy artifacts to the "artifacts" folder
docker run -v $PWD/buildchain/artifacts:/artifacts -it esp32cs/buildchain /home/espressif/make_build.sh $1

echo ""
echo "Build complete. To flash, discard the prevoius message and instead use: ./flash.sh serial_port"
echo ""
