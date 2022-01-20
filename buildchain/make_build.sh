#!/bin/bash
set -e

# Move to source code folder
cd /home/espressif/ESP32CommandStation

# Target if required
if [ "x$1" == "x" ]; then
    echo "Missing target"
    exit 1
else
    TARGET=$1
fi
idf.py set-target $TARGET

# Config if optional
if [ "x$2" == "xmenuconfig" ]; then
    idf.py menuconfig
fi 

# Build
idf.py build

# Copy artifacts
cp build/bootloader/bootloader.bin /artifacts/
cp build/partition_table/partition-table.bin /artifacts/
cp build/ota_data_initial.bin /artifacts/
cp build/ESP32CommandStation.bin /artifacts/
