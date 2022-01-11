#!/bin/bash
set -e

# Move to source code folder
cd /home/espressif/ESP32CommandStation

# Config if required
if [ "x$1" == "xmenuconfig" ]; then
    idf.py menuconfig
fi 

# Build
idf.py build

# Copy artifacts
cp build/bootloader/bootloader.bin /artifacts/
cp build/partition_table/partition-table.bin /artifacts/
cp build/ota_data_initial.bin /artifacts/
cp build/ESP32CommandStation.bin /artifacts/
