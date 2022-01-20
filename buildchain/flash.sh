#!/bin/bash

# Port is required
if [ "x$1" == "x" ]; then
    echo "Usage: flash.sh serial_port [chip_type]"
else
    SERIAL_PORT=$1
fi 

# Set file locations
BOOTLOADER_FILE=artifacts/bootloader.bin
PARTITION_TABLE_FILE=artifacts/partition-table.bin
DATA_INITIAL_FILE=artifacts/ota_data_initial.bin
FIRMWARE_FILE=artifacts/ESP32CommandStation.bin

# Set chip type
echo ""
if [ "x$2" == "x" ]; then
    CHIP_TYPE=esp32
    echo "Using default chip type: esp32"
else
    CHIP_TYPE=$2
    echo "Using chip type: $CHIP_TYPE"
fi
echo ""

# Flash
esptool.py -p $SERIAL_PORT -b 460800 --before default_reset --after hard_reset --chip $CHIP_TYPE  write_flash --flash_mode dio --flash_size detect --flash_freq 40m 0x1000 $BOOTLOADER_FILE 0x8000 $PARTITION_TABLE_FILE 0xe000 $DATA_INITIAL_FILE 0x10000 $FIRMWARE_FILE
