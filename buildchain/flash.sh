#!/bin/bash

# Port is required
if [ "x$1" == "x" ]; then
    echo "Usage: flash.sh serial_port"
fi 

# Set file locations
BOOTLOADER_FILE=artifacts/bootloader.bin
PARTITION_TABLE_FILE=artifacts/partition-table.bin
DATA_INITIAL_FILE=artifacts/ota_data_initial.bin
FIRMWARE_FILE=artifacts/ESP32CommandStation.bin

# Flash
esptool.py -p $1 -b 460800 --before default_reset --after hard_reset --chip esp32  write_flash --flash_mode dio --flash_size detect --flash_freq 40m 0x1000 $BOOTLOADER_FILE 0x8000 $PARTITION_TABLE_FILE 0xe000 $DATA_INITIAL_FILE 0x10000 $FIRMWARE_FILE
