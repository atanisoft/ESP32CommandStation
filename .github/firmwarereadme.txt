Using this binary firmware bundle
---------------------------------

This firmware bundle contains a pre-compiled and pre-configured version of the
ESP32CommandStation code.

There are currently four ways to utilize this firmware bundle:

1. esptool.
2. Espressif Flash Download Tool.
3. JMRI Firmware Update utility.
4. Firmware Update via built-in web interface.

esptool
-------

esptool is a command line utility for flashing binary firmware to various ESP32
devices. This is the preferred method for initial firmware flashing.

The command below can be used to flash the firmware to the ESP32:

esptool.py --port /dev/ttyUSB0 --before=default_reset --after=hard_reset
    --baud 460800 write_flash 0x1000 bootloader.bin 0x8000 partition-table.bin
    0xe000 ota_data_initial.bin 0x10000 ESP32CommandStation.bin

Note: The command above should be a single line.

Note: For the ESP32-S3 the bootloader.bin should be flashed at 0x0000 instead
of 0x1000 as listed above.

The port parameter should be adjusted to your environment, on Windows this is
usually prefixed by COM rather. On Linux it will be similar to the provided
value. On Mac the ESP32 may show up as /dev/cu.SLAB_USBtoUART and you may also
require additional drivers.

If you need/want to erase the flash on the ESP32 the following command can be
used:

esptool.py --port /dev/ttyUSB0 erase_flash

Note: The port parameter will need to be adjusted similar to the flashing
command above.

Espressif Flash Download Tool
-----------------------------

The Espressif Flash Download Tool is a graphical Windows only utility that
operates similar to esptool. This tool can be downloaded via the link below:
https://www.espressif.com/en/support/download/other-tools

Use the following offsets and binaries for this utility:

0x8000 partition-table.bin
0xe000 ota_data_initial.bin
0x1000 bootloader.bin
0x10000 ESP32CommandStation.bin

JMRI Firmware Update Utility
----------------------------

JMRI has built-in support for uploading firmware to OpenLCB (LCC) connected
devices. At this time ESP32 Command Station only supports this feature when
using a CAN (TWAI) physical connection between the ESP32 Command Station and
the computer running JMRI. This approach can not be used as the initial
firmware upload to the ESP32.

When using JMRI to upload the firmware only ESP32CommandStation.bin should be
used for upload.


Firmware Update via built-in web interface
------------------------------------------

After the initial firmware flashing has been completed it is possible to use
the built-in web interface to upload new firmware versions to the ESP32 Command
Station. Using this interface only ESP32CommandStation.bin should be uploaded.