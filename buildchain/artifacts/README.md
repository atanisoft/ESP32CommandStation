## Docker-based buildchain

This is a Docker-based buildchain. It allows to build the firmware with the only requirement of having Docker installed.


To start the building procedure, simply run the `build.sh` script:

    ./build.sh 

If no arguments are provided, it will build a firmware for the `esp32` chip type and with the default Command Station settings. The script supports two arguments: the `menuconfig` switch to enable popping up the configuration menu before building, and to set a chip type using the `esp32` or `esp32` switches. For example:

    ./build.sh menuconfig

or

    ./build.sh menuconfig esp32s3


Build artifacts (.e. the fiwmware) are placed in the `artifacts` folder.



A `flash.sh` script is also provided, which requires the `esptool` utility to be installed and that allows to easliy flash the firmware.
