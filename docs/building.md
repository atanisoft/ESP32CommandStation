# Building the compiled binary

After configuring via `menuconfig` it is necessary to build the source code
into a usable binary for the ESP32. 

## Graphical or command line build

If you are using VSCode with the [Espressif IDF plugin](https://marketplace.visualstudio.com/items?itemName=espressif.esp-idf-extension)
you can simply press the :gear: icon to build the source code.

For command line based building, you will need to prepare your ESP-IDF build
enviornment by running `source {IDF_PATH}/export.sh` if you are on Mac/Linux or
`{IDF_PATH}\export.bat` on Windows, replace `{IDF_PATH}` with the actual path
where ESP-IDF has been installed. This step will need to be done anytime a new
shell is opened the first time for building.

After the shell is configured for ESP-IDF navigate to the ESP32CommandStation
directory and run `idf.py build`.

The build will take between five and ten minutes depending on selected
configuration options and the computer being used to build.

## Flashing binaries to the ESP32

The generated binaries will be in the `build` directory and should be used in
the same manner as pre-compiled binaries as described [here](precompiled.md).

##

[home](README.md)