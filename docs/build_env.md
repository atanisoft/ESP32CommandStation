# Configuring the build environment

The ESP32 Command Station code is built using the ESP-IDF framework.

## Setting up ESP-IDF build environment

ESP-IDF is a required component and unfortunately there are only a few options
for setting up the build environment. For all options it is required to have
Python3 and the Git client installed and available in the path.

### VSCode "Espressif IDF" extension

The [Espressif IDF plugin](https://marketplace.visualstudio.com/items?itemName=espressif.esp-idf-extension)
provides a relatively easy way to install all components and configure them for
building. There are a few bugs in the plugin still but overall it will work.

Once installed proceed to [Configuring build options](configuring.md).

### Command line building on Windows

The easiest way to install the dependencies is to use the
[ESP-IDF Windows Installer](https://dl.espressif.com/dl/esp-idf/?idf=4.4).
This utility will download and configure all tools automatically for you. When
prompted it is recommended to install ESP-IDF v4.3 or v4.4.

Once completed proceed to [Configuring build options](configuring.md).

### Command line building on Linux

Many Linux distributions include all necessary tools by default. In some cases
it will be necessary to install additional tools. Please use
[this](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/linux-setup.html)
guide to install required software updates.

### Command line building on MacOS

MacOS includes most of the required tools, please use
[this](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/macos-setup.html)
guide to install required software updates.

## Obtaining ESP-IDF

ESP32 Command Station supports ESP-IDF v4.3 (or later). For Linux/MacOS the
ESP-IDF framework should be downloaded via Git using the commands below:

```
mkdir -p ~/esp
cd ~/esp
git clone --recursive https://github.com/espressif/esp-idf.git --branch release/v4.3
```

If you prefer to use ESP-IDF v4.4 adjust the branch name to `release/v4.4`
instead.

## Configuring ESP-IDF Environment

Once ESP-IDF has been downloaded it needs to be installed using the following
command:

```
cd ~/esp
sh install.sh
```

This will install and configure a python virtual environment which is used for
all build related activities. You will need to run `source ~/esp/export.sh`
from any shell which is used for building ESP-IDF based projects.

## What's next?

Proceed to [Configuring build options](configuring.md).

[home](README.md)