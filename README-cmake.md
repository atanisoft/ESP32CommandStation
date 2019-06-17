# ESP32CommandStation Cmake project

This directory includes the CMake build system as an alternative to platformio.
See the general setup/usage instructions 
 https://docs.espressif.com/projects/esp-idf/en/latest/get-started-cmake/index.html
 
 
# Setup

This repo includes submodules. Make sure to pull/update recursively.
```
git submodule update --init --recursive
```

# Build
On the command line within the `<ROOT>/cmake` directory, type 
```
idf.py menuconfig
```
this will start the build process and open `menuconfig` which allows us to customize the build options.
![DCC Menuconfig](./cmake/images/menuconfig1.png?raw=true) 
Select the 'DCC/LCC...' entry and then the 'WiFi' settings
![Wifi](./cmake/images/menuconfig2.png?raw=true)
Other settings
![Other](./cmake/images/menuconfig3.png?raw=true) 
At the end save the settings and exit `idf.py menuconfig`. The build process will then generate the `sdkconfig.h` which is included during compilation. Start compiling and linking with:


```
idf.py build
```
After this you can flash the image with 
```
idf.py -p <port> flash
``` 
and


# VisualGDB

On windows there is a Microsoft VisualStudio plugin called VisualGDB https://visualgdb.com/ which allows developing code for the ESP32 platform. Open the project file

```
<ROOT>/cmake/ESP32CommandStation.vgdbproj
```
with VisualStudio and you will be able to compile/link/debug. This was tested on the Wrover ESP32 development platform. There you get a a nice graphical interface as a `menuconfig` replacement - still using the cmake project files.
![VisualGDB](./cmake/images/visgdb1.png?raw=true)
The plugin works with gdb in the background.
![Debugger](./cmake/images/visgdb2.png?raw=true)

