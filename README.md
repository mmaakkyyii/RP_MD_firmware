# RPMD firmware
RPMD is motor driver using RP2040.

https://github.com/mmaakkyyii/RP_MD

# Folder structure
```
raspi_pico  
    ├─RP_MD_firmware  
    │   ├─build  
    │   │   ├─hogehoge  
    │   │   └─hello_world.uf2  
    │   ├─CMakeLists.txt  
    │   ├─RPMD.cpp  
    │   └─pico_sdk_import.cmake  
    └─pico-sdk  
        └─hogehoge  
```
# Build
```
cd RP_MD_firmware
mkdir build
cd build
cmake .. -DPICO_SDK_PATH=/mnt/d/raspi_pico/pico-sdk
make RPMD
```
