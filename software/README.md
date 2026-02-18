# RP2040 #

## Table of Contents

1. [Overview](#overview)
2. [Getting Started with RP2040](#start)
3. [Build a new library](#build)
4. [Use an existant library](#use)
5. [Flash firmware](#run)
6. [Code static analysis and formatting check](#checks)


## <a name="overview"/> Overview ##
Library composed by 4 layers:
* Third Party: Dependencies
* Interfaces: Communications
* Drivers: Devices
* Examples: Feature examples

Follow the example:
Read data from sensor X using UART communication with an external library Y.
* Third Party: External library Y
* Interface: TRIBE-RP-DRIVERS UART implementation using external library Y
* Driver: Sensor X functions based on TRIBE-RP-DRIVERS UART to interact with it
* Example: How to implement some feature of sensor X using its driver functions

## <a name="start"/> Getting Started

### Get the SDK to work with RP2040 microcontroller ###
The Raspberry Pi Pico development is fully supported with both C/C++ SDK and an official MicroPython port. <br />
As Pico is built around the RP2040 microcontroller designed by Raspberry Pi, we can use the same environment to build our projects around our own board. <br />

The SDK can be got in https://github.com/raspberrypi/pico-sdk:
```
git clone -b master https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init
```

To build the applications is necessary to install some extra tools:
```
sudo apt update
sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential
```

* Note that Ubuntu and Debian users might additionally need to also install libstdc++-arm-none-eabi-newlib

### Add SDK Path as environment variable ###
Open .bashrc file:

`nano .bashrc`

Add SDK path to the file (define the absolute path in your environment):

`export PICO_SDK_PATH = /.../pico-sdk/`

## <a name="build"/> Create and build a new library ##
Assume, for example, that you want to add an external library to be included in another application layer.
In order to avoid relative and non-intuitive paths inside the repository, we compile the library and install it using cmake configurations. For that, you can follow the steps:

1. In third_party folder, edit the CMakeLists.txt and add 4 different blocks to the new library be recognized and installed in the project.

    **Block 1:** Add source and header files to the library that will be compiled

    ```
    add_library(NNNN_lib
                PATH_TO_SRC/NNNN.c
                PATH_TO_HEADER/NNNN.h
                )
    ```

    **Block 2:** Link the needed dependencies to the library

    ```
    target_link_libraries(NNNN_lib
                          DEPENDENCY_LIB_XXX
                          DEPENDENCY_LIB_YYY
                          )
    ```

    **Block 3:** Install the library in the build workspace

    ```
    install(TARGETS NNNN_lib
            ARCHIVE DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
            LIBRARY DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
            RUNTIME DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin
            )
    ```

    **Block 4:** Install the header file in the build workspace allowing to be included for other files
    ```
    install(DIRECTORY PATH_TO_HEADER/
            DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/include
            )
    ```

2. Once the cmake configurations were done, you can build the whole project.

    In scripts folder, run the build.sh file. It will compile the new configurations and install the new libraries.

    `./build.sh`

     > Do not run the build.sh file with sudo user permissions, unless you have the PICO_SDK_PATH exported and visible to sudo user environment.

If all the steps were well-succeed you must to be able to find:
- /build/
- /build/third-party/NNNN_lib.a
- /build/lib/NNNN_lib.a
- /build/include/NNNN.h

## <a name="use"/> Include a library in a different application layer ##

After installed, a library can be used for another application layer. Assume the new library NNNN_lib installed in third-party folder. To include it in an interfaces layer library, for example, follow the steps:

1. In interfaces folder, edit the CMakeLists.txt and add the NNNN_lib in 2 different blocks.

    **Block 1:** Include the header files in the library
    ```
    target_include_directories(IIII_lib
                               PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../third_party/include)
    ```

    **Block 2:** Link the needed dependencies to the library
    ```
    target_link_libraries(IIII_lib
                          DEPENDENCY_LIB_ZZZ
                          DEPENDENCY_LIB_DDD
                          NNNN_lib)
    ```

2. Once the cmake configurations were done, you can build the whole project. For that, you can follow the steps:

    In scripts folder, run the build.sh file. It will compile the new configurations and install the new libraries.

    `./build.sh`

    > Do not run the build.sh file with sudo user permissions, unless you have the PICO_SDK_PATH exported and visible to sudo user environment.

    > This was just an example however you can follow it to all application layers. Pay attention to the blocks already implemented in the CMakeLists and follow the same structure.


## <a name="run"/> Flash the built code to microcontroller RP2040 ##
1. Change the device to USB Mass Storage mode <br />

    **Option 1:** Using RPBOOT (SW1 button) and RUN (SW2 button) pins <br />
    - Hold RPBOOT button/pin down and after that pull down RUN pin<br /><br />

    **Option 2:** Using the command line<br />
    - `sudo picotool reboot -f -u`

2. Flash the built code on RP2040 <br />

    **Option 1:** Using command line
    * Mount the device on /mnt/pico directory and copy to there the code.uf2 (generated on compilation process and located on build folder)
    * The device is automatically unmounted, however to ensure everything is processed successfully, manually unmounted

      ```
      sudo mount /dev/sdb1 /mnt/pico
      sudo cp code.uf2 /mnt/pico
      sudo sync
      sudo umount /mnt/pico
      ```

     **Option 2:** Manually
     * Copy the file code.uf2 directly to RP2040 storage folder

## <a name="checks"/> Code static analysis and code formatting verification ##

Before committing any code to the repository, you can check the code format and perform a static analysis using the scripts available in the scripts folder.

### Code format ###

To check the code formatting without changing anything:

`./format_code --check`

To check the code formatting and apply the necessary changes:

`./format_code`


### Static analysis ###

To run static analysis with all types of logging enabled:

`./static_analysis.sh`

To run static analysis quietly (without logging):

`./static_analysis.sh --quiet`

## TIPS ##

> After flash the built code, RP2040 will leave the USB mass storage mode and change to RUN mode. Here, the device /dev/ttyACM0 (or ACMX) will be visible. If not, something failed initializing USB communication or running the code.

> When pico-sdk repository is a submodule of a project, besides to init and update it, you have to do the same for the tinyusb module, located in /pico-sdk/lib/tinyusb/. If you don't, the cmake process will warn you about that. The code will compile but after it will result on usb port loss (/dev/ttyACM0 won't appear after copy the uf2 file to RP2040)

> Forceinline error (duplicate 'inline') appears. It's mentioned That bug was fixed and merged on develop branch.
When tested that branch, the compiling error remains.
https://github.com/raspberrypi/pico-sdk/pull/670
