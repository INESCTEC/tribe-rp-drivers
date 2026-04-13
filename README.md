<div align="center">
  <p align="center">
    <img src="./docs/INESCTEC_MAIN.png" alt="Logotipo Instituição A" width="200"/>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; <!-- Espaço entre as imagens -->
    <img src="./docs/TRIBE_MAIN.png" alt="Logotipo Instituição B" width="200"/>
  </p>
</div>

# TRIBE-RP-DRIVERS

## Description
This repository, developed under the [TRIBE Laboratory](https://www.inesctec.pt/en/laboratories/tribe-laboratory-of-robotics-and-iot-for-smart-precision-agriculture-and-forestry) from [INESC TEC](https://www.inesctec.pt), provides libraries and tools for working with RP2040. Hardware and firmware material for complete integration of the microcontroller are available, allowing to standardize its usage alongside different projects and needs.

In terms of hardware, if you need to integrate RP2040 into your PCB, you can contact the [contributors](#contributors). <br/>

In terms of software/firmware, there are libraries for working on the interface/communications layer and drivers for sensing and actuating devices. <br/>
You can find documentation about its usage in the following link: https://github.com/INESCTEC/tribe-rp-drivers/blob/main/software/README.md

___

## Integrate with your tools
Follow the folder structure for your project to easily standardize the integrations inside the teamwork:

PROJECT_NAME <br />
&nbsp;&nbsp;&nbsp;&nbsp;| <br />
&nbsp;&nbsp;&nbsp;&nbsp;|__ extern <br />
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|__ tribe-rp-drivers <br />
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|__ ... <br />
&nbsp;&nbsp;&nbsp;&nbsp;|__ software <br />
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|__ src <br />
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|__ include <br />
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|__ CMakeLists.txt <br />
&nbsp;&nbsp;&nbsp;&nbsp;|__ hardware <br />

To use this repository in your project, add it as a submodule of the repository that you’re working on, inside extern folder.

> `git submodule add https://github.com/INESCTEC/tribe-rp-drivers`

Some submodules have to be initialized and updated. Use the following command inside the root folder.

> `git submodule update --init --recursive`


In PROJECT_NAME/software/CMakeLists.txt follow the example:

```
cmake_minimum_required(VERSION 3.16)

include(pico_sdk_import.cmake)

project(PROJECT_NAME C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

# Find linker paths
link_directories(${CMAKE_CURRENT_BINARY_DIR}/../../extern/tribe-rp-drivers/software/build/drivers/)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/../../extern/tribe-rp-drivers/software/build/interfaces/)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/../../extern/tribe-rp-drivers/software/build/third-party/)

# Find include paths
include_directories(PROJECT_NAME PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../../extern/tribe-rp-drivers/software/build/drivers/include/)
include_directories(PROJECT_NAME PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../../extern/tribe-rp-drivers/software/build/interfaces/include/)
include_directories(PROJECT_NAME PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../../extern/tribe-rp-drivers/software/build/third_party/include/)

add_executable(PROJECT_EXEC_NAME
                src/file_name.cpp)

pico_enable_stdio_usb(PROJECT_EXEC_NAME 1)
pico_enable_stdio_uart(PROJECT_EXEC_NAME 0)

pico_add_extra_outputs(PROJECT_EXEC_NAME)

# Link libraries
target_link_libraries(PROJECT_EXEC_NAME
        rp_agrolib_driver_name
        rp_agrolib_interface_name
        pico_stdlib
)
```
The driver library should come before the interface library.

___

## Development tools

The following tool versions are used and validated in CI:

- clang-format >= 18
- cppcheck >= 2.13

Using different versions may lead to formatting or static analysis differences.

___

## Useful links
https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf <br />
https://datasheets.raspberrypi.com/rp2040/hardware-design-with-rp2040.pdf

___

## Roadmap
### Interfaces
- [x] I2C
- [x] UART
- [x] USB
- [x] Simple GPIO
- [x] Simple PWM
- [ ] Simple Timer
- [ ] Simple I2C
- [ ] Simple ADC

### Drivers
- [x] Air temperature and humidity sensor HTU21D
- [x] Flash memory
- [x] Stepper motor driver DRV8825
- [x] Stepper motor general use
- [x] Servo motor DRS0101
- [x] GNSS LC29H (Quectel)
- [x] USB power delivery TPS25750DRJKR (Texas)
- [x] Optical distance sensor GP2Y0E02B
- [x] LiDAR VL53L4CD ToF
- [x] LiDAR TFmini-Plus
- [x] ADS1115
- [x] ADS1232
- [X] BNO055 IMU
- [X] FD60 linear actuator
- [X] LD20 linear actuator
- [X] MCP23017 16-Bit I/O expander
- [X] ADS8684 - 16-Bit, 500-kSPS, 4-Ch SAR ADC w/ programmable (±10/±5/±2.5V) input ranges on +5V supply
- [X] AD7980 - 16-Bit, 1 MSPS, PulSAR ADC in MSOP/LFCSP
- [X] AD5680 - 5 V 18-Bit nanoDAC® in a SOT-23
___

## Contributors
We thank the following team members for their contributions to this repository:

| Name                        | Email                            |
|-----------------------------| -------------------------------- |
| **Pedro Moura** (main contact)     | [pedro.h.moura@inesctec.pt](mailto:pedro.h.moura@inesctec.pt) |
| André Aguiar     | [andre.s.aguiar@inesctec.pt](mailto:andre.s.aguiar@inesctec.pt) |
| Daniel Silva     | [daniel.q.silva@inesctec.pt](mailto:daniel.q.silva@inesctec.pt) |
| Domingos Bento   | [domingos.bento@inesctec.pt](mailto:domingos.bento@inesctec.pt) |
| Francisco Terra  | [francisco.m.terra@inesctec.pt](mailto:francisco.m.terra@inesctec.pt) |
| Filipe Santos    | [filipe.n.santos@inesctec.pt](mailto:filipe.n.santos@inesctec.pt) |
| Isabel Pinheiro  | [isabel.a.pinheiro@inesctec.pt](mailto:isabel.a.pinheiro@inesctec.pt) |
| João Castro      | [joao.t.castro@inesctec.pt](mailto:joao.t.castro@inesctec.pt) |
| Pedro Rodrigues  | [pedro.rodrigues@inesctec.pt](mailto:pedro.rodrigues@inesctec.pt) |
| Vítor Tinoco     | [vitor.tinoco@inesctec.pt](mailto:vitor.tinoco@inesctec.pt) |
| Sandro Magalhães | [sandro.a.magalhaes@inesctec.pt](mailto:sandro.a.magalhaes@inesctec.pt) |

___

## License
This project is licensed under the GNU Affero General Public License v3.0 - see the [LICENSE](https://github.com/INESCTEC/tribe-rp-drivers/blob/master/LICENSE) file for details.

___
