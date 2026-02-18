cmake_minimum_required(VERSION 3.16)

include(pico_sdk_import.cmake)

project(rp2040_agrolib C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_subdirectory(third_party)
add_subdirectory(interfaces)
add_subdirectory(drivers)
add_subdirectory(examples)