#########################################
# MCP23017
#########################################

add_library(pico_interfaces INTERFACE)
target_include_directories(pico_interfaces INTERFACE ${CMAKE_CURRENT_LIST_DIR}/pico-interfaces/)

add_library(mcp23017_lib
        ${CMAKE_CURRENT_LIST_DIR}/pico-mcp23017/source/mcp23017.cpp
        ${CMAKE_CURRENT_LIST_DIR}/pico-mcp23017/source/mcp23017_input.cpp
        ${CMAKE_CURRENT_LIST_DIR}/pico-mcp23017/source/mcp23017_latching_output.cpp
        )

target_include_directories(mcp23017_lib PUBLIC ${CMAKE_CURRENT_LIST_DIR}/pico-mcp23017/api)

target_link_libraries(mcp23017_lib PUBLIC
        pico_interfaces
        pico_stdlib
        hardware_gpio
        hardware_i2c
        )

install(TARGETS mcp23017_lib
        ARCHIVE DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        LIBRARY DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        RUNTIME DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin
        )

install(DIRECTORY pico-mcp23017/api/
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/include
        )

#########################################
# BNO055
#########################################

add_library(bno055_lib
        BNO055_SensorAPI/bno055.c
        BNO055_SensorAPI/bno055.h
        )

target_include_directories(bno055_lib INTERFACE ${CMAKE_CURRENT_LIST_DIR}/BNO055_SensorAPI)

target_link_libraries(bno055_lib PUBLIC
        pico_stdlib
        )

install(TARGETS bno055_lib
        ARCHIVE DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        LIBRARY DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        RUNTIME DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin
        )

install(DIRECTORY BNO055_SensorAPI/
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/include
        )

#########################################
# VL53L4CD
#########################################

add_library(vl53l4cd_uld_api
        VL53L4CD_ULD_API/VL53L4CD_api.c
        VL53L4CD_ULD_API/VL53L4CD_api.h
        VL53L4CD_ULD_API/VL53L4CD_calibration.c
        VL53L4CD_ULD_API/VL53L4CD_calibration.h
        VL53L4CD_ULD_API/platform.c
        VL53L4CD_ULD_API/platform.h
        )

target_include_directories(vl53l4cd_uld_api INTERFACE ${CMAKE_CURRENT_LIST_DIR}/VL53L4CD_ULD_API)

target_link_libraries(vl53l4cd_uld_api PUBLIC
        hardware_i2c
        pico_stdlib
        )

install(TARGETS vl53l4cd_uld_api
        ARCHIVE DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        LIBRARY DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        RUNTIME DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin
        )

install(DIRECTORY VL53L4CD_ULD_API/
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/include
        )
