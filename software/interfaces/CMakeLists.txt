###########################################################################
# RP AGROLIB I2C
###########################################################################

add_library(rp_agrolib_i2c
        src/rp_agrolib_i2c.cpp
        include/rp_agrolib_i2c.h)

pico_enable_stdio_usb(rp_agrolib_i2c 1)
pico_enable_stdio_uart(rp_agrolib_i2c 0)

target_link_libraries(rp_agrolib_i2c PUBLIC
        pico_stdlib
        hardware_i2c
        )

target_include_directories(rp_agrolib_i2c PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PICO_SDK_PATH}/src/rp2_common/hardware_i2c/include
)

###########################################################################
# RP AGROLIB UART
###########################################################################
add_library(rp_agrolib_uart
        src/rp_agrolib_uart.cpp
        include/rp_agrolib_uart.h)

pico_enable_stdio_usb(rp_agrolib_uart 1)
pico_enable_stdio_uart(rp_agrolib_uart 0)

target_link_libraries(rp_agrolib_uart PUBLIC
        pico_stdlib
        hardware_uart
        )

target_include_directories(rp_agrolib_uart PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PICO_SDK_PATH}/src/rp2_common/hardware_uart/include
)

###########################################################################
# TRIBE RP DRIVER SIMPLE GPIO
###########################################################################
add_library(rp_agrolib_simple_gpio
        include/rp_agrolib_simple_gpio.hpp)

pico_enable_stdio_usb(rp_agrolib_simple_gpio 1)
pico_enable_stdio_uart(rp_agrolib_simple_gpio 0)

target_link_libraries(rp_agrolib_simple_gpio PUBLIC
        pico_stdlib
        hardware_gpio
        )

target_include_directories(rp_agrolib_simple_gpio PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PICO_SDK_PATH}/src/rp2_common/hardware_gpio/include
)

###########################################################################
# TRIBE RP DRIVER SIMPLE PWM
###########################################################################
add_library(rp_agrolib_simple_pwm
        include/rp_agrolib_simple_pwm.hpp)

pico_enable_stdio_usb(rp_agrolib_simple_pwm 1)
pico_enable_stdio_uart(rp_agrolib_simple_pwm 0)

target_link_libraries(rp_agrolib_simple_pwm PUBLIC
        pico_stdlib
        hardware_gpio
        hardware_pwm
        hardware_clocks
        )

target_include_directories(rp_agrolib_simple_pwm PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PICO_SDK_PATH}/src/rp2_common/rp_agrolib_simple_pwm/include
)



###########################################################################
# COMMON - To all interfaces
###########################################################################

install(TARGETS rp_agrolib_i2c rp_agrolib_uart rp_agrolib_simple_gpio

        ARCHIVE DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        LIBRARY DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        RUNTIME DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin
        )

install(DIRECTORY include/
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/include
        )
