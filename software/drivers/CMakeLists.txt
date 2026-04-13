###########################################################################
# HTU21 - Air Temperature and Humidity Sensor
###########################################################################

add_library(rp_agrolib_htu21
        src/rp_agrolib_htu21.cpp
        include/rp_agrolib_htu21.h)

# target_include_directories(rp_agrolib_htu21 PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../interfaces/include)

pico_enable_stdio_usb(rp_agrolib_htu21 1)
pico_enable_stdio_uart(rp_agrolib_htu21 0)

target_link_libraries(rp_agrolib_htu21 PUBLIC
        rp_agrolib_i2c
        pico_stdlib
        )

target_include_directories(rp_agrolib_htu21 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

install(TARGETS rp_agrolib_htu21
        ARCHIVE DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        LIBRARY DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        RUNTIME DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin
        )

###########################################################################
# TPS25750 - Texas USB Power Delivery
###########################################################################

add_library(rp_agrolib_tps25750
        src/rp_agrolib_tps25750.cpp
        include/rp_agrolib_tps25750.h)

# target_include_directories(rp_agrolib_tps25750 PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../interfaces/include)

pico_enable_stdio_usb(rp_agrolib_tps25750 1)
pico_enable_stdio_uart(rp_agrolib_tps25750 0)

target_link_libraries(rp_agrolib_tps25750 PUBLIC
        rp_agrolib_i2c  # Linkando com a biblioteca rp_agrolib_i2c
        rp_agrolib_uart
        pico_stdlib
        # Outras bibliotecas necessárias
        )

target_include_directories(rp_agrolib_tps25750 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

install(TARGETS rp_agrolib_tps25750
        ARCHIVE DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        LIBRARY DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        RUNTIME DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin
        )

###########################################################################
# Stepper
###########################################################################

add_library(rp_agrolib_stepper
        src/rp_agrolib_stepper.c
        include/rp_agrolib_stepper.h)

# target_include_directories(rp_agrolib_stepper PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../interfaces/include)

# Configurações específicas para a biblioteca rp_agrolib_stepper
pico_enable_stdio_usb(rp_agrolib_stepper 1)
pico_enable_stdio_uart(rp_agrolib_stepper 0)

# Link com as bibliotecas necessárias
target_link_libraries(rp_agrolib_stepper PUBLIC
        pico_stdlib
        hardware_pwm
        )

target_include_directories(rp_agrolib_stepper PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Configuração da instalação para rp_agrolib_stepper
install(TARGETS rp_agrolib_stepper
        ARCHIVE DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        LIBRARY DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        RUNTIME DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin
        )

###########################################################################
# DRS0101 - Herkulex RS232 Servo Motor
###########################################################################

add_library(rp_agrolib_drs0101
        src/rp_agrolib_drs0101.cpp
        include/rp_agrolib_drs0101.h)

# target_include_directories(rp_agrolib_drs0101 PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../interfaces/include)

pico_enable_stdio_usb(rp_agrolib_drs0101 1)
pico_enable_stdio_uart(rp_agrolib_drs0101 0)

target_link_libraries(rp_agrolib_drs0101 PUBLIC
        rp_agrolib_uart
        pico_stdlib
        # Outras bibliotecas necessárias
        )

target_include_directories(rp_agrolib_drs0101 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

install(TARGETS rp_agrolib_drs0101
        ARCHIVE DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        LIBRARY DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        RUNTIME DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin
        )


###########################################################################
# GP2Y0E02B - Sharp Optical Distance Sensor (4-50cm)
###########################################################################

add_library(rp_agrolib_gp2y0e02b
        src/rp_agrolib_gp2y0e02b.cpp
        include/rp_agrolib_gp2y0e02b.h)

# target_include_directories(rp_agrolib_gp2y0e02b PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../interfaces/include)

pico_enable_stdio_usb(rp_agrolib_gp2y0e02b 1)
pico_enable_stdio_uart(rp_agrolib_gp2y0e02b 0)

target_link_libraries(rp_agrolib_gp2y0e02b PUBLIC
        rp_agrolib_i2c
        pico_stdlib
        # Outras bibliotecas necessárias
        )

target_include_directories(rp_agrolib_gp2y0e02b PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

install(TARGETS rp_agrolib_gp2y0e02b
        ARCHIVE DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        LIBRARY DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        RUNTIME DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin
        )

###########################################################################
# LC29H - Quectel RTK Module
###########################################################################

add_library(rp_agrolib_lc29h
        src/rp_agrolib_lc29h.cpp
        include/rp_agrolib_lc29h.h)

# target_include_directories(rp_agrolib_lc29h PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../interfaces/include)

pico_enable_stdio_usb(rp_agrolib_lc29h 1)
pico_enable_stdio_uart(rp_agrolib_lc29h 0)

target_link_libraries(rp_agrolib_lc29h PUBLIC
        rp_agrolib_uart
        pico_stdlib
        # Outras bibliotecas necessárias
        )

target_include_directories(rp_agrolib_lc29h PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

install(TARGETS rp_agrolib_lc29h
        ARCHIVE DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        LIBRARY DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        RUNTIME DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin
        )

###########################################################################
# TFmini Plus - Benewake 1D ToF LiDAR
###########################################################################

add_library(rp_agrolib_tfminiplus
        src/rp_agrolib_tfminiplus.cpp
        include/rp_agrolib_tfminiplus.h)

# target_include_directories(rp_agrolib_tfminiplus PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../interfaces/include)

pico_enable_stdio_usb(rp_agrolib_tfminiplus 1)
pico_enable_stdio_uart(rp_agrolib_tfminiplus 0)

target_link_libraries(rp_agrolib_tfminiplus PUBLIC
        rp_agrolib_uart
        rp_agrolib_i2c
        pico_stdlib
        # Outras bibliotecas necessárias
        )

target_include_directories(rp_agrolib_tfminiplus PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

install(TARGETS rp_agrolib_tfminiplus
        ARCHIVE DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        LIBRARY DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        RUNTIME DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin
        )

###########################################################################
# ADS1115 - External AD of 16 bits
###########################################################################

add_library(rp_agrolib_ads1115
        src/rp_agrolib_ads1115.c
        include/rp_agrolib_ads1115.h)

# target_include_directories(rp_agrolib_ads1115 PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../interfaces/include)

pico_enable_stdio_usb(rp_agrolib_ads1115 1)
pico_enable_stdio_uart(rp_agrolib_ads1115 0)

target_link_libraries(rp_agrolib_ads1115 PUBLIC
        pico_stdlib
	      hardware_i2c
        # Outras bibliotecas necessárias
        )

target_include_directories(rp_agrolib_ads1115 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

install(TARGETS rp_agrolib_ads1115
        ARCHIVE DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        LIBRARY DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        RUNTIME DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin
        )

###########################################################################
# Flash memory
###########################################################################

add_library(rp_agrolib_flash
        src/rp_agrolib_flash.cpp
        include/rp_agrolib_flash.h)

# target_include_directories(rp_agrolib_flash PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../interfaces/include)

pico_enable_stdio_usb(rp_agrolib_flash 1)
pico_enable_stdio_uart(rp_agrolib_flash 0)

target_link_libraries(rp_agrolib_flash PUBLIC
        pico_stdlib
	      hardware_flash
        # Outras bibliotecas necessárias
        )

target_include_directories(rp_agrolib_flash PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

install(TARGETS rp_agrolib_flash
        ARCHIVE DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        LIBRARY DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        RUNTIME DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin
        )

###########################################################################
# DRV8825 - Stepper motor
###########################################################################

add_library(rp_agrolib_drv8825
        src/rp_agrolib_drv8825.c
        include/rp_agrolib_drv8825.h)

# target_include_directories(rp_agrolib_drv8825 PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../interfaces/include)

pico_enable_stdio_usb(rp_agrolib_drv8825 1)
pico_enable_stdio_uart(rp_agrolib_drv8825 0)

target_link_libraries(rp_agrolib_drv8825 PUBLIC
        pico_stdlib
        hardware_pio
        hardware_pwm
        # Outras bibliotecas necessárias
        )

target_include_directories(rp_agrolib_drv8825 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

install(TARGETS rp_agrolib_drv8825
        ARCHIVE DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        LIBRARY DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        RUNTIME DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin
        )

###########################################################################
# ADS1232 - ADC for bridge sensor
###########################################################################

add_library(rp_agrolib_ads1232
        src/rp_agrolib_ads1232.cpp
        include/rp_agrolib_ads1232.h)

# target_include_directories(rp_agrolib_ads1232
 #       PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../interfaces/include)

pico_enable_stdio_usb(rp_agrolib_ads1232 1)
pico_enable_stdio_uart(rp_agrolib_ads1232 0)

target_link_libraries(rp_agrolib_ads1232 PUBLIC
        pico_stdlib
        hardware_pwm
        )

target_include_directories(rp_agrolib_ads1232 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

install(TARGETS rp_agrolib_ads1232
        ARCHIVE DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        LIBRARY DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        RUNTIME DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin
        )


###########################################################################
# AS7341 - sepctral sensor
###########################################################################

add_library(rp_agrolib_as7341
        src/rp_agrolib_as7341.cpp
        include/rp_agrolib_as7341.h)

# target_include_directories(rp_agrolib_as7341
#        PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../interfaces/include)

pico_enable_stdio_usb(rp_agrolib_as7341 1)
pico_enable_stdio_uart(rp_agrolib_as7341 0)

target_link_libraries(rp_agrolib_as7341 PUBLIC
        pico_stdlib
        rp_agrolib_i2c
        )

target_include_directories(rp_agrolib_as7341 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

install(TARGETS rp_agrolib_as7341
        ARCHIVE DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        LIBRARY DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        RUNTIME DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin
        )



###########################################################################
# ADS8354 -  Dual, 700 kSPS, 16 bit Simultaneous-Sampling ADC
###########################################################################

add_library(rp_agrolib_ads8354
        src/rp_agrolib_ads8354.cpp
        include/rp_agrolib_ads8354.h)

# target_include_directories(rp_agrolib_ads8354
   #     PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../interfaces/include)

pico_enable_stdio_usb(rp_agrolib_ads8354 1)
pico_enable_stdio_uart(rp_agrolib_ads8354 0)

target_link_libraries(rp_agrolib_ads8354 PUBLIC
        pico_stdlib
        hardware_spi
        )

target_include_directories(rp_agrolib_ads8354 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

install(TARGETS rp_agrolib_ads8354
        ARCHIVE DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        LIBRARY DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        RUNTIME DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin
        )

###########################################################################
# Hamamatsu -  sensores espectrais C12880MA e C16767MA
###########################################################################

add_library(rp_agrolib_hamamatsu_spectral
        src/rp_agrolib_hamamatsu_spectral.cpp
        include/rp_agrolib_hamamatsu_spectral.h)

# target_include_directories(rp_agrolib_hamamatsu_spectral
 #       PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../interfaces/include)

pico_enable_stdio_usb(rp_agrolib_hamamatsu_spectral 1)
pico_enable_stdio_uart(rp_agrolib_hamamatsu_spectral 0)

target_link_libraries(rp_agrolib_hamamatsu_spectral PUBLIC
        pico_stdlib
        hardware_spi
        hardware_adc
        )

target_include_directories(rp_agrolib_hamamatsu_spectral PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

install(TARGETS rp_agrolib_hamamatsu_spectral
        ARCHIVE DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        LIBRARY DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        RUNTIME DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin
        )


###########################################################################
# Hamamatsu -  Lampada Xenon L13651-11
###########################################################################

add_library(rp_agrolib_hamamatsu_xenon
        src/rp_agrolib_hamamatsu_xenon.cpp
        include/rp_agrolib_hamamatsu_xenon.h)

#target_include_directories(rp_agrolib_hamamatsu_xenon
 #       PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../interfaces/include)

pico_enable_stdio_usb(rp_agrolib_hamamatsu_xenon 1)
pico_enable_stdio_uart(rp_agrolib_hamamatsu_xenon 0)

target_link_libraries(rp_agrolib_hamamatsu_xenon PUBLIC
        pico_stdlib
        )

target_include_directories(rp_agrolib_hamamatsu_xenon PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

install(TARGETS rp_agrolib_hamamatsu_xenon
        ARCHIVE DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        LIBRARY DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        RUNTIME DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin
        )


###########################################################################
# DFRobot rainfall sensor
###########################################################################

add_library(rp_agrolib_rainfall_sensor
        src/rp_agrolib_rainfall_sensor.cpp
        include/rp_agrolib_rainfall_sensor.h)

#target_include_directories(rp_agrolib_rainfall_sensor
 #       PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../interfaces/include)

pico_enable_stdio_usb(rp_agrolib_rainfall_sensor 1)
pico_enable_stdio_uart(rp_agrolib_rainfall_sensor 0)

target_link_libraries(rp_agrolib_rainfall_sensor PUBLIC
        pico_stdlib
        hardware_i2c
        )

target_include_directories(rp_agrolib_rainfall_sensor PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

install(TARGETS rp_agrolib_rainfall_sensor
        ARCHIVE DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        LIBRARY DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        RUNTIME DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin
        )



###########################################################################
# LoRa-E5 - ST LoRa & LoRaWAN module SX126x embedded
###########################################################################

add_library(rp_agrolib_lorae5
        src/rp_agrolib_loraE5.cpp
        include/rp_agrolib_loraE5.h)

#target_include_directories(rp_agrolib_lorae5 PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../interfaces/include)

pico_enable_stdio_usb(rp_agrolib_lorae5 1)
pico_enable_stdio_uart(rp_agrolib_lorae5 0)

target_link_libraries(rp_agrolib_lorae5 PUBLIC
        rp_agrolib_uart
        pico_stdlib
        # Outras bibliotecas necessárias
        )

target_include_directories(rp_agrolib_lorae5 PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}/include
		)

install(TARGETS rp_agrolib_lorae5
        ARCHIVE DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        LIBRARY DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        RUNTIME DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin
        )

###########################################################################
# BNO055 - Bosch IMU
###########################################################################

add_library(rp_agrolib_bno055
        src/rp_agrolib_bno055.cpp
        include/rp_agrolib_bno055.h)

pico_enable_stdio_usb(rp_agrolib_bno055 1)
pico_enable_stdio_uart(rp_agrolib_bno055 0)

target_link_libraries(rp_agrolib_bno055 PUBLIC
        rp_agrolib_i2c
        pico_stdlib
        # Outras bibliotecas necessárias
        bno055_lib
        )

target_include_directories(rp_agrolib_bno055 PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}/include
		)

install(TARGETS rp_agrolib_bno055
        ARCHIVE DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        LIBRARY DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        RUNTIME DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin
        )


###########################################################################
# ADS8684 - TI ADC of 16 bits with SPI interface (4 channels)
###########################################################################

add_library(rp_agrolib_ads8684
        src/rp_agrolib_ads8684.cpp
        include/rp_agrolib_ads8684.h)

pico_enable_stdio_usb(rp_agrolib_ads8684 1)
pico_enable_stdio_uart(rp_agrolib_ads8684 0)

target_link_libraries(rp_agrolib_ads8684 PUBLIC
        pico_stdlib
        hardware_spi
        hardware_gpio
        )

target_include_directories(rp_agrolib_ads8684 PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}/include
		)

install(TARGETS rp_agrolib_ads8684
        ARCHIVE DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        LIBRARY DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        RUNTIME DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin
        )


###########################################################################
# AD7980 - Analog Devices ADC of 16 bits with SPI interface
###########################################################################

add_library(rp_agrolib_ad7980
        src/rp_agrolib_ad7980.cpp
        include/rp_agrolib_ad7980.h)

pico_enable_stdio_usb(rp_agrolib_ad7980 1)
pico_enable_stdio_uart(rp_agrolib_ad7980 0)

target_link_libraries(rp_agrolib_ad7980 PUBLIC
        pico_stdlib
        hardware_spi
        hardware_gpio
        )

target_include_directories(rp_agrolib_ad7980 PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}/include
		)

install(TARGETS rp_agrolib_ad7980
        ARCHIVE DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        LIBRARY DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        RUNTIME DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin
        )

# Enable interrupt support
target_compile_definitions(rp_agrolib_ad7980 PRIVATE
    PICO_MAX_SHARED_IRQ_HANDLERS=8
)


###########################################################################
# AD5680 - Analog Devices DAC of 16 bits with SPI interface
###########################################################################

add_library(rp_agrolib_ad5680
        src/rp_agrolib_ad5680.cpp
        include/rp_agrolib_ad5680.h)

pico_enable_stdio_usb(rp_agrolib_ad5680 1)
pico_enable_stdio_uart(rp_agrolib_ad5680 0)

target_link_libraries(rp_agrolib_ad5680 PUBLIC
        pico_stdlib
        hardware_spi
        hardware_gpio
        )

target_include_directories(rp_agrolib_ad5680 PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}/include
		)

install(TARGETS rp_agrolib_ad5680
        ARCHIVE DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        LIBRARY DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib
        RUNTIME DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin
        )


###########################################################################
# COMMON - To all drivers
###########################################################################

install(DIRECTORY include/
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/include
        )
