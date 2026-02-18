include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

###########################################################################
# HTU i2C example
###########################################################################

add_executable(rp_htu21_read src/rp_htu21_read.cpp)

target_include_directories(rp_htu21_read PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../drivers/include)

pico_enable_stdio_usb(rp_htu21_read 1)
pico_enable_stdio_uart(rp_htu21_read 0)

pico_add_extra_outputs(rp_htu21_read)

target_link_libraries(rp_htu21_read
        rp_agrolib_htu21
        pico_stdlib)

###########################################################################
# TPS25750 configuration example
###########################################################################

add_executable(rp_tps25750_config src/rp_tps25750_config.cpp)

target_include_directories(rp_tps25750_config PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../drivers/include)
target_include_directories(rp_tps25750_config PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/tps25750_bin_configs)

pico_enable_stdio_usb(rp_tps25750_config 1)
pico_enable_stdio_uart(rp_tps25750_config 0)

pico_add_extra_outputs(rp_tps25750_config)

target_link_libraries(rp_tps25750_config
        rp_agrolib_tps25750
        pico_stdlib)

###########################################################################
# TPS25750 update patch bundle example
###########################################################################

add_executable(rp_tps25750_patch_bundle src/rp_tps25750_patch_bundle.cpp)

target_include_directories(rp_tps25750_patch_bundle PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../drivers/include)
target_include_directories(rp_tps25750_patch_bundle PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/tps25750_bin_configs)

pico_enable_stdio_usb(rp_tps25750_patch_bundle 1)
pico_enable_stdio_uart(rp_tps25750_patch_bundle 0)

pico_add_extra_outputs(rp_tps25750_patch_bundle)

target_link_libraries(rp_tps25750_patch_bundle
        rp_agrolib_tps25750
        rp_agrolib_uart
        pico_stdlib)

###########################################################################
# TPS25750 run state machine and keep working
###########################################################################

add_executable(rp_tps25750_run src/rp_tps25750_run.cpp)

target_include_directories(rp_tps25750_run PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../drivers/include)
target_include_directories(rp_tps25750_run PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/tps25750_bin_configs)

pico_enable_stdio_usb(rp_tps25750_run 1)
pico_enable_stdio_uart(rp_tps25750_run 0)

pico_add_extra_outputs(rp_tps25750_run)

target_link_libraries(rp_tps25750_run
        rp_agrolib_tps25750
        rp_agrolib_uart
        pico_stdlib)

###########################################################################
# DRS0101 move example
###########################################################################

add_executable(rp_move_drs0101 src/rp_move_drs0101.cpp)

target_include_directories(rp_move_drs0101 PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../drivers/include)

pico_enable_stdio_usb(rp_move_drs0101 1)
pico_enable_stdio_uart(rp_move_drs0101 0)

pico_add_extra_outputs(rp_move_drs0101)

target_link_libraries(rp_move_drs0101
        rp_agrolib_drs0101
        pico_stdlib)

###########################################################################
# Stepper move example
###########################################################################
add_executable(rp_stepper src/rp_stepper.cpp)

target_include_directories(rp_stepper PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../drivers/include)

pico_enable_stdio_usb(rp_stepper 1)
pico_enable_stdio_uart(rp_stepper 0)

pico_add_extra_outputs(rp_stepper)

target_link_libraries(rp_stepper
        rp_agrolib_stepper
        pico_stdlib)

###########################################################################
# LoraE5 class A lorawan example
###########################################################################
add_executable(rp_loraE5_classA_lorawan src/rp_loraE5_classA_lorawan.cpp)

target_include_directories(rp_loraE5_classA_lorawan PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../drivers/include)

pico_enable_stdio_usb(rp_loraE5_classA_lorawan 1)
pico_enable_stdio_uart(rp_loraE5_classA_lorawan 0)

pico_add_extra_outputs(rp_loraE5_classA_lorawan)

target_link_libraries(rp_loraE5_classA_lorawan
        rp_agrolib_lorae5
        pico_stdlib)

###########################################################################
# LoraE5 class A P2P example
###########################################################################
add_executable(rp_loraE5_classA_p2p src/rp_loraE5_classA_p2p.cpp)

target_include_directories(rp_loraE5_classA_p2p PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../drivers/include)

pico_enable_stdio_usb(rp_loraE5_classA_p2p 1)
pico_enable_stdio_uart(rp_loraE5_classA_p2p 0)

pico_add_extra_outputs(rp_loraE5_classA_p2p)

target_link_libraries(rp_loraE5_classA_p2p
        rp_agrolib_lorae5
        pico_stdlib)

###########################################################################
# LoraE5 class C P2P example
###########################################################################
add_executable(rp_loraE5_classC_p2p src/rp_loraE5_classC_p2p.cpp)

target_include_directories(rp_loraE5_classC_p2p PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../drivers/include)

pico_enable_stdio_usb(rp_loraE5_classC_p2p 1)
pico_enable_stdio_uart(rp_loraE5_classC_p2p 0)

pico_add_extra_outputs(rp_loraE5_classC_p2p)

target_link_libraries(rp_loraE5_classC_p2p
        rp_agrolib_lorae5
        pico_stdlib)

###########################################################################
# Dual core example
###########################################################################
add_executable(rp_dual_core src/rp_dual_core.cpp)

pico_enable_stdio_usb(rp_dual_core 1)
pico_enable_stdio_uart(rp_dual_core 0)

pico_add_extra_outputs(rp_dual_core)

target_link_libraries(rp_dual_core
        pico_stdlib
        pico_multicore)

###########################################################################
# Dual core interrupt example
###########################################################################
add_executable(rp_dual_core_interrupt src/rp_dual_core.cpp)

pico_enable_stdio_usb(rp_dual_core_interrupt 1)
pico_enable_stdio_uart(rp_dual_core_interrupt 0)

pico_add_extra_outputs(rp_dual_core_interrupt)

target_link_libraries(rp_dual_core_interrupt
        pico_stdlib
        pico_multicore)

###########################################################################
# GP2Y0E02B Distance example
###########################################################################
add_executable(rp_gp2y0e02b_distance src/rp_gp2y0e02b_distance.cpp)

target_include_directories(rp_gp2y0e02b_distance PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../drivers/include)

pico_enable_stdio_usb(rp_gp2y0e02b_distance 1)
pico_enable_stdio_uart(rp_gp2y0e02b_distance 0)

pico_add_extra_outputs(rp_gp2y0e02b_distance)

target_link_libraries(rp_gp2y0e02b_distance
        rp_agrolib_gp2y0e02b
        pico_stdlib)

###########################################################################
# TFmini Plus Lidar (UART) example
###########################################################################
add_executable(rp_tfminiplus_lidar_uart src/rp_tfminiplus_lidar_uart.cpp)

target_include_directories(rp_tfminiplus_lidar_uart PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../drivers/include)

pico_enable_stdio_usb(rp_tfminiplus_lidar_uart 1)
pico_enable_stdio_uart(rp_tfminiplus_lidar_uart 0)

pico_add_extra_outputs(rp_tfminiplus_lidar_uart)

target_link_libraries(rp_tfminiplus_lidar_uart
        rp_agrolib_tfminiplus
        pico_stdlib)

###########################################################################
# TFmini Plus Lidar (I2C) example
###########################################################################
add_executable(rp_tfminiplus_lidar_i2c src/rp_tfminiplus_lidar_i2c.cpp)

target_include_directories(rp_tfminiplus_lidar_i2c PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../drivers/include)

pico_enable_stdio_usb(rp_tfminiplus_lidar_i2c 1)
pico_enable_stdio_uart(rp_tfminiplus_lidar_i2c 0)

pico_add_extra_outputs(rp_tfminiplus_lidar_i2c)

target_link_libraries(rp_tfminiplus_lidar_i2c
        rp_agrolib_tfminiplus
        pico_stdlib)

###########################################################################
# MCP23017 output example
###########################################################################

add_executable(mcp23017_output src/rp_mcp23017_output.cpp)

target_include_directories(mcp23017_output PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../third_party/include)

pico_enable_stdio_usb(mcp23017_output 1)
pico_enable_stdio_uart(mcp23017_output 0)

pico_add_extra_outputs(mcp23017_output)

target_link_libraries(mcp23017_output
        pico_stdlib
        hardware_gpio
        hardware_i2c
        mcp23017_lib)

###########################################################################
# MCP23017 input example
###########################################################################

add_executable(mcp23017_input src/rp_mcp23017_input.cpp)

target_include_directories(mcp23017_input PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../third_party/include)

pico_enable_stdio_usb(mcp23017_input 1)
pico_enable_stdio_uart(mcp23017_input 0)

pico_add_extra_outputs(mcp23017_input)

target_link_libraries(mcp23017_input
        pico_stdlib
        hardware_gpio
        hardware_i2c
        mcp23017_lib)

###########################################################################
# Flash example
###########################################################################

add_executable(rp_flash src/rp_flash.cpp)

target_include_directories(rp_flash PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../drivers/include)

pico_enable_stdio_usb(rp_flash 1)
pico_enable_stdio_uart(rp_flash 0)

pico_add_extra_outputs(rp_flash)

target_link_libraries(rp_flash
        pico_stdlib
        hardware_flash
        rp_agrolib_flash)

###########################################################################
# Stepper DRV8825 example
###########################################################################

add_executable(rp_stepper_drv8825 src/rp_stepper_drv8825.cpp)

target_include_directories(rp_stepper_drv8825 PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../drivers/include)

pico_enable_stdio_usb(rp_stepper_drv8825 1)
pico_enable_stdio_uart(rp_stepper_drv8825 0)

pico_add_extra_outputs(rp_stepper_drv8825)

target_link_libraries(rp_stepper_drv8825
        pico_stdlib
        hardware_pio
        hardware_pwm
        rp_agrolib_drv8825
        )

###########################################################################
# ADS1232 example
###########################################################################

add_executable(rp_ads1232 src/rp_ads1232.cpp)

target_include_directories(rp_ads1232
        PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../drivers/include)

pico_enable_stdio_usb(rp_ads1232 1)
pico_enable_stdio_uart(rp_ads1232 0)

pico_add_extra_outputs(rp_ads1232)

target_link_libraries(rp_ads1232
        pico_stdlib
        hardware_adc
        rp_agrolib_ads1232
        )


###########################################################################
# AS7341 example
###########################################################################

add_executable(rp_as7341 src/rp_as7341.cpp)

target_include_directories(rp_as7341
        PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../drivers/include)

pico_enable_stdio_usb(rp_as7341 1)
pico_enable_stdio_uart(rp_as7341 0)

pico_add_extra_outputs(rp_as7341)

target_link_libraries(rp_as7341
        pico_stdlib
        hardware_i2c
        rp_agrolib_as7341
        )


###########################################################################
# LD20 electrical linear motor example
###########################################################################

add_executable(rp_linear_motor_ld20 src/rp_linear_motor_ld20.cpp)

target_include_directories(rp_linear_motor_ld20
        PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../drivers/include
        PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../interfaces/include)

pico_enable_stdio_usb(rp_linear_motor_ld20 1)
pico_enable_stdio_uart(rp_linear_motor_ld20 0)

pico_add_extra_outputs(rp_linear_motor_ld20)

target_link_libraries(rp_linear_motor_ld20
        pico_stdlib
        hardware_pwm
        cmsis_core
        hardware_i2c
        hardware_adc
        rp_agrolib_i2c
        mcp23017_lib
        )


###########################################################################
# FD60 electrical linear motor example
###########################################################################

add_executable(rp_linear_motor_fd60 src/rp_linear_motor_fd60.cpp)

target_include_directories(rp_linear_motor_fd60
        PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../drivers/include
        PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../interfaces/include)

pico_enable_stdio_usb(rp_linear_motor_fd60 1)
pico_enable_stdio_uart(rp_linear_motor_fd60 0)

pico_add_extra_outputs(rp_linear_motor_fd60)

target_link_libraries(rp_linear_motor_fd60
        pico_stdlib
        hardware_pwm
        cmsis_core
        hardware_i2c
        hardware_adc
        rp_agrolib_i2c
        mcp23017_lib
        )


###########################################################################
# Hamamatsu spectral sensors
###########################################################################

add_executable(rp_hamamatsu src/rp_hamamatsu.cpp)

target_include_directories(rp_hamamatsu
        PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../drivers/include
        PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../interfaces/include)

pico_enable_stdio_usb(rp_hamamatsu 1)
pico_enable_stdio_uart(rp_hamamatsu 0)

pico_add_extra_outputs(rp_hamamatsu)

target_link_libraries(rp_hamamatsu
        pico_stdlib
        hardware_adc
        hardware_spi
        rp_agrolib_hamamatsu_spectral
        rp_agrolib_ads8354
        )


###########################################################################
# Hamamatsu xenon lamp
###########################################################################

add_executable(rp_hamamatsu_xenon src/rp_hamamatsu_xenon.cpp)

target_include_directories(rp_hamamatsu_xenon
        PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../drivers/include
        PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../interfaces/include)

pico_enable_stdio_usb(rp_hamamatsu_xenon 1)
pico_enable_stdio_uart(rp_hamamatsu_xenon 0)

pico_add_extra_outputs(rp_hamamatsu_xenon)

target_link_libraries(rp_hamamatsu_xenon
        pico_stdlib
        rp_agrolib_hamamatsu_xenon
        )


###########################################################################
# DFRobot rain sensor
###########################################################################

add_executable(rp_rainfall_read src/rp_rainfall_read.cpp)

target_include_directories(rp_rainfall_read
        PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../drivers/include
        PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../interfaces/include)

pico_enable_stdio_usb(rp_rainfall_read 1)
pico_enable_stdio_uart(rp_rainfall_read 0)

pico_add_extra_outputs(rp_rainfall_read)

target_link_libraries(rp_rainfall_read
        pico_stdlib
        rp_agrolib_rainfall_sensor
        )


###########################################################################
# BNO055 IMU example
###########################################################################

add_executable(rp_bno055_imu src/rp_bno055_imu.cpp)

target_include_directories(rp_bno055_imu PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../drivers/include)

pico_enable_stdio_usb(rp_bno055_imu 1)
pico_enable_stdio_uart(rp_bno055_imu 0)

pico_add_extra_outputs(rp_bno055_imu)

target_link_libraries(rp_bno055_imu
        rp_agrolib_bno055
        pico_stdlib)


###########################################################################
# VL53L4CD ToF sensor example
###########################################################################

add_executable(rp_vl53l4cd_tof_i2c src/rp_vl53l4cd_tof_i2c.cpp)

target_include_directories(rp_vl53l4cd_tof_i2c
        PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../interfaces/include
        PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../third-party/include)

pico_enable_stdio_usb(rp_vl53l4cd_tof_i2c 1)
pico_enable_stdio_uart(rp_vl53l4cd_tof_i2c 0)

pico_add_extra_outputs(rp_vl53l4cd_tof_i2c)

target_link_libraries(rp_vl53l4cd_tof_i2c
        rp_agrolib_i2c
        vl53l4cd_uld_api
        pico_stdlib)
