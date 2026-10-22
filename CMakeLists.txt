
cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico_w CACHE STRING "Target board")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(INF2004_Embedded_Systems C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(buddy2_hardware_test
    hardware_test/main.c
    src/motor_driver.c
    src/encoder.c
)

target_include_directories(buddy2_hardware_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(buddy2_hardware_test
    pico_stdlib
    hardware_pwm
    hardware_clocks
)

pico_enable_stdio_usb(buddy2_hardware_test 1)
pico_enable_stdio_uart(buddy2_hardware_test 0)
pico_add_extra_outputs(buddy2_hardware_test)

add_executable(buddy4_imu_test
    hardware_test/imu_main.c        # Or whatever you named your test main file
    src/imu_driver.c
    src/imu_processor.c
)

target_include_directories(buddy4_imu_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(buddy4_imu_test
    pico_stdlib
    hardware_i2c                    # Required for your IMU
)
pico_enable_stdio_usb(buddy4_imu_test 1)
pico_enable_stdio_uart(buddy4_imu_test 0)
pico_add_extra_outputs(buddy4_imu_test)