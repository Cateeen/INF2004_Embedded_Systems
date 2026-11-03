
cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico_w CACHE STRING "Target board")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_SDK_PATH "C:/Users/User/.pico-sdk/sdk/1.5.1" CACHE PATH "Pico SDK")
include(pico_sdk_import.cmake)

project(INF2004_Embedded_Systems C CXX ASM)

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

add_executable(buddy5_obs
    hardware_test/obs_main.c
    src/obs_port_pico_bare.c
    src/obs_pack.c
    src/obs_profile.c
    src/obs_scan.c
)

target_include_directories(buddy5_obs PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(buddy5_obs
    pico_stdlib
    hardware_pwm
    hardware_clocks
    m
)

pico_enable_stdio_usb(buddy5_obs 1)
pico_enable_stdio_uart(buddy5_obs 0)
pico_add_extra_outputs(buddy5_obs)

add_executable(picocar
    integration/car_main.c
    src/obs_scan.c 
    src/obs_profile.c 
    src/obs_pack.c
    src/motor_driver.c
    integration/obs_port_pico_car.c
    integration/vehicle_ctrl.c)

target_include_directories(picocar PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

# target_compile_definitions(picocar PRIVATE CAR_LINE_PIN=2 CAR_LINE_ACTIVE_LEVEL=0)   # real TCRT5000 D0, when wired
target_link_libraries(picocar pico_stdlib pico_multicore pico_util
    hardware_gpio hardware_pwm hardware_clocks m)
 
pico_enable_stdio_usb(picocar 1)
pico_enable_stdio_uart(picocar 0)
pico_add_extra_outputs(picocar)