# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
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
    src/motion_control.c
    src/pid_controller.c
    src/wheel_odometry.c
    src/wheel_feedback.c
    src/wheel_speed_control.c
    src/wheel_feedback_pid.c
    src/motion_feedback_bridge.c
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

add_executable(buddy3_ir_test
    hardware_test/ir_sensor_test.c
)

target_include_directories(buddy3_ir_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(buddy3_ir_test
    pico_stdlib
    hardware_adc
)

pico_enable_stdio_usb(buddy3_ir_test 1)
pico_enable_stdio_uart(buddy3_ir_test 0)
pico_add_extra_outputs(buddy3_ir_test)