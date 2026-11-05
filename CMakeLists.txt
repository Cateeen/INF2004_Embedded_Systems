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
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

include(pico_sdk_import.cmake)

project(INF2004_Embedded_Systems C CXX ASM)

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
    hardware_test/imu_main.c
    src/imu_driver.c
    src/imu_processor.c
    src/buddy1.c
    src/telemetry_protocol.c
    src/wifi_mqtt_port_pico.c
)

target_include_directories(buddy4_imu_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(buddy4_imu_test
    pico_stdlib
    hardware_i2c
    pico_cyw43_arch_lwip_poll
    pico_lwip_mqtt
)
pico_enable_stdio_usb(buddy4_imu_test 1)
pico_enable_stdio_uart(buddy4_imu_test 0)
pico_add_extra_outputs(buddy4_imu_test)

add_executable(buddy3_ir_test
    hardware_test/ir_sensor_test.c
    src/line_following.c
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

add_executable(buddy3_line_following_motion_test
    tests/line_following_motion_test.c
    src/line_following.c
    src/motor_driver.c
)

target_include_directories(buddy3_line_following_motion_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(buddy3_line_following_motion_test
    pico_stdlib
    hardware_adc
    hardware_pwm
    hardware_clocks
)

pico_enable_stdio_usb(buddy3_line_following_motion_test 1)
pico_enable_stdio_uart(buddy3_line_following_motion_test 0)
pico_add_extra_outputs(buddy3_line_following_motion_test)

add_library(barcode_scanner STATIC
    src/barcode_scanner.c
)

target_include_directories(barcode_scanner PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

add_executable(buddy3_barcode_test
    tests/barcode_test.c
    src/barcode_scanner.c
    src/line_following.c
    src/encoder.c
)

target_include_directories(buddy3_barcode_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(buddy3_barcode_test
    pico_stdlib
    hardware_adc
    hardware_gpio
)

pico_enable_stdio_usb(buddy3_barcode_test 1)
pico_enable_stdio_uart(buddy3_barcode_test 0)
pico_add_extra_outputs(buddy3_barcode_test)

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
    src/line_following.c
    integration/obs_port_pico_car.c
    integration/vehicle_ctrl.c)

target_include_directories(picocar PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

# target_compile_definitions(picocar PRIVATE CAR_LINE_PIN=2 CAR_LINE_ACTIVE_LEVEL=0)   # real TCRT5000 D0, when wired
target_link_libraries(picocar
    pico_stdlib
    pico_multicore
    pico_util
    hardware_adc
    hardware_gpio
    hardware_pwm
    hardware_clocks
    line_steering
    line_status
    m
)
 
pico_enable_stdio_usb(picocar 1)
pico_enable_stdio_uart(picocar 0)
pico_add_extra_outputs(picocar)

add_executable(buddy4_imu_only_test
    hardware_test/imu_only_main.c
    src/imu_driver.c
    src/imu_processor.c
)

target_include_directories(buddy4_imu_only_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(buddy4_imu_only_test
    pico_stdlib
    hardware_i2c
)

# ==========================================
# PICO W WIFI TEST
# ==========================================
add_executable(wifi_test
    hardware_test/wifi_test.c
)

target_include_directories(wifi_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(wifi_test
    pico_stdlib
    pico_cyw43_arch_lwip_poll
)

pico_enable_stdio_usb(wifi_test 1)
pico_enable_stdio_uart(wifi_test 0)

pico_add_extra_outputs(wifi_test)

pico_enable_stdio_usb(buddy4_imu_only_test 1)
pico_enable_stdio_uart(buddy4_imu_only_test 0)
pico_add_extra_outputs(buddy4_imu_only_test)

# ==========================================
# BUDDY 2 + BUDDY 3 LINE STEERING ADAPTER
# ==========================================

add_library(line_steering STATIC
    src/line_steering.c
)

target_include_directories(line_steering PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

# ==========================================
# BUDDY 3 + BUDDY 5 SHARED LINE STATUS
# ==========================================

add_library(line_status STATIC
    src/line_status.c
)

target_include_directories(line_status PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(line_status PUBLIC
    pico_sync
)