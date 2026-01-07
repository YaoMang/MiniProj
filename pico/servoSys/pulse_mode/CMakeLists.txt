cmake_minimum_required(VERSION 3.13)

# 引入 pico-sdk
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pulse_mode C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# 初始化 SDK
pico_sdk_init()

# ================================
# 可执行文件
# ================================
add_executable(pulse_mode
    main.cpp
    
    pio/pio_exec.cpp
    pio/pio_test.cpp

    drivers/ps100.cpp
    drivers/pwm_motor.cpp
)

target_include_directories(pulse_mode
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# ================================
# PIO 文件（必须）
# ================================
pico_generate_pio_header(
    pulse_mode
    ${CMAKE_CURRENT_LIST_DIR}/pio/motor_exec.pio
)

pico_generate_pio_header(
    pulse_mode
    ${CMAKE_CURRENT_LIST_DIR}/pio/radar_sync.pio
)

# ================================
# 链接硬件库
# ================================
target_link_libraries(pulse_mode
    pico_stdlib
    hardware_pio
    hardware_dma
    hardware_pwm
    hardware_gpio
    hardware_clocks
)

# ================================
# 启用 stdio（USB）
# ================================
pico_enable_stdio_usb(pulse_mode 1)
pico_enable_stdio_uart(pulse_mode 0)

# ================================
# 生成 UF2 / BIN / ELF
# ================================
pico_add_extra_outputs(pulse_mode)
