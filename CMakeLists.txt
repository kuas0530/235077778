cmake_minimum_required(VERSION 3.13)

# 引入 SDK
include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

project(host_hid C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# 初始化 SDK
pico_sdk_init()

# 建立執行檔
add_executable(host_hid
    src/main.c
    src/usb_descriptors.c
    src/hid_reports.c
)

# 包含路徑
target_include_directories(host_hid PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# 設定 Host 模式，並強制指定使用 Port 0 (原生 USB)
target_compile_definitions(host_hid PRIVATE
    CFG_TUSB_RHPORT0_MODE=OPT_MODE_HOST
    CFG_TUSB_RHPORT1_MODE=0
)

# 連結庫
target_link_libraries(host_hid
    pico_stdlib
    tinyusb_host
    tinyusb_board
)

# 禁用 stdio USB 避免衝突
pico_enable_stdio_usb(host_hid 0)
pico_enable_stdio_uart(host_hid 1)

# 產生 uf2
pico_add_extra_outputs(host_hid)
