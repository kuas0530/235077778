cmake_minimum_required(VERSION 3.13)

# 關鍵點：這裡必須用 ${} 括號，不能用 ENV，因為我們在 YAML 是用 -D 傳進來的
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

project(host_hid C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(host_hid
    src/main.c
    src/usb_descriptors.c
    src/hid_reports.c
)

target_include_directories(host_hid PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_compile_definitions(host_hid PRIVATE
    CFG_TUSB_RHPORT0_MODE=OPT_MODE_HOST
)

target_link_libraries(host_hid
    pico_stdlib
    tinyusb_host
    tinyusb_board
)

pico_enable_stdio_usb(host_hid 0)
pico_enable_stdio_uart(host_hid 1)

pico_add_extra_outputs(host_hid)

