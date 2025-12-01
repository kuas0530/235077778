cmake_minimum_required(VERSION 3.13)

# 1. 引入 Pico SDK
# 注意：GitHub Actions 會自動填入 PICO_SDK_PATH 變數，這裡直接引用
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

# 2. 設定專案與語言標準
project(host_hid C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# 3. 初始化 SDK
pico_sdk_init()

# 4. 建立執行檔 (包含所有原始碼)
add_executable(host_hid
    src/main.c
    src/usb_descriptors.c
    src/hid_reports.c
)

# 5. 設定標頭檔路徑
target_include_directories(host_hid PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# 6. 【關鍵修正】設定 TinyUSB 模式
# 強制設定 Port 0 為 Host 模式
# 強制設定 Port 1 為 0 (禁用)，這能解決 fatal error: pio_usb.h 的問題
target_compile_definitions(host_hid PRIVATE
    CFG_TUSB_RHPORT0_MODE=OPT_MODE_HOST
    CFG_TUSB_RHPORT1_MODE=0
)

# 7. 連結必要的庫
target_link_libraries(host_hid
    pico_stdlib
    tinyusb_host
    tinyusb_board
)

# 8. 【關鍵修正】禁用 USB stdio
# 這能解決 unknown type name 'bool' 的衝突錯誤
pico_enable_stdio_usb(host_hid 0)
pico_enable_stdio_uart(host_hid 1)

# 9. 產生 .uf2 燒錄檔
pico_add_extra_outputs(host_hid)
