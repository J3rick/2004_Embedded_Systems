cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD pico_w)

project(PicotoFlash C CXX ASM)

pico_sdk_init()
set(PICO_LWIP_CONTRIB_PATH "${PICO_SDK_PATH}/lib/lwip/contrib/apps")

add_executable(PicotoFlash
    hw_config.c
    identification.c
    sd_functions.c
    display_functions.c
    picotoflash.c
    write.c
    erase.c
    read.c
    ${PICO_LWIP_CONTRIB_PATH}/ping/ping.c
)

add_subdirectory(fatfs/FatFs_SPI build/fatfs_spi)

target_include_directories(PicotoFlash PRIVATE 
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/fatfs
    ${CMAKE_CURRENT_LIST_DIR}/fatfs/FatFs_SPI
    ${PICO_LWIP_CONTRIB_PATH}/apps/ping

)

target_compile_definitions(PicotoFlash PRIVATE
    NO_SYS=0
    LWIP_SOCKET=1
    PING_USE_SOCKETS=1
)

target_link_libraries(PicotoFlash
    pico_stdlib
    hardware_gpio
    hardware_adc
    hardware_spi
    FatFs_SPI
    pico_cyw43_arch_lwip_sys_freertos
    FreeRTOS-Kernel-Heap4

)


pico_enable_stdio_usb(PicotoFlash 1)
pico_enable_stdio_uart(PicotoFlash 0)
pico_add_extra_outputs(PicotoFlash)
