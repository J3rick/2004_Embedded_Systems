cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD pico_w)

project(PicotoFlash C CXX ASM)

add_executable(PicotoFlash
    hw_config.c
    identification.c
    sd_functions.c
    display_functions.c
    picotoflash.c
    write.c
    erase.c
    read.c
)

add_subdirectory(fatfs/FatFs_SPI build/fatfs_spi)

target_include_directories(PicotoFlash PRIVATE 
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/fatfs
    ${CMAKE_CURRENT_LIST_DIR}/fatfs/FatFs_SPI
)

target_link_libraries(PicotoFlash
    pico_stdlib
    hardware_gpio
    hardware_adc
    hardware_spi
    FatFs_SPI
)


pico_enable_stdio_usb(PicotoFlash 1)
pico_enable_stdio_uart(PicotoFlash 0)
pico_add_extra_outputs(PicotoFlash)
