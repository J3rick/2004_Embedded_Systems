cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD pico_w)

project(Project C CXX ASM)

add_executable(Project
    hw_config.c
    identification.c
    sd_functions.c
    display_functions.c
    read.c
    write.c
    erase.c
)

add_subdirectory(fatfs/FatFs_SPI build/fatfs_spi)

target_include_directories(Project PRIVATE 
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/helper_functions/fatfs
    ${CMAKE_CURRENT_LIST_DIR}/helper_functions/fatfs/FatFs_SPI
)

target_link_libraries(Project
    pico_stdlib
    hardware_gpio
    hardware_adc
    hardware_spi
    FatFs_SPI
)

pico_enable_stdio_usb(Project 1)
pico_enable_stdio_uart(Project 1)
pico_add_extra_outputs(Project)
