cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD pico_w)

project(project111 C CXX ASM)

add_executable(project111
    main.c
    hw_config.c
    identification.c
    sd_functions.c
    display_functions.c
)

add_subdirectory(fatfs/FatFs_SPI build/fatfs_spi)

target_include_directories(project111 PRIVATE 
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/fatfs
    ${CMAKE_CURRENT_LIST_DIR}/fatfs/FatFs_SPI
)

target_link_libraries(project111
    pico_stdlib
    hardware_gpio
    hardware_adc
    hardware_spi
    FatFs_SPI
)

pico_enable_stdio_usb(project111 1)
pico_enable_stdio_uart(project111 1)
pico_add_extra_outputs(project111)
