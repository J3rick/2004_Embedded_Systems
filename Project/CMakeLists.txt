cmake_minimum_required(VERSION 3.13)

# --- Tell CMake where the Pico SDK is ---
if(NOT DEFINED PICO_SDK_PATH)
    set(PICO_SDK_PATH "C:/pico/pico-sdk" CACHE PATH "Path to the Raspberry Pi Pico SDK")
endif()

# Import Pico SDK
if(EXISTS "${PICO_SDK_PATH}/external/pico_sdk_import.cmake")
    include("${PICO_SDK_PATH}/external/pico_sdk_import.cmake")
elseif(EXISTS "${CMAKE_SOURCE_DIR}/pico_sdk_import.cmake")
    include("${CMAKE_SOURCE_DIR}/pico_sdk_import.cmake")
else()
    message(FATAL_ERROR
        "Couldn't find pico_sdk_import.cmake. Set PICO_SDK_PATH correctly.")
endif()

# --- Project / board ---
project(Project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_BOARD pico_w)
pico_sdk_init()

# --- Include directories ---
include_directories(
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/helper_functions
    ${CMAKE_SOURCE_DIR}/helper_functions/fatfs
    ${CMAKE_SOURCE_DIR}/helper_functions/fatfs/FatFs_SPI
    ${CMAKE_SOURCE_DIR}/helper_functions/fatfs/FatFs_SPI/sd_driver
)

# =========================
# Target: Project
# =========================
add_executable( Project
    # Main integrated file
    main_flash_forensic.c
    
    # Your helper modules
    helper_functions/read.c
    helper_functions/write.c
    helper_functions/erase.c
    helper_functions/identification.c
    helper_functions/sd_functions.c
    helper_functions/display_functions.c
    
    # FatFS library files (adjust paths if different)
    helper_functions/fatfs/FatFs_SPI/ff15/source/ff.c
    helper_functions/fatfs/FatFs_SPI/ff15/source/ffsystem.c
    helper_functions/fatfs/FatFs_SPI/ff15/source/ffunicode.c
    helper_functions/fatfs/FatFs_SPI/sd_driver/sd_card.c
    helper_functions/fatfs/FatFs_SPI/sd_driver/spi.c
    
    # Add hw_config.c if you have it
    # helper_functions/hw_config.c
)

target_link_libraries(Project
    pico_stdlib
    pico_multicore
    hardware_uart
    hardware_gpio
    hardware_spi
    hardware_rtc
    hardware_clocks
    FatFS_SPI
    # Add WiFi support when needed
    # pico_cyw43_arch_lwip_threadsafe_background
)

# USB stdio ON, UART stdio OFF
pico_enable_stdio_usb(Project 1)
pico_enable_stdio_uart(Project 0)
pico_add_extra_outputs(Project)