add_executable(PicotoFlash
    picotoflash.c    # <-- rename if your file has a different name
    write.c
    erase.c
    read.c
)

# Use USB for stdio (appears as a serial COM port). Disable UART stdio.
pico_enable_stdio_usb(PicotoFlash 1)
pico_enable_stdio_uart(PicotoFlash 0)

# Generate UF2, bin, etc.
pico_add_extra_outputs(PicotoFlash)

# Link libraries
target_link_libraries(PicotoFlash
    pico_stdlib
    hardware_spi
)
