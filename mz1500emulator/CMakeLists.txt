add_executable(mz1500emulator)

# by default the header is generated into the build dir
pico_generate_pio_header(mz1500emulator ${CMAKE_CURRENT_LIST_DIR}/hsync.pio)
pico_generate_pio_header(mz1500emulator ${CMAKE_CURRENT_LIST_DIR}/vsync.pio)
pico_generate_pio_header(mz1500emulator ${CMAKE_CURRENT_LIST_DIR}/rgb.pio)
# however, alternatively you can choose to generate it somewhere else (in this case in the source tree for check in)
#pico_generate_pio_header(pio_blink ${CMAKE_CURRENT_LIST_DIR}/blink.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR})

# Make sure TinyUSB can find tusb_config.h
target_include_directories(mz1500emulator PUBLIC ${CMAKE_CURRENT_LIST_DIR})

target_sources(mz1500emulator PRIVATE mz1500emulator.c z80.c hid_app.c vga16_graphics.c lfs.c lfs_util.c)

pico_enable_stdio_usb(mz1500emulator 0)
pico_enable_stdio_uart(mz1500emulator 1)

target_link_libraries(mz1500emulator PRIVATE pico_stdlib hardware_pio hardware_timer hardware_dma hardware_uart hardware_flash pico_multicore hardware_pwm tinyusb_host tinyusb_board)
pico_add_extra_outputs(mz1500emulator)
