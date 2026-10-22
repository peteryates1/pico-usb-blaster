cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(pico_usb_blaster)

add_compile_options(-O3)
add_compile_options(-Wall)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# rest of your project

add_executable(pico_usb_blaster
    main.c
    usb_descriptors.c
    blaster.c
    cdc_uart.c
    ws2812.c
)

pico_generate_pio_header(pico_usb_blaster ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)
pico_generate_pio_header(pico_usb_blaster ${CMAKE_CURRENT_LIST_DIR}/blaster_jtag.pio)

# Board / harness pin profile — see board_config.h.
#   0 = flying leads, 1 = pico-usb-debug-jtag JTAG1, 2 = pico-usb-debug-jtag JTAG0
if(DEFINED BOARD_TYPE)
    target_compile_definitions(pico_usb_blaster PRIVATE BOARD_TYPE=${BOARD_TYPE})
endif()

# On the carrier (1, 2, and the default, which is 1) GP0/GP1 are J4, a
# pico-debug UART, so no stdio UART console there (blaster.c checks this)
if(NOT DEFINED BOARD_TYPE OR BOARD_TYPE EQUAL 1 OR BOARD_TYPE EQUAL 2)
    pico_enable_stdio_uart(pico_usb_blaster 0)

    # The carrier's J2 debug probe (dap_swd.c, HAS_J2_SWD in board_config.h):
    # ARM's CMSIS-DAP command handling and debugprobe's PIO SWD layer, both
    # from raspberrypi/debugprobe debugprobe-v2.3.1 -- see cmsis_dap/README.md
    target_sources(pico_usb_blaster PRIVATE
        dap_swd.c
        probe.c
        sw_dp_pio.c
        cmsis_dap/DAP.c
        cmsis_dap/DAP_vendor.c
    )
    target_include_directories(pico_usb_blaster PRIVATE cmsis_dap)
    pico_generate_pio_header(pico_usb_blaster ${CMAKE_CURRENT_LIST_DIR}/probe.pio)
    # cmsis_compiler.h only: cmsis_core itself would add the SDK's system_RP2040.c
    target_link_libraries(pico_usb_blaster PRIVATE cmsis_core_headers)
endif()

# JTAG clock divider: TCK = sys_clk / (JTAG_CLKDIV * 4). 5 = 6 MHz, 40 = 750 kHz.
if(DEFINED JTAG_CLKDIV)
    target_compile_definitions(pico_usb_blaster PRIVATE BLASTER_JTAG_CLKDIV=${JTAG_CLKDIV})
endif()

# Diagnostic build: invert the TDO pull to tell a floating pad from a driven one
if(TDO_DIAG_PULLDOWN)
    target_compile_definitions(pico_usb_blaster PRIVATE TDO_DIAG_PULLDOWN)
endif()

# Print the active pin/clock config at configure time. These are cache
# variables: re-running cmake WITHOUT -DTDO_DIAG_PULLDOWN does not clear a
# previously set one, so a stale diagnostic build is easy to flash by accident.
message(STATUS "blaster: BOARD_TYPE=${BOARD_TYPE} JTAG_CLKDIV=${JTAG_CLKDIV} "
               "TDO_DIAG_PULLDOWN=${TDO_DIAG_PULLDOWN}")
if(TDO_DIAG_PULLDOWN)
    message(WARNING "blaster: DIAGNOSTIC build — TDO pull is INVERTED. "
                    "Configure with -DTDO_DIAG_PULLDOWN=0 for a normal image.")
endif()

# Make sure TinyUSB can find tusb_config.h
target_include_directories(pico_usb_blaster PUBLIC
    .
)

target_link_libraries(pico_usb_blaster PUBLIC 
    pico_stdlib 
    pico_unique_id 
    tinyusb_device 
    tinyusb_board
    pico_platform
    hardware_pio
    hardware_uart
    hardware_irq
)

pico_set_binary_type(pico_usb_blaster copy_to_ram)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(pico_usb_blaster)