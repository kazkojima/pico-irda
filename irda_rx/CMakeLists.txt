add_executable(pio_irda_rx)

pico_generate_pio_header(pio_irda_rx ${CMAKE_CURRENT_LIST_DIR}/irda_rx.pio)

target_sources(pio_irda_rx PRIVATE irda_rx.c)

target_link_libraries(pio_irda_rx PRIVATE
        pico_stdlib
        pico_multicore
        hardware_pio
	hardware_dma
        )

# enable usb output, disable uart output
pico_enable_stdio_usb(pio_irda_rx 1)
pico_enable_stdio_uart(pio_irda_rx 0)

pico_add_extra_outputs(pio_irda_rx)

# add url via pico_set_program_url
#example_auto_set_url(pio_irda_rx)
