cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(pico_irda_rx)

pico_sdk_init()

add_subdirectory(irda_rx)
