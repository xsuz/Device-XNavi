cmake_minimum_required(VERSION 3.15)

project(SEGGER LANGUAGES CXX)

add_library(SEGGER
    src/SEGGER_RTT.c
    src/SEGGER_RTT_printf.c
)

target_include_directories(SEGGER
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/Core/Inc
)

target_compile_features(SEGGER
    PRIVATE cxx_std_11
)