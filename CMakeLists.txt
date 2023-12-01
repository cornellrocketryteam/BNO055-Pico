add_library(BNO055-Pico INTERFACE)

target_sources(BNO055-Pico INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/bno055.cpp
)

target_link_libraries(BNO055-Pico INTERFACE
    hardware_i2c
    pico_stdlib
)
