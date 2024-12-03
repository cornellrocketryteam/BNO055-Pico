add_library(BNO055-Pico INTERFACE)

target_sources(BNO055-Pico INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/bno055.cpp
)

target_link_libraries(BNO055-Pico INTERFACE
    pico_stdlib
    hardware_i2c
)
