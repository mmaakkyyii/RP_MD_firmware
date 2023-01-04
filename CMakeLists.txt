cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(RPMD)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()


# rest of your project
add_executable(RPMD
    RPMD.cpp
)

# Create C header file with the name <pio program>.pio.h
pico_generate_pio_header(${PROJECT_NAME}  
${CMAKE_CURRENT_LIST_DIR}/pio_rotary_encoder1.pio
)
pico_generate_pio_header(${PROJECT_NAME}  
${CMAKE_CURRENT_LIST_DIR}/pio_rotary_encoder2.pio
)
# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(
    RPMD
    pico_stdlib
    hardware_pwm
    hardware_adc
    hardware_pio
    pico_multicore
)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(RPMD)