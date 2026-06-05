cmake_minimum_required(VERSION 3.16)
project(modulation_toolkit VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ─── Dependencies ────────────────────────────────────────────────────────────

# libbladeRF
find_path(BLADERF_INCLUDE_DIR libbladeRF.h
    HINTS /usr/include /usr/local/include)
find_library(BLADERF_LIBRARY
    NAMES bladeRF
    HINTS /usr/lib /usr/local/lib)
if(NOT BLADERF_INCLUDE_DIR OR NOT BLADERF_LIBRARY)
    message(FATAL_ERROR "libbladeRF not found. Install with: sudo apt install libbladerf-dev")
endif()

# liquid-dsp
find_path(LIQUID_INCLUDE_DIR liquid/liquid.h
    HINTS /usr/include /usr/local/include)
find_library(LIQUID_LIBRARY
    NAMES liquid
    HINTS /usr/lib /usr/local/lib)
if(NOT LIQUID_INCLUDE_DIR OR NOT LIQUID_LIBRARY)
    message(FATAL_ERROR
        "liquid-dsp not found.\n"
        "Install with: sudo apt install libliquid-dev\n"
        "Or build from source: https://github.com/jgaeddert/liquid-dsp")
endif()

# SoapySDR
find_path(SOAPY_INCLUDE_DIR SoapySDR/Device.hpp
    HINTS /usr/include /usr/local/include)
find_library(SOAPY_LIBRARY
    NAMES SoapySDR
    HINTS /usr/lib /usr/local/lib)
if(NOT SOAPY_INCLUDE_DIR OR NOT SOAPY_LIBRARY)
    message(FATAL_ERROR
        "SoapySDR not found.\n"
        "Install with: sudo apt install libsoapysdr-dev")
endif()

# ─── Main executable ─────────────────────────────────────────────────────────

add_executable(modulation_toolkit
    src/main.cpp

    src/hw/circular_buffer.cpp
    src/hw/bladerf_device.cpp

    src/modulation/modulator_base.cpp
    src/modulation/demodulator_base.cpp
    src/modulation/rrc_filter.cpp
    src/modulation/am_mod.cpp
    src/modulation/fm_mod.cpp
    src/modulation/bpsk_mod.cpp
    src/modulation/bpsk_demod.cpp
    src/modulation/qpsk_mod.cpp
    src/modulation/qpsk_demod.cpp
    src/modulation/qam_mod.cpp
    src/modulation/qam_demod.cpp
    src/modulation/mm_timing_pll.cpp

    src/fec/conv_encoder.cpp
    src/fec/viterbi_decoder.cpp
    src/fec/interleaver.cpp

    src/ofdm/ofdm_modulator.cpp
    src/ofdm/ofdm_demodulator.cpp
    src/ofdm/resource_grid.cpp
    src/ofdm/channel_estimator.cpp

    src/channel/awgn_channel.cpp
    src/channel/multipath_channel.cpp

    src/cv2x/cv2x_sidelink.cpp
    src/cv2x/resource_pool.cpp

    src/measurement/ber_analyzer.cpp
    src/measurement/evm_meter.cpp
)

target_include_directories(modulation_toolkit PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${BLADERF_INCLUDE_DIR}
    ${LIQUID_INCLUDE_DIR}
    ${SOAPY_INCLUDE_DIR}
)

target_link_libraries(modulation_toolkit PRIVATE
    ${BLADERF_LIBRARY}
    ${LIQUID_LIBRARY}
    ${SOAPY_LIBRARY}
    pthread
)

# ─── Test executable ─────────────────────────────────────────────────────────

add_executable(encoder_validation
    tests/ber_loopback_test.cpp
    src/fec/conv_encoder.cpp
    src/fec/viterbi_decoder.cpp
    src/fec/interleaver.cpp
    src/modulation/modulator_base.cpp
    src/modulation/demodulator_base.cpp
    src/modulation/rrc_filter.cpp
    src/modulation/bpsk_mod.cpp
    src/modulation/bpsk_demod.cpp
    src/modulation/mm_timing_pll.cpp
    src/channel/awgn_channel.cpp
    src/measurement/ber_analyzer.cpp
)
target_include_directories(encoder_validation PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${LIQUID_INCLUDE_DIR}
)

target_link_libraries(encoder_validation PRIVATE
    ${LIQUID_LIBRARY}
    pthread
)

# ─── Build type defaults ─────────────────────────────────────────────────────

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# ─── Compiler flags ──────────────────────────────────────────────────────────

target_compile_options(modulation_toolkit PRIVATE
    -Wall -Wextra -Werror -march=native
    $<$<CONFIG:Release>:-O2>
    $<$<CONFIG:Debug>:-O0>
    $<$<CONFIG:Debug>:-g3>
)

target_compile_options(encoder_validation PRIVATE
    -Wall -Wextra -march=native
    $<$<CONFIG:Release>:-O2>
    $<$<CONFIG:Debug>:-O0>
    $<$<CONFIG:Debug>:-g3>
)

# ─── Sanitizers ──────────────────────────────────────────────────────────────

set(SANITIZE "" CACHE STRING "Sanitizer to enable: address | thread | (empty)")

if(SANITIZE STREQUAL "address")
    message(STATUS "AddressSanitizer enabled")
    target_compile_options(modulation_toolkit PRIVATE -fsanitize=address -fno-omit-frame-pointer)
    target_link_options   (modulation_toolkit PRIVATE -fsanitize=address)
elseif(SANITIZE STREQUAL "thread")
    message(STATUS "ThreadSanitizer enabled")
    target_compile_options(modulation_toolkit PRIVATE -fsanitize=thread -fno-omit-frame-pointer)
    target_link_options   (modulation_toolkit PRIVATE -fsanitize=thread)
elseif(NOT SANITIZE STREQUAL "")
    message(FATAL_ERROR "Unknown sanitizer '${SANITIZE}'. Use 'address' or 'thread'.")
endif()