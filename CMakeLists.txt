cmake_minimum_required(VERSION 3.16)
project(stm32_dsp_vibration_spectral_analyzer C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

include_directories(include)

# Host Simulation Executable
add_executable(dsp_demo
    src/dma_pingpong.c
    src/fft_engine.c
    src/vibration_analyzer.c
    src/main.c
)
target_link_libraries(dsp_demo m)

# Automated Unit Tests
enable_testing()
add_executable(dsp_tests
    src/dma_pingpong.c
    src/fft_engine.c
    src/vibration_analyzer.c
    tests/unity/unity.c
    tests/test_dsp_suite.c
)
target_include_directories(dsp_tests PRIVATE tests/unity)
target_link_libraries(dsp_tests m)
add_test(NAME DSP_Spectral_Tests COMMAND dsp_tests)
