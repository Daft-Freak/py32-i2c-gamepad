cmake_minimum_required(VERSION 3.12)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

project(py32-test C CXX ASM)

set(CMAKE_EXECUTABLE_SUFFIX ".elf")

add_library(py32-startup INTERFACE)
target_sources(py32-startup INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/startup_py32f002a.s
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/CMSIS/Device/PY32F0xx/Source/system_py32f0xx.c
)
target_include_directories(py32-startup INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/CMSIS/Core/Include
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/CMSIS/Device/PY32F0xx/Include
)

add_executable(py32-test main.cpp)
target_link_libraries(py32-test py32-startup)
target_compile_options(py32-test PRIVATE -Wall -Wextra)