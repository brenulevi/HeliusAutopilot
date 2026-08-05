cmake_minimum_required(VERSION 3.10)

project(Helius)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_C_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(MSVC)
    set(HELIUS_WARNING_FLAGS /W4 /we4013)
else()
    set(HELIUS_WARNING_FLAGS
        -Wall
        -Wextra
        -Wno-unused-parameter
        -Werror=implicit-function-declaration
    )
endif()

set(BUILD_TARGET "STM32" CACHE STRING "Build target for the project")
set_property(CACHE BUILD_TARGET PROPERTY STRINGS
    JSBSIM
    STM32
)

add_subdirectory(core)

if(BUILD_TARGET STREQUAL "JSBSIM")
    add_subdirectory(simulation/jsbsim)
elseif(BUILD_TARGET STREQUAL "STM32")
    add_subdirectory(firmware)
endif()