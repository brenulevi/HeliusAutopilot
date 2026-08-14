cmake_minimum_required(VERSION 3.22)

# -----------------------------------------------------------------------------
# Build target selection (extend STRINGS as new platforms are added)
# -----------------------------------------------------------------------------
set(HELIUS_TARGET "STM32" CACHE STRING "Helius build target platform")
set_property(CACHE HELIUS_TARGET PROPERTY STRINGS "STM32")

if(HELIUS_TARGET STREQUAL "STM32")
    if(NOT CMAKE_TOOLCHAIN_FILE)
        set(CMAKE_TOOLCHAIN_FILE
            "${CMAKE_CURRENT_SOURCE_DIR}/Firmware/cmake/gcc-arm-none-eabi.cmake"
            CACHE FILEPATH "STM32 GCC toolchain file" FORCE)
    endif()
else()
    message(FATAL_ERROR "Unsupported HELIUS_TARGET='${HELIUS_TARGET}'. Supported: STM32")
endif()

project(HeliusAutopilot
    VERSION 0.1.0
    LANGUAGES C ASM
)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

message(STATUS "Helius target: ${HELIUS_TARGET}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# Shared autopilot core (static library)
add_subdirectory(Core)

if(HELIUS_TARGET STREQUAL "STM32")
    add_subdirectory(Firmware)
    # Plain signature required: CubeMX already linked Firmware without keywords
    target_link_libraries(Firmware helius_core)

    find_program(STM32_PROGRAMMER_CLI
        NAMES STM32_Programmer_CLI
        PATHS
            "$ENV{HOME}/.local/share/stm32cube/bundles/programmer"
            "/opt/st"
        PATH_SUFFIXES
            bin
            "2.23.0/bin"
            "tools/bin"
        DOC "STM32CubeProgrammer CLI for ST-Link flashing"
    )

    if(STM32_PROGRAMMER_CLI)
        add_custom_target(flash
            COMMAND ${STM32_PROGRAMMER_CLI}
                -c port=SWD
                -w $<TARGET_FILE:Firmware>
                -v
                -rst
            DEPENDS Firmware
            COMMENT "Flashing Firmware.elf via ST-Link (SWD)"
            USES_TERMINAL
            VERBATIM
        )
        message(STATUS "Flash tool: ${STM32_PROGRAMMER_CLI}")
    else()
        message(WARNING "STM32_Programmer_CLI not found; 'flash' target disabled")
    endif()
endif()
