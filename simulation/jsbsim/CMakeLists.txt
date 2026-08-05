cmake_minimum_required(VERSION 3.10)

set(BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(BUILD_PYTHON_MODULE OFF CACHE BOOL "" FORCE)
set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_PLAYGROUND OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)

add_subdirectory(vendor/jsbsim EXCLUDE_FROM_ALL)

add_executable(jsbsim_simulation
    "src/main.cpp"
)

target_include_directories(jsbsim_simulation
    PRIVATE "vendor/jsbsim/src"
)

target_link_libraries(jsbsim_simulation
    PUBLIC helius
    PRIVATE libJSBSim
)

add_custom_command(TARGET jsbsim_simulation POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/resources/aircraft
        $<TARGET_FILE_DIR:jsbsim_simulation>/aircraft
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/resources/engine
        $<TARGET_FILE_DIR:jsbsim_simulation>/engine
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/resources/systems
        $<TARGET_FILE_DIR:jsbsim_simulation>/systems
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/resources/data_output
        $<TARGET_FILE_DIR:jsbsim_simulation>/data_output
)