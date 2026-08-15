add_library(helius_core STATIC
    src/math/vec3.c
    src/math/quat.c
    src/math/mat6.c
    src/math/mat3.c

    src/ahrs.c
)

add_library(helius::core ALIAS helius_core)

target_include_directories(helius_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include/helius
        ${CMAKE_CURRENT_SOURCE_DIR}/include/helius/math
)

target_link_libraries(helius_core
    PUBLIC
        m
)

target_compile_features(helius_core
    PUBLIC
        c_std_11
)
