cmake_minimum_required(VERSION 3.10)

add_library(helius STATIC
    src/estimator.c
    src/mat.c
    src/quat.c
    src/vec3.c
)

if(HELIUS_MATH_BACKEND STREQUAL "CMSIS")
    target_sources(helius PRIVATE src/math/mat_backend_cmsis_stub.c)
else()
    target_sources(helius PRIVATE src/math/mat_backend_scalar.c)
endif()

target_include_directories(helius
    PUBLIC "include/"
    PRIVATE "src/"
    PRIVATE "include/helius"
    PRIVATE "include/helius/math"
)

target_compile_options(helius PRIVATE ${HELIUS_WARNING_FLAGS})

if(HELIUS_MATH_BACKEND STREQUAL "CMSIS")
    target_compile_definitions(helius PUBLIC HELIUS_MATH_BACKEND_CMSIS=1)
else()
    target_compile_definitions(helius PUBLIC HELIUS_MATH_BACKEND_SCALAR=1)
endif()
