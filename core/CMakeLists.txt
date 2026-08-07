cmake_minimum_required(VERSION 3.10)

add_library(helius STATIC
    "src/estimator.c"
    "src/quat.c"
    "src/vec3.c"
    "src/mat3.c"
)

target_include_directories(helius
    PUBLIC "include/"
    PRIVATE "include/helius"
    PRIVATE "include/helius/math"
)