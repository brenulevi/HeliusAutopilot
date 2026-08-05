cmake_minimum_required(VERSION 3.10)

add_library(helius STATIC
    "src/estimator.c"
)

target_include_directories(helius
    PUBLIC "include/"
    PRIVATE "include/helius"
)