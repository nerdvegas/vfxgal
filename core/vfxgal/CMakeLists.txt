cmake_minimum_required(VERSION 2.8)
project(vfxgal_core)

install(
    DIRECTORY .
    DESTINATION include/vfxgal
    PATTERN CMake* EXCLUDE
)
