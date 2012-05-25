cmake_minimum_required(VERSION 2.8)
project(vfxgal_hip)

install(
    DIRECTORY .
    DESTINATION houdini/hip
    PATTERN CMake* EXCLUDE
)
