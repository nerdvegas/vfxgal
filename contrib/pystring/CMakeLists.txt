cmake_minimum_required(VERSION 2.8)
project(vfxgal_pystring)

add_definitions(-fPIC)
add_library(vfxgal_pystring STATIC pystring.cpp)
