cmake_minimum_required(VERSION 2.8)
project(vfxgal_test)


if(NOT VFXGAL_NO_TBB)
    include_directories(${TBB_INCLUDE_DIRS})
    link_directories(${TBB_LIBRARY_DIRS})
endif(NOT VFXGAL_NO_TBB)

    
add_executable(vfxgal_test test_main.cpp)
target_link_libraries(vfxgal_test ${_VFXGAL_LIBS})
