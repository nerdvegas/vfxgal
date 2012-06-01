cmake_minimum_required(VERSION 2.8)
project(vfxgal)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
include(GetCXXCompiler)
include(GetBoost)

set(BOOST_ROOT $ENV{BOOST_ROOT})
set(ILMBASE_ROOT $ENV{ILMBASE_ROOT})
set(TBB_ROOT $ENV{TBB_ROOT})
set(HOUDINI_ROOT $ENV{HOUDINI_ROOT})
set(QHULL_ROOT $ENV{QHULL_ROOT})


####
#### USER EDIT BEGIN HERE...
####
# Set paths to other software here
#set(BOOST_ROOT /wherever/your/boost/is/installed)
#set(ILMBASE_ROOT /wherever/your/ilmbase/is/installed)
#set(TBB_ROOT /wherever/your/tbb/is/installed)
#set(HOUDINI_ROOT /wherever/your/houdini/is/installed) # OR just add houdini to $PATH
#set(QHULL_ROOT /wherever/your/qhull/is/installed) # OR just add qhull to $PATH

# Set options to ON/OFF here
option(VFXGAL_STATIC_LINK_BOOST "Statically link to boost libs" OFF)
option(VFXGAL_ENSURE_BOOST_VERSION_MATCH "Make sure system and app boost versions match" ON)
option(VFXGAL_NO_TBB "Disable multithreading" OFF)
option(VFXGAL_BUILD_TEST "Build unit test" ON)
# for voronoi fracturing...
option(VFXGAL_QHULL_REQUIRED "Do not install unless qhull can be found" ON)
# houdini only...
option(VFXGAL_BUILD_HOUDINI "Build houdini binding" ON)
option(VFXGAL_USE_HOUDINI_TBB "Defer to houdini tbb" ON)
####
#### USER EDIT END HERE.
####



set(VFXGAL_VERSION_MAJOR 1)
set(VFXGAL_VERSION_MINOR 0)
set(VFXGAL_VERSION_PATCH 0)
set(VFXGAL_VERSION ${VFXGAL_VERSION_MAJOR}.${VFXGAL_VERSION_MINOR}.${VFXGAL_VERSION_PATCH})
message(STATUS "Building vfxgal-${VFXGAL_VERSION}...")


# Find Dependencies
###############################################################################

GetCXXCompiler(VFXGAL)

message(STATUS "\nFinding boost...")
if(VFXGAL_STATIC_LINK_BOOST)
    set(Boost_USE_STATIC_LIBS ON)
endif(VFXGAL_STATIC_LINK_BOOST)
GetBoost(1.36 COMPONENTS system REQUIRED)
set(_boost_VER_1 ${Boost_NORMALIZED_VERSION})
message(STATUS "Found boost-${Boost_NORMALIZED_VERSION} at ${Boost_LIBRARY_DIRS}")

message(STATUS "\nFinding ilmbase...")
find_package(ILMBase 1.0.2 REQUIRED)
message(STATUS "Found ilmbase-${ILMBASE_VERSION} at ${ILMBASE_LIBRARY_DIRS}")

message(STATUS "\nFinding qhull...")
find_package(QHull 2012.1)
if(QHULL_FOUND)
    message(STATUS "Found qhull-${QHULL_VERSION} at ${QHULL_BINARY}")
else(QHULL_FOUND)
    if(VFXGAL_QHULL_REQUIRED)
        message(FATAL_ERROR "Qhull not found! Vfxgal can be installed without it, but voronoi fracturing will be unavailable. To do this, change the VFXGAL_QHULL_REQUIRED option in the topmost CMakeLists.txt to OFF")
    else(VFXGAL_QHULL_REQUIRED)
        message(WARNING "Qhull not found! Voronoi fracturing will be unavailable until qhull is installed")
    endif(VFXGAL_QHULL_REQUIRED)
endif(QHULL_FOUND)

if(VFXGAL_NO_TBB)
    message(STATUS "\nSkipping tbb...")
    add_definitions(-DVFXGAL_NO_TBB)
elseif(VFXGAL_BUILD_TEST OR NOT VFXGAL_USE_HOUDINI_TBB)
    message(STATUS "\nFinding tbb...")
    find_package(Tbb REQUIRED)
    message(STATUS "Found tbb-${TBB_VERSION} at ${TBB_LIBRARY_DIRS}")
endif(VFXGAL_NO_TBB)


# Include Dependencies
###############################################################################

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${ILMBASE_INCLUDE_DIRS})

link_directories(${Boost_LIBRARY_DIRS})
link_directories(${ILMBASE_LIBRARY_DIRS})

set(_VFXGAL_LIBS vfxgal_pystring ${Boost_LIBRARIES})
if(NOT VFXGAL_NO_TBB)
    set(_VFXGAL_LIBS ${_VFXGAL_LIBS} tbb)
endif(NOT VFXGAL_NO_TBB)


# Build Subdirs
###############################################################################

add_subdirectory(core/vfxgal)
include_directories(core)

add_subdirectory(contrib/pystring)
include_directories(contrib/pystring)
link_directories(${CMAKE_SOURCE_DIR}/contrib/pystring)

include_directories(contrib/boost_process)

if(VFXGAL_BUILD_TEST)
    add_subdirectory(core/test)
endif(VFXGAL_BUILD_TEST)
if(VFXGAL_BUILD_HOUDINI)
    add_subdirectory(houdini)
endif(VFXGAL_BUILD_HOUDINI)


#
# Copyright 2012, Allan Johns
#
# This file is part of vfxgal.
#
# vfxgal is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# vfxgal is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with vfxgal.  If not, see <http://www.gnu.org/licenses/>.
#




