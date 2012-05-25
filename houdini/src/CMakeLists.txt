cmake_minimum_required(VERSION 2.8)
project(vfxgal_houdini_cpp)

message(STATUS "\nFinding houdini...")
find_package(Houdini REQUIRED)
message(STATUS "Found houdini-${HOUDINI_VERSION} at ${HOUDINI_LIBRARY_DIRS}")

if(NOT VFXGAL_NO_TBB)
    if(VFXGAL_USE_HOUDINI_TBB)
        message(STATUS "\nFinding houdini's tbb...")
        get_filename_component(HOUDINI_TOOLKIT_DIR ${HOUDINI_INCLUDE_DIRS} PATH)
        set(TBB_ROOT ${HOUDINI_TOOLKIT_DIR})
        find_package(Tbb REQUIRED)
        message(STATUS "Found tbb-${TBB_VERSION} at ${TBB_LIBRARY_DIRS}")
    endif(VFXGAL_USE_HOUDINI_TBB)
    include_directories(${TBB_INCLUDE_DIRS})
    link_directories(${TBB_LIBRARY_DIRS})
endif(NOT VFXGAL_NO_TBB)


add_definitions(${HOUDINI_DEFINITIONS})
add_definitions(-DVFXGAL_HOU_MAJOR_VER=${HOUDINI_MAJOR_VERSION})
include_directories(${HOUDINI_INCLUDE_DIRS})
link_directories(${HOUDINI_LIBRARY_DIRS})
include_directories(.)

set(_HLIBS HoudiniGEO HoudiniUT HoudiniPRM)
set(_LIBS ${_VFXGAL_LIBS} ${_HLIBS})

FILE(GLOB_RECURSE cpp_files "util/*.cpp")
add_library(vfxgal_houdini SHARED ${cpp_files})
target_link_libraries(vfxgal_houdini ${_LIBS})

macro(add_hou_dso name cppfile)
    add_library(${name} SHARED ${cppfile})
    target_link_libraries(${name} ${_LIBS} vfxgal_houdini)
    set_target_properties(${name} PROPERTIES PREFIX "")
endmacro(add_hou_dso name)

add_hou_dso(SOP_vfxgal_planeclip        SOP_PlaneClip.cpp)
add_hou_dso(SOP_vfxgal_hullclip         SOP_HullClip.cpp)
add_hou_dso(SOP_vfxgal_voronoifracture  SOP_VoronoiFracture.cpp)
add_hou_dso(SOP_vfxgal_cleanmesh        SOP_CleanMesh.cpp)
add_hou_dso(SOP_vfxgal_addedges         SOP_AddEdges.cpp)

install(TARGETS vfxgal_houdini DESTINATION lib)

install(
    TARGETS
    SOP_vfxgal_planeclip
    SOP_vfxgal_hullclip
    SOP_vfxgal_voronoifracture
    SOP_vfxgal_cleanmesh
    SOP_vfxgal_addedges
    DESTINATION houdini/dso
)


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



