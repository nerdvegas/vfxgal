cmake_minimum_required(VERSION 2.8)
project(vfxgal_test)


if(NOT VFXGAL_NO_TBB)
    include_directories(${TBB_INCLUDE_DIRS})
    link_directories(${TBB_LIBRARY_DIRS})
endif(NOT VFXGAL_NO_TBB)

    
add_executable(vfxgal_test test_main.cpp)
target_link_libraries(vfxgal_test ${_VFXGAL_LIBS})


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
