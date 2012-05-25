cmake_minimum_required(VERSION 2.8)
project(vfxgal_hip)

install(
    DIRECTORY .
    DESTINATION houdini/hip
    PATTERN CMake* EXCLUDE
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
