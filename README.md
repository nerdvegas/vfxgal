vfxgal
======

Geometric algorithms library for the VFX industry.

core/
-----
This directory contains the majority of the implementation. Vfxgal is primarily a template-based, generic geometric
algorithm library. It provides functions such as - mesh/plane clipping (with closure); mesh/convex hull clipping;
removal of degenerate mesh vertices; mesh edge addition; and mesh voronoi cell fracturing.

Vfxgal functions use an adaptor pattern, which means that you can write a small adaptor class (see core/adaptors),
and then you will be able to input your own data types directly into the vfxgal algorithms. For eg, at the time of
writing vfxgal ships with houdini-11 adaptors. All you need to do is include core/adaptors/houdini.hpp, and then you
can pass houdini types directly into the various vfxgal algorithms.

Dependencies: boost, tbb, qhull (cmdlin only, for voronoi only).
Dependencies shipped with the code: pystring, boost.process.

houdini/
--------
This is the houdini binding for vfxgal. Various geometric functions are implemented as SOP nodes. Only Houdini 11 is 
supported at the moment - because of hou12's geo API changes, new bindings have to be written for it.



Copyright 2012, Allan Johns

vfxgal is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

vfxgal is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with vfxgal.  If not, see <http://www.gnu.org/licenses/>.

