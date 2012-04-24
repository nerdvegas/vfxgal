#ifndef _DGAL_VORONOI_CELL__H_
#define _DGAL_VORONOI_CELL__H_

#include "points.hpp"
#include "adaptors/points_indexer.hpp"
#include <vector>


namespace vfxgal {

	/*
	 * @class VoronoiCell
	 * @brief A voronoi cell. The vertices in this cell reference some external point list.
	 * @var m_isBound True if this cell includes the infinity vertex, false otherwise.
	 * @var m_bounds The bounds of the vertices of the cell, not including the infinity
	 * vertex. If this cell is bound, then m_bounds is the bounding box of the entire cell.
	 * @var m_faces The faces of the cell, not including faces that use the infinity vertex.
	 * @var m_cutPlanes The cut planes, whos sum is the same as the cell's convex hull.
	 */
	template<typename T>
	struct VoronoiCell3D
	{
		typedef Imath::Vec3<T>				vec3_type;
		typedef Imath::Plane3<T>			plane3_type;
		typedef Imath::Box<vec3_type>		box_type;
		typedef std::vector<unsigned int>	face_type;

		bool m_isBound;
		box_type m_bounds;
		std::vector<face_type> m_faces;
		std::vector<plane3_type> m_cutPlanes;
	};

}

#endif




















/***
    Copyright 2008-2012 Dr D Studios Pty Limited (ACN 127 184 954) (Dr. D Studios)

    This file is part of anim-studio-tools.

    anim-studio-tools is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    anim-studio-tools is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with anim-studio-tools.  If not, see <http://www.gnu.org/licenses/>.
***/
