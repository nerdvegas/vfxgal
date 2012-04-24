#ifndef _DGAL_ADAPTORS_MESH__H_
#define _DGAL_ADAPTORS_MESH__H_

#include <vector>
#include <OpenEXR/ImathVec.h>
#include <boost/mpl/bool.hpp>
#include "points_indexer.hpp"


namespace vfxgal {

	/*
	 * @class mesh_adaptor
	 * @brief
	 * Adaptor for plugging various data sources into vfxgal algorithms to represent a mesh.
	 * The default implementation can be templatised on simple_mesh<U>, where U is any
	 * Imath Vec type.
	 */
	template<typename T>
	struct mesh_adaptor
	{
		typedef T											target_type;
		typedef typename T::scalar_type						scalar;
		typedef typename T::index_type						index_type;
		typedef typename T::point_type						point_type;
		typedef const point_type&							const_point_ref;
		typedef points_indexer<typename T::points_vector>	poly_type;
		typedef points_indexer<typename T::points_vector>	const_poly_ref;

		mesh_adaptor(const target_type& t):m_target(t){}

		inline unsigned int numPoints() const { return m_target.m_points.size(); }
		inline unsigned int numPolys() const { return m_target.m_polys.size(); }

		inline const_point_ref getPoint(unsigned int i) const {
			return m_target.m_points[i];
		}

		inline const_poly_ref getPoly(unsigned int i) const {
			const typename T::polygon_type& poly = m_target.m_polys[i];
			return poly_type(&poly[0], &poly[0]+poly.size(), m_target.m_points);
		}

		const target_type& m_target;
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
