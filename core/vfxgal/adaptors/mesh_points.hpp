#ifndef _DGAL_ADAPTORS_MESH_POINTS__H_
#define _DGAL_ADAPTORS_MESH_POINTS__H_

#include "points.hpp"
#include "mesh.hpp"


namespace vfxgal {

	/*
	 * @class mesh_points
	 * @brief
	 * This class and the following points_adaptor specialization allow us to use a
	 * mesh's point list directly as a point source.
	 */
	template<typename Mesh>
	struct mesh_points
	{
		typedef mesh_adaptor<Mesh> 					Adaptor;
		typedef typename Adaptor::point_type		elem_type;
		typedef typename Adaptor::const_point_ref	const_elem_ref;
		typedef typename Adaptor::scalar			scalar;

		mesh_points(const Mesh& m):m_a(m){}

		inline unsigned int size() const 						{ return m_a.numPoints(); }
		inline const_elem_ref operator[](unsigned int i) const 	{ return m_a.getPoint(i); }
		inline unsigned int index(unsigned int i) const 		{ return i; }
		inline bool is_indexed() const 							{ return false; }

		Adaptor m_a;
	};


	template<typename Mesh>
	struct points_adaptor<mesh_points<Mesh> >
	: public detail::custom_points_adaptor<mesh_points<Mesh> >
	{
		typedef mesh_points<Mesh> T;
		points_adaptor(const T& t):detail::custom_points_adaptor<T>(t){}
	};

}

#endif










/***
    Copyright 2008-2012 Dr D Studios Pty Limited (ACN 127 184 954) (Dr. D Studios)

    This file is part of vfxgal.

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
***/
