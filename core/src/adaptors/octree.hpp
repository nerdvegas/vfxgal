#ifndef _DGAL_ADAPTORS_OCTREE__H_
#define _DGAL_ADAPTORS_OCTREE__H_

#include "../spatials/octree.hpp"
#include "mesh.hpp"

namespace vfxgal {

	/*
	 * @brief Octree mesh specialization. Note that only the zeroeth mesh in an octree
	 * is currently supported.
	 */
	template<typename DataSet>
	struct mesh_adaptor<Octree<DataSet> >
	{
		typedef Octree<DataSet>						target_type;
		typedef DataSet								dataset_type;
		typedef typename DataSet::mesh_type			Mesh3D;

		typedef mesh_adaptor<Mesh3D>				Adaptor;
		typedef typename Adaptor::scalar			scalar;
		typedef typename Adaptor::index_type		index_type;
		typedef typename Adaptor::point_type		point_type;
		typedef typename Adaptor::const_point_ref	const_point_ref;
		typedef typename Adaptor::poly_type			poly_type;
		typedef typename Adaptor::const_poly_ref	const_poly_ref;

		mesh_adaptor(const target_type& t)
		: m_a(t.datasets()[0]->mesh()){}

		inline unsigned int numPoints() const { return m_a.numPoints(); }
		inline unsigned int numPolys() const { return m_a.numPolys(); }
		inline const_point_ref getPoint(unsigned int i) const { return m_a.getPoint(i); }
		inline const_poly_ref getPoly(unsigned int i) const { return m_a.getPoly(i); }

		Adaptor m_a;
	};


	template<typename T>
	struct is_octree : public boost::mpl::false_{};

	template<typename T>
	struct is_octree<Octree<T> > : public boost::mpl::true_{};


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
