#ifndef _DGAL_MESH__H_
#define _DGAL_MESH__H_

#include "adaptors/mesh.hpp"
#include "simple_mesh.hpp"
#include <boost/unordered_map.hpp>
#include <boost/utility.hpp>


namespace vfxgal {

	/*
	 * @brief strip_unused_points
	 * Make a copy of the given mesh, with any unused points removed.
	 */
	// todo this should go into algorithms
	// todo need to add point_remapping.
	template<typename Mesh>
	void strip_unused_points(const Mesh& mesh,
		simple_mesh<typename mesh_adaptor<Mesh>::point_type>& result);


///////////////////////// impl

template<typename Mesh>
void strip_unused_points(const Mesh& mesh,
	simple_mesh<typename mesh_adaptor<Mesh>::point_type>& result)
{
	typedef mesh_adaptor<Mesh>							Adaptor;
	typedef typename Adaptor::point_type				point_type;
	typedef typename Adaptor::poly_type					poly_type;
	typedef points_adaptor<poly_type>					PointsAdaptor;
	typedef simple_mesh<point_type>						simple_mesh_type;
	typedef typename simple_mesh_type::polygon_type		simple_polygon_type;

	typedef boost::unordered_map<unsigned int, unsigned int> u_u_map;

	Adaptor a(mesh);
	u_u_map remapping;
	result.m_points.clear();

	unsigned int npolys = a.numPolys();
	for(unsigned int i=0; i<npolys; ++i)
	{
		PointsAdaptor pa(a.getPoly(i));
		for(unsigned int j=0; j<pa.size(); ++j)
		{
			unsigned int ptNum = pa.index(j);
			if(remapping.find(ptNum) == remapping.end())
			{
				remapping.insert(u_u_map::value_type(ptNum, result.m_points.size()));
				result.m_points.push_back(a.getPoint(ptNum));
			}
		}
	}

	result.m_polys.resize(npolys);
	for(unsigned int i=0; i<npolys; ++i)
	{
		PointsAdaptor pa(a.getPoly(i));
		simple_polygon_type& poly = result.m_polys[i];
		poly.resize(pa.size());

		for(unsigned int j=0; j<pa.size(); ++j)
			poly[j] = remapping.find(pa.index(j))->second;
	}
}

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
