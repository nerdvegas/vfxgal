#ifndef _DGAL_CLEAN_MESH__H_
#define _DGAL_CLEAN_MESH__H_

#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include "../adaptors/mesh.hpp"
#include "../adaptors/points.hpp"
#include "../simple_mesh.hpp"

namespace dgal {

	/*
	 * @brief cleanMesh
	 * Removes degenerate edges and polys, and unused points, from the mesh.
	 * @param mesh The mesh to clean.
	 * @param[out] result The resulting mesh.
	 * @param minEdgeLength Edges shorter than this will collapse into a single vertex. This
	 * may create degenerate faces, but these will be removed also.
	 * @param[out] point_remapping Remapping of points onto resultant mesh. Note that <0
	 * results will never be returned - ints are only used in order to be compatible with
	 * the remapMesh() function.
	 * @param[out] poly_remapping Remapping of polygons onto resultant mesh. See
	 * ClippedPoly.m_polyId - the same ID remapping scheme is used here, but a value of
	 * zero will never be returned. Negative values are returned where a poly has lost one
	 * or more vertices. As in point_remapping, this ID scheme is used for compatibility
	 * with the remapMesh() function.
	 */
	template<typename Mesh>
	void cleanMesh(const Mesh& mesh,
		simple_mesh<typename mesh_adaptor<Mesh>::point_type>& result,
		typename mesh_adaptor<Mesh>::scalar minEdgeLength =
			std::numeric_limits<typename mesh_adaptor<Mesh>::scalar>::epsilon(),
		std::vector<int>* point_remapping = NULL,
		std::vector<int>* poly_remapping = NULL);


///////////////////////// impl

template<typename Mesh>
void cleanMesh(const Mesh& mesh,
	simple_mesh<typename mesh_adaptor<Mesh>::point_type>& result,
	typename mesh_adaptor<Mesh>::scalar minEdgeLength,
	std::vector<int>* point_remapping,
	std::vector<int>* poly_remapping)
{
	typedef mesh_adaptor<Mesh>						Adaptor;
	typedef typename Adaptor::scalar				T;
	typedef typename Adaptor::point_type			vec_type;
	typedef typename Adaptor::poly_type				poly_type;
	typedef typename Adaptor::const_point_ref		const_point_ref;
	typedef simple_mesh<vec_type>					simple_mesh;
	typedef points_adaptor<poly_type>				PointsAdaptor;

	typedef boost::unordered_map<unsigned int, unsigned int>	u_u_map;
	typedef boost::unordered_set<unsigned int>					u_set;
	typedef std::vector<unsigned int>							u_vec;

	Adaptor a(mesh);
	unsigned int npoints = a.numPoints();
	unsigned int npolys = a.numPolys();
	T epsilon = minEdgeLength*minEdgeLength;

	// prepare output
	result.clear();
	result.m_points.reserve(npoints/2);
	result.m_polys.reserve(npolys/2);
	if(point_remapping)
	{
		point_remapping->clear();
		point_remapping->reserve(npoints/2);
	}
	if(poly_remapping)
	{
		poly_remapping->clear();
		poly_remapping->reserve(npoints/2);
	}

	u_u_map degen_pt_map;
	u_vec pt_islands;
	u_set s;

	// iterate over edges. If an edge is too short, put both its pts into an island. If
	// the pts belong to different islands then merge the islands
	for(unsigned int i=0; i<npolys; ++i)
	{
		PointsAdaptor pa(a.getPoly(i));
		unsigned int nverts = pa.size();
		for(unsigned int j=0; j<nverts; ++j)
		{
			unsigned int k = (j+1) % nverts;
			if((pa[j] - pa[k]).length2() <= epsilon)
			{
				unsigned int pt1 = pa.index(j);
				unsigned int pt2 = pa.index(k);

				u_u_map::const_iterator it1 = degen_pt_map.find(pt1);
				u_u_map::const_iterator it2 = degen_pt_map.find(pt2);
				bool found1 = (it1 != degen_pt_map.end());
				bool found2 = (it2 != degen_pt_map.end());

				if(found1 && found2)
				{
					// merge islands
					pt_islands[it1->second] = pt_islands[it2->second];
				}
				else if(found1 != found2)
				{
					// add new pt to existing island
					if(found1)
						degen_pt_map.insert(u_u_map::value_type(pt2, it1->second));
					else
						degen_pt_map.insert(u_u_map::value_type(pt1, it2->second));
				}
				else
				{
					// create new island containing both pts
					unsigned int n = pt_islands.size();
					pt_islands.push_back(n);
					degen_pt_map.insert(u_u_map::value_type(pt1, n));
					degen_pt_map.insert(u_u_map::value_type(pt2, n));
				}
			}
		}
	}

	u_u_map pt_remap;
	u_u_map pt_island_remap;

	// iterate over polys, building a new point list as we go, and skipping polys that
	// are or have become degenerate
	for(int i=0; i<npolys; ++i)
	{
		PointsAdaptor pa(a.getPoly(i));
		unsigned int nverts = pa.size();
		if(nverts < 3)
			continue;

		// check if poly is degenerate
		unsigned int num_nondegen_pts = 0;
		s.clear();

		for(unsigned int j=0; j<nverts; ++j)
		{
			unsigned int ptnum = pa.index(j);
			u_u_map::const_iterator it = degen_pt_map.find(ptnum);
			if(it == degen_pt_map.end())
				++num_nondegen_pts;
			else
				s.insert(pt_islands[it->second]);
		}

		unsigned int nverts_result = num_nondegen_pts + s.size();
		if(nverts_result < 3)
			continue;

		// add poly, remapping verts at the same time
		result.m_polys.push_back(u_vec());
		u_vec& resultPoly = result.m_polys.back();
		resultPoly.resize(nverts_result);

		if(poly_remapping)
			poly_remapping->push_back((nverts_result == nverts)? i+1 : -1-i);

		unsigned int k = 0;
		s.clear();

		for(unsigned int j=0; j<nverts; ++j)
		{
			unsigned int ptnum = pa.index(j);
			u_u_map::const_iterator it = pt_remap.find(ptnum);
			if(it == pt_remap.end())
			{
				it = degen_pt_map.find(ptnum);
				if(it == degen_pt_map.end())
				{
					unsigned int new_ptnum = result.m_points.size();
					result.m_points.push_back(pa[j]);
					if(point_remapping)
						point_remapping->push_back(ptnum);

					pt_remap.insert(u_u_map::value_type(ptnum, new_ptnum));
					resultPoly[k++] = new_ptnum;
				}
				else
				{
					unsigned int pt_island = pt_islands[it->second];
					if(s.find(pt_island) == s.end())
					{
						it = pt_island_remap.find(pt_island);
						if(it == pt_island_remap.end())
						{
							unsigned int new_ptnum = result.m_points.size();
							result.m_points.push_back(pa[j]);
							if(point_remapping)
								point_remapping->push_back(ptnum);

							pt_island_remap.insert(u_u_map::value_type(pt_island, new_ptnum));
							resultPoly[k++] = new_ptnum;
						}
						else
							resultPoly[k++] = it->second;

						s.insert(pt_island);
					}
				}
			}
			else
				resultPoly[k++] = it->second;
		}

		assert(k == nverts_result);
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
