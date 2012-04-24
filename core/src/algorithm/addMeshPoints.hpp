#ifndef _DGAL_ADD_MESH_POINTS__H_
#define _DGAL_ADD_MESH_POINTS__H_

#include "../adaptors/mesh_points.hpp"
#include "../EdgeIntersection.hpp"
#include "../edge.hpp"
#include "../simple_mesh.hpp"
#include <boost/unordered_map.hpp>
#include <boost/type_traits/is_same.hpp>
#include <iterator>
#include <set>

namespace vfxgal {

	/*
	 * @brief addMeshPoints
	 * Adds points to edges of the given mesh.
	 * @param mesh The mesh to add points to.
	 * @param[out] result The resulting mesh.
	 * @param pointsBegin, pointsEnd Points to add.
	 * @param[out] point_remapping See remapMesh.MeshRemapSettings.pointMapping.
	 * @param[out] poly_remapping See remapMesh.MeshRemapSettings.polyMapping. Note that
	 * faces which have point(s) added will be given a negative ID, and _no_ face will be
	 * given a zero id.
	 * @returns The number of points added to the mesh. Invalid points are silently skipped.
	 * @note Points can be added between two existing points that are not connected by an
	 * edge, but you'll just end up with a floating point as a result.
	 * @note New points are appended to the existing list of points, and poly order is
	 * unchanged. You'll still need the poly remapping however, since polys that have
	 * gained a point or more will need to have their vertex data remapped.
	 */
	template<typename Mesh, typename EdgeIntersectionConstIterator>
	unsigned int addMeshPoints(const Mesh& mesh,
		simple_mesh<typename mesh_adaptor<Mesh>::point_type>& result,
		EdgeIntersectionConstIterator pointsBegin,
		EdgeIntersectionConstIterator pointsEnd,
		std::vector<int>* point_remapping = NULL,
		std::vector<int>* poly_remapping = NULL);


///////////////////////// impl

template<typename Mesh, typename EdgeIntersectionConstIterator>
unsigned int addMeshPoints(const Mesh& mesh,
	simple_mesh<typename mesh_adaptor<Mesh>::point_type>& result,
	EdgeIntersectionConstIterator pointsBegin,
	EdgeIntersectionConstIterator pointsEnd,
	std::vector<int>* point_remapping,
	std::vector<int>* poly_remapping)
{
	typedef mesh_adaptor<Mesh>										Adaptor;
	typedef typename Adaptor::index_type							index_type;
	typedef typename Adaptor::scalar								scalar_type;
	typedef typename Adaptor::point_type							point_type;
	typedef typename Adaptor::const_point_ref						const_point_ref;
	typedef EdgeIntersection<index_type,scalar_type>				edge_intersection;

	typedef typename Adaptor::poly_type								poly_type;
	typedef points_adaptor<poly_type>								PointsAdaptor;

	typedef bidirectional_edge<index_type>							bi_edge;
	typedef std::pair<scalar_type, index_type>						mapped_intersection;
	typedef std::set<mapped_intersection>							mapped_intersections;
	typedef boost::unordered_map<bi_edge, mapped_intersections>		new_pt_map;

	BOOST_MPL_ASSERT((boost::is_same<edge_intersection,
		typename std::iterator_traits<EdgeIntersectionConstIterator>::value_type>));

	Adaptor a(mesh);
	unsigned int npoints = a.numPoints();
	unsigned int npolys = a.numPolys();

	// prepare output
	result.clear();
	result.m_points.resize(npoints);
	result.m_polys.resize(npolys);

	// create directly-mapped points
	unsigned int nnew_pts = 0;
	new_pt_map new_pts;

	for(unsigned int i=0; i<npoints; ++i)
		result.m_points[i] = a.getPoint(i);

	if(point_remapping)
	{
		point_remapping->resize(npoints);
		for(unsigned int i=0; i<npoints; ++i)
			(*point_remapping)[i] = i;
	}

	// append new points
	{
		mesh_points<Mesh> meshPts(mesh);

		for(EdgeIntersectionConstIterator it=pointsBegin; it!=pointsEnd; ++it)
		{
			if((it->m_point1 >= npoints) || (it->m_point2 >= npoints))
				continue;

			unsigned int newpt_index = result.m_points.size();
			result.m_points.push_back(point_type());
			it->getPos(meshPts, result.m_points.back());

			bi_edge e(it->m_point1, it->m_point2);
			mapped_intersections& mi = new_pts[e];
			scalar_type frac = (e.first() == it->m_point1)? it->m_u : 1-it->m_u;
			mi.insert(mapped_intersection(frac, newpt_index));

			if(point_remapping)
				point_remapping->push_back(-1);

			++nnew_pts;
		}
	}

	// create polys
	if(poly_remapping)
		poly_remapping->resize(npolys);

	for(unsigned int i=0; i<npolys; ++i)
	{
		PointsAdaptor srcPoly(a.getPoly(i));
		std::vector<index_type>& destPoly = result.m_polys[i];

		destPoly.clear();
		bool added_verts = false;

		unsigned int nverts = srcPoly.size();
		for(unsigned int j=0; j<nverts; ++j)
		{
			unsigned int pt1 = srcPoly.index(j);
			unsigned int pt2 = srcPoly.index((j+1)%nverts);

			destPoly.push_back(pt1);

			bi_edge e(pt1, pt2);
			typename new_pt_map::const_iterator it = new_pts.find(e);

			if(it != new_pts.end())
			{
				added_verts = true;
				const mapped_intersections& mis = it->second;
				bool reverse = (e.first() != pt1);

				if(reverse)
				{
					for(typename mapped_intersections::const_reverse_iterator it2=mis.rbegin();
						it2!=mis.rend(); ++it2)
						destPoly.push_back(it2->second);
				}
				else
				{
					for(typename mapped_intersections::const_iterator it2=mis.begin();
						it2!=mis.end(); ++it2)
						destPoly.push_back(it2->second);
				}
			}
		}

		if(poly_remapping)
			(*poly_remapping)[i] = (added_verts)? -1-i : i+1;
	}

	return nnew_pts;
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
