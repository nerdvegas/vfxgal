#ifndef _DGAL_ADD_MESH_EDGES__H_
#define _DGAL_ADD_MESH_EDGES__H_

#include "../adaptors/mesh_points.hpp"
#include "addMeshPoints.hpp"
#include "addPolyEdges.hpp"
#include "../MeshConnectivity.hpp"
#include <algorithm>

namespace dgal {

	/*
	 * @brief addMeshEdges
	 * Add edges to the given mesh, splitting existing faces.
	 * @param mesh The mesh to add points to.
	 * @param[out] result The resulting mesh.
	 * @param edgesBegin, edgesEnd Edges to add.
	 * @param[out] poly_remapping See remapMesh.MeshRemapSettings.polyMapping. Note that no
	 * zero-id polys will be generated.
	 * @returns The number of edges added to the mesh. Edges which do not bisect a poly are
	 * silently skipped, as are those that duplicate an existing edge.
	 */
	template<typename Mesh, typename EdgeConstIterator>
	unsigned int addMeshEdges(const Mesh& mesh,
		simple_mesh<typename mesh_adaptor<Mesh>::point_type>& result,
		EdgeConstIterator edgesBegin, EdgeConstIterator edgesEnd,
		std::vector<int>* poly_remapping = NULL);


///////////////////////// impl

template<typename Mesh, typename EdgeConstIterator>
unsigned int addMeshEdges(const Mesh& mesh,
	simple_mesh<typename mesh_adaptor<Mesh>::point_type>& result,
	EdgeConstIterator edgesBegin, EdgeConstIterator edgesEnd,
	std::vector<int>* poly_remapping)
{
	typedef mesh_adaptor<Mesh>									Adaptor;
	typedef typename Adaptor::index_type						index_type;
	typedef typename Adaptor::scalar							scalar_type;
	typedef typename Adaptor::poly_type							poly_type;
	typedef points_adaptor<poly_type>							PointsAdaptor;

	typedef MeshConnectivity<index_type>						mesh_conn;
	typedef typename mesh_conn::index_set						conn_set;

	typedef std::vector<index_type>								index_vector;
	typedef std::pair<index_type,index_type>					edge_type;
	typedef std::vector<edge_type>								edge_vector;
	typedef boost::unordered_map<index_type,edge_vector>		poly_newedges_map;

	BOOST_MPL_ASSERT((boost::is_same<edge_type,
		typename std::iterator_traits<EdgeConstIterator>::value_type>));

	Adaptor a(mesh);
	unsigned int npoints = a.numPoints();
	unsigned int npolys = a.numPolys();

	// prepare output
	result.clear();
	result.m_polys.clear();
	result.m_polys.reserve(npolys);
	if(poly_remapping)
		poly_remapping->clear();

	// copy points directly, no remapping occurs
	result.setPoints(mesh_points<Mesh>(mesh));

	// poly -> new-edges
	poly_newedges_map target_polys;

	if(edgesBegin != edgesEnd)
	{
		// calc point-face connectivity
		mesh_conn conn(mesh, true);

		// associate new edges with target polys
		for(EdgeConstIterator it=edgesBegin; it!=edgesEnd; ++it)
		{
			if((it->first >= npoints) || (it->second >= npoints))
				continue;

			const conn_set& faces1 = conn.m_pt_face[it->first];
			const conn_set& faces2 = conn.m_pt_face[it->second];

			index_vector faces;
			std::set_intersection(faces1.begin(), faces1.end(),
				faces2.begin(), faces2.end(), std::back_inserter(faces));

			if(faces.size() > 1)
			{
				// need to determine which poly this edge bisects
				// todo
				std::cerr << "not implemented - this is why you didn't get the edge!" << std::endl;
			}

			if(faces.size() == 1)
				target_polys[faces[0]].push_back(*it);
		}

		// split target polys
		for(typename poly_newedges_map::const_iterator it=target_polys.begin();
			it!=target_polys.end(); ++it)
		{
			std::vector<index_vector> new_polys;
			addPolyEdges(a.getPoly(it->first), it->second, new_polys);
			int polyID = -1-static_cast<int>(it->first);

			for(unsigned int i=0; i<new_polys.size(); ++i)
			{
				result.appendPoly(new_polys[i]);
				if(poly_remapping)
					poly_remapping->push_back(polyID);
			}
		}
	}

	// copy those polys not split
	for(unsigned int i=0; i<npolys; ++i)
	{
		if(target_polys.find(i) == target_polys.end())
		{
			result.appendPoly(a.getPoly(i));
			if(poly_remapping)
				poly_remapping->push_back(i+1);
		}
	}

	return 0;
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
