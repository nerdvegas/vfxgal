#ifndef _DGAL_MESHCONNECTIVITY__H_
#define _DGAL_MESHCONNECTIVITY__H_

#include "adaptors/mesh.hpp"
#include "edge.hpp"
#include <boost/unordered_map.hpp>
#include <set>
#include <algorithm>
#include <stdexcept>


namespace vfxgal {

	/*
	 * @class MeshConnectivity
	 * @brief A class that gathers connectivity data for the given mesh.
	 * @var m_pt_face (point -> incident faces)
	 * @var m_face_face (face -> adjacent faces)
	 * @var m_edge_face (edge -> adjacent faces)
	 * @var m_pt_vert (point -> incident vertices)
	 */
	template<typename Index>
	struct MeshConnectivity
	{
		typedef Index										index_type;
		typedef bidirectional_edge<Index>					bi_edge;
		typedef std::set<Index>								index_set;
		typedef std::pair<Index, Index>						vertex_type; // face:vertex
		typedef std::set<vertex_type>						vertex_set;

		typedef boost::unordered_map<Index, index_set>		pt_face_map;
		typedef boost::unordered_map<Index, index_set>		face_face_map;
		typedef boost::unordered_map<bi_edge, index_set>	edge_face_map;
		typedef boost::unordered_map<Index, vertex_set>		pt_vert_map;

		MeshConnectivity() { clear(); }

		template<typename Mesh>
		MeshConnectivity(const Mesh& mesh, bool pt_face=false, bool face_face=false,
			bool edge_face=false, bool pt_vert=false)
		{
			set<Mesh>(mesh, pt_face, face_face, edge_face, pt_vert);
		}

		template<typename Mesh>
		void set(const Mesh& mesh, bool pt_face=false, bool face_face=false,
			bool edge_face=false, bool pt_vert=false);

		void clear();

		bool m_pt_face_valid;
		bool m_face_face_valid;
		bool m_edge_face_valid;
		bool m_pt_vert_valid;

		pt_face_map 	m_pt_face;
		face_face_map 	m_face_face;
		edge_face_map 	m_edge_face;
		pt_vert_map 	m_pt_vert;
	};


///////////////////////// impl

template<typename Index>
void MeshConnectivity<Index>::clear()
{
	m_pt_face.clear();
	m_face_face.clear();
	m_edge_face.clear();
	m_pt_vert.clear();

	m_pt_face_valid = false;
	m_face_face_valid = false;
	m_edge_face_valid = false;
	m_pt_vert_valid = false;
}


template<typename Index>
template<typename Mesh>
void MeshConnectivity<Index>::set(const Mesh& mesh, bool pt_face, bool face_face,
	bool edge_face, bool pt_vert)
{
	typedef mesh_adaptor<Mesh>						Adaptor;
	typedef typename Adaptor::index_type			mesh_index_type;
	typedef typename Adaptor::poly_type				poly_type;
	typedef points_adaptor<poly_type>				PointsAdaptor;
	BOOST_MPL_ASSERT((boost::is_same<Index, mesh_index_type>));

	clear();

	Adaptor a(mesh);
	unsigned int npoints = a.numPoints();
	unsigned int npolys = a.numPolys();

	// pt->face, pt->vert
	if(pt_face || pt_vert)
	{
		for(unsigned int i=0; i<npolys; ++i)
		{
			PointsAdaptor poly(a.getPoly(i));
			unsigned int nverts = poly.size();

			for(unsigned int j=0; j<nverts; ++j)
			{
				unsigned int ptnum = poly.index(j);
				if(ptnum < npoints)
				{
					if(pt_face)
						m_pt_face[ptnum].insert(i);
					if(pt_vert)
						m_pt_vert[ptnum].insert(vertex_type(i,j));
				}
			}
		}

		if(pt_face)	m_pt_face_valid = true;
		if(pt_vert)	m_pt_vert_valid = true;
	}

	// edge->face
	if(edge_face || face_face)
	{
		for(unsigned int i=0; i<npolys; ++i)
		{
			PointsAdaptor poly(a.getPoly(i));
			unsigned int nverts = poly.size();

			for(unsigned int j=0; j<nverts; ++j)
			{
				unsigned int ptnum = poly.index(j);
				if(ptnum < npoints)
				{
					unsigned int ptnum2 = poly.index((j+1)%nverts);
					if(ptnum2 < npoints)
						m_edge_face[bi_edge(ptnum, ptnum2)].insert(i);
				}
			}
		}

		m_edge_face_valid = true;
	}

	// face->face
	if(face_face)
	{
		for(unsigned int i=0; i<npolys; ++i)
		{
			index_set faces[2];
			unsigned int k = 0;
			PointsAdaptor poly(a.getPoly(i));
			unsigned int nverts = poly.size();

			for(unsigned int j=0; j<nverts; ++j)
			{
				unsigned int ptnum = poly.index(j);
				unsigned int ptnum2 = poly.index((j+1)%nverts);
				typename edge_face_map::const_iterator it =
					m_edge_face.find(bi_edge(ptnum, ptnum2));

				if(it != m_edge_face.end())
				{
					std::set_union(it->second.begin(), it->second.end(), faces[k].begin(),
						faces[k].end(), std::inserter(faces[1-k], faces[1-k].begin()));

					k=1-k;
				}
			}

			faces[k].erase(i);
			m_face_face.insert(typename face_face_map::value_type(i, faces[k]));
		}

		m_face_face_valid = true;
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
