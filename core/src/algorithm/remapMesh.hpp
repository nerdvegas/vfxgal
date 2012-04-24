#ifndef _DGAL_ALGO_REMAP_MESH__H_
#define _DGAL_ALGO_REMAP_MESH__H_

#include <set>
#include <map>
#include <vector>
#include <boost/unordered_map.hpp>
#include <boost/type_traits/is_signed.hpp>
#include <boost/type_traits/make_unsigned.hpp>
#include "../adaptors/mesh.hpp"
#include "../poly3D.hpp"
#include "../simple_mesh.hpp"


namespace vfxgal {


	/*
	 * @brief MeshRemapSettings
	 * Structure describing mesh remapping settings.
	 * @var m_genPointRemapping If true, point remapping will be generated.
	 * @var m_genPolyRemapping If true, poly remapping will be generated.
	 * @var m_genVertexRemapping If true, vertex remapping will be generated.
	 * @var firstPoint Remapping data written to remapMesh.result will start destination
	 * points at this index.
	 * @var firstPoly Remapping data written to remapMesh.result will start destination
	 * polys at this index.
	 * @var m_pointMapping If supplied, tells the algorithm which points are known to map
	 * directly to existing source points (>=0 entries), and which need to be calculated
	 * as interpolated points, or are new points on new faces (-1 entries).
	 * @var m_polyMapping If supplied, tells the algorithm which polys are known to map
	 * directly to existing source polys (>0); which polys are a fragment of an original
	 * (<0); and which are new faces that do not correspond to a source poly (0). The
	 * original poly id is abs(id)-1.
	 * @var identity_point_mapping If true, this tells remapMesh that all points in the
	 * target mesh map directly to original points. It is important to set this value
	 * correctly, otherwise you may incur the overhead of many barycentric calculations when
	 * generating vertex remapping. This takes precedence over a non-null m_pointMapping.
	 * @note m_pointMapping, m_polyMapping and identity_point_mapping serve as hints for
	 * the algorithm, and this will speed up performance and improve accuracy of the result.
	 */
	template<typename Index>
	struct MeshRemapSettings
	{
		typedef Index										index_type;
		typedef typename boost::make_unsigned<Index>::type 	UIndex;
		BOOST_MPL_ASSERT((boost::is_signed<Index>));

		MeshRemapSettings(
			bool genPointRemapping = true,
			bool genPolyRemapping = true,
			bool genVertexRemapping = true,
			UIndex firstPoint = 0,
			UIndex firstPoly = 0,
			const std::vector<Index>* pointMapping = NULL,
			const std::vector<Index>* polyMapping = NULL,
			bool identity_point_mapping = false)
		:	m_genPointRemapping(genPointRemapping),
			m_genPolyRemapping(genPolyRemapping),
			m_genVertexRemapping(genVertexRemapping),
			m_firstPoint(firstPoint),
			m_firstPoly(firstPoly),
			m_pointMapping(pointMapping),
			m_polyMapping(polyMapping),
			m_identity_point_mapping(identity_point_mapping)
		{}

		bool m_genPointRemapping;
		bool m_genPolyRemapping;
		bool m_genVertexRemapping;
		UIndex m_firstPoint;
		UIndex m_firstPoly;
		const std::vector<Index>* m_pointMapping;
		const std::vector<Index>* m_polyMapping;
		bool m_identity_point_mapping;
		// todo add m_identity_poly_mapping. Usage eg: point reorder with no change to polys
	};


	/*
	 * @brief MeshRemapResult
	 * Structure describing the remapping data from a call to remapMesh.
	 * @var m_pointMapping A direct (destPt,srcPt) mapping for points.
	 * @var m_polyMapping A direct (destPoly,srcPoly) mapping for polygons.
	 * @var m_vertexMapping A direct (destPoly,srcPoly) mapping for polygons that are
	 * unclipped. This will be a subset of m_polyMapping.
	 * @var m_pointIMapping A mapping of (destPt, [(srcPt,frac)*N]). This is provided for
	 * points that are the interpolation of several source points.
	 * @var m_vertexIMapping A mapping of (destPoly, <srcPoly, [[(srcVert,frac)*N]*M]>).
	 * This is provided for polys which are a fragment of a source poly - the value in this
	 * map is the interpolated vertex contribution for each vertex in the destination poly
	 * (which contains M vertices). 'srcVert' indexes into srcPoly's local vertices.
	 */
	template<typename Index, typename Scalar>
	struct MeshRemapResult
	{
		typedef Index											index_type;
		typedef Scalar											scalar;

		typedef boost::unordered_map<Index, Index>				remap_map;

		typedef std::pair<Index, Scalar>						contrib_type;
		typedef std::vector<contrib_type>						contribs_type;
		typedef boost::unordered_map<Index, contribs_type>		interpolated_point_remap;

		typedef std::vector<contribs_type>						contribs_vector;
		typedef std::pair<unsigned int, contribs_vector>		vertex_contribs;
		typedef boost::unordered_map<Index, vertex_contribs>	interpolated_vertex_remap;

		remap_map 					m_pointMapping;
		remap_map 					m_polyMapping;
		remap_map 					m_vertexMapping;
		interpolated_point_remap	m_pointIMapping;
		interpolated_vertex_remap	m_vertexIMapping;
	};


	/*
	 * @brief remapMesh
	 * Given a source mesh and a destination mesh, calculate remapping data for mapping
	 * attributes from the source mesh onto the destination mesh. The destination mesh is
	 * expected to be piece(s) of the original mesh - for example, it may be fragments
	 * resulting from a clipping operation. This algorithm does _NOT_ perform distance-
	 * based remapping between unrelated topologies.
	 * @param meshSrc The source mesh.
	 * @param meshDest The destination mesh.
	 * @param settings Mesh remapping settings, see MeshRemapSettings for more info.
	 * @param[out] result The resultant remapping data. Note that any existing remappings
	 * in the result will be added to.
	 */
	template<typename MeshSrc, typename MeshDest, typename Index>
	void remapMesh(const MeshSrc& meshSrc, const MeshDest& meshDest,
		const MeshRemapSettings<Index>& settings,
		MeshRemapResult<typename boost::make_unsigned<Index>::type,
			typename mesh_adaptor<MeshSrc>::scalar>& result);


	/*
	 * @brief combinePolyRemapping
	 * Combine two poly remappings to produce a third.
	 * @note In-place combine is supported, ie 'result' can be the same vector as
	 * mapping1 or mapping2, or a separate vector.
	 */
	template<typename Index>
	void combinePolyRemapping(
		const std::vector<Index>& mapping1,
		const std::vector<Index>& mapping2,
		std::vector<Index>& result);


///////////////////////// impl

template<typename MeshSrc, typename MeshDest, typename Index>
void remapMesh(const MeshSrc& meshSrc, const MeshDest& meshDest,
	const MeshRemapSettings<Index>& s,
	MeshRemapResult<typename boost::make_unsigned<Index>::type,
		typename mesh_adaptor<MeshSrc>::scalar>& result)
{
	typedef mesh_adaptor<MeshSrc> 								SrcAdaptor;
	typedef mesh_adaptor<MeshDest> 								DestAdaptor;
	typedef typename SrcAdaptor::scalar							T;
	typedef typename SrcAdaptor::const_poly_ref					const_src_poly_ref;
	typedef typename SrcAdaptor::poly_type						src_poly_type;
	typedef typename DestAdaptor::poly_type						dest_poly_type;
	typedef points_adaptor<src_poly_type>						SrcPointsAdaptor;
	typedef points_adaptor<dest_poly_type>						DestPointsAdaptor;
	typedef typename boost::make_unsigned<Index>::type			UIndex;
	typedef MeshRemapResult<UIndex,T>							result_type;
	typedef typename result_type::contrib_type					result_contrib_type;
	typedef typename result_type::contribs_type					result_contribs_type;
	typedef typename result_type::vertex_contribs				result_vertex_contribs;
	typedef typename result_type::remap_map						result_remap_map;
	typedef typename result_type::contribs_vector				result_contribs_vector;
	typedef boost::unordered_map<unsigned int, unsigned int>	u_u_map;

	BOOST_MPL_ASSERT((boost::is_same<T,typename DestAdaptor::scalar>));

	if(!s.m_genPointRemapping && !s.m_genPolyRemapping && !s.m_genVertexRemapping)
		return;

	SrcAdaptor srcMesh(meshSrc);
	DestAdaptor destMesh(meshDest);

	// create identity point remapping if all points map directly
	std::vector<Index> point_mapping_;
	const std::vector<Index>* point_mapping = s.m_pointMapping;
	if(s.m_identity_point_mapping)
	{
		unsigned int npoints = destMesh.numPoints();
		point_mapping_.resize(npoints);
		for(unsigned int i=0; i<npoints; ++i)
			point_mapping_[i] = i;
		point_mapping = &point_mapping_;
	}

	if(s.m_genPointRemapping && point_mapping)
	{
		// direct point remapping (straightforward, just copy >=0 entries into the map)
		unsigned int npoints = point_mapping->size();

		for(unsigned int i=0; i<npoints; ++i)
		{
			Index ptnum = (*point_mapping)[i];
			if(ptnum >= 0)
			{
				typename result_remap_map::value_type v(
					s.m_firstPoint+i, static_cast<UIndex>(ptnum));
				result.m_pointMapping.insert(v);
			}
		}
	}

	if(s.m_genPolyRemapping || s.m_genVertexRemapping)
	{
		// we don't support missing poly remapping yet!
		if(!s.m_polyMapping || s.m_polyMapping->size() != destMesh.numPolys())
		{
			throw std::runtime_error("Missing/partial poly mapping not yet supported");
		}

		const std::vector<Index>& polyremap = *(s.m_polyMapping);
		unsigned int npolys = destMesh.numPolys();
		std::vector<T> coeffs;
		u_u_map pt_vert_lookup;

		// remap polys
		for(unsigned int i=0; i<npolys; ++i)
		{
			Index polynum = polyremap[i];
			if(polynum > 0)
			{
				typename result_remap_map::value_type v(
					s.m_firstPoly+i, static_cast<UIndex>(polynum-1));

				if(s.m_genPolyRemapping)
					result.m_polyMapping.insert(v);

				if(s.m_genVertexRemapping)
					result.m_vertexMapping.insert(v);
			}
			else if(polynum < 0)
			{
				UIndex upolynum = static_cast<UIndex>(-1-polynum);

				if(s.m_genPolyRemapping)
				{
					typename result_remap_map::value_type v(s.m_firstPoly+i, upolynum);
					result.m_polyMapping.insert(v);
				}

				// this is a poly fragment
				if(s.m_genVertexRemapping || s.m_genPointRemapping)
				{
					const_src_poly_ref srcPoly = srcMesh.getPoly(upolynum);
					DestPointsAdaptor destPoly(destMesh.getPoly(i));
					unsigned int ndestverts = destPoly.size();

					result_vertex_contribs* vcontribs = NULL;
					if(s.m_genVertexRemapping)
					{
						vcontribs = &(result.m_vertexIMapping[s.m_firstPoly+i]);
						vcontribs->first = upolynum;
						vcontribs->second.resize(ndestverts);

						// build (src pt -> destpoly vert index) lookup
						pt_vert_lookup.clear();
						SrcPointsAdaptor a_srcPoly(srcPoly);
						unsigned int nsrcverts = a_srcPoly.size();

						for(unsigned int j=0; j<nsrcverts; ++j)
							pt_vert_lookup.insert(u_u_map::value_type(a_srcPoly.index(j), j));
					}

					// iterate over dest poly verts
					for(unsigned int j=0; j<ndestverts; ++j)
					{
						coeffs.clear();

						// does the vert map to a src point?
						unsigned int dest_ptnum = destPoly.index(j);
						int src_ptnum = (point_mapping && (dest_ptnum < point_mapping->size()))?
							(*point_mapping)[dest_ptnum] : -1;

						if(s.m_genVertexRemapping)
						{
							result_contribs_type& contribs = vcontribs->second[j];

							if(src_ptnum >= 0)
							{
								u_u_map::const_iterator it = pt_vert_lookup.find(src_ptnum);
								if(it != pt_vert_lookup.end())
								{
									// mapped dest vert to src, via common point
									contribs.resize(1);
									contribs[0] = result_contrib_type(it->second, 1);
								}
							}

							if(contribs.empty())
							{
								// no common point found, do barycentric interpolation
								getGeneralBarycentricCoord(srcPoly, destPoly[j], coeffs);
								unsigned int ncoeffs = coeffs.size();
								contribs.resize(ncoeffs);

								for(unsigned int k=0; k<ncoeffs; ++k)
									contribs[k] = result_contrib_type(k, coeffs[k]);
							}
						}

						dest_ptnum += s.m_firstPoint;

						if(s.m_genPointRemapping && (src_ptnum < 0) &&
							(result.m_pointIMapping.find(dest_ptnum) == result.m_pointIMapping.end()))
						{
							// we can reuse the vertex barycentric coord if that happened
							if(coeffs.empty())
								getGeneralBarycentricCoord(srcPoly, destPoly[j], coeffs);

							unsigned int ncoeffs = coeffs.size();
							result_contribs_type& contribs = result.m_pointIMapping[dest_ptnum];
							contribs.resize(ncoeffs);

							SrcPointsAdaptor a_srcPoly(srcPoly);
							for(unsigned int k=0; k<ncoeffs; ++k)
								contribs[k] = result_contrib_type(a_srcPoly.index(k), coeffs[k]);
						}
					}
				}
			}
		}
	}
}


template<typename Index>
void combinePolyRemapping(
	const std::vector<Index>& mapping1,
	const std::vector<Index>& mapping2,
	std::vector<Index>& result)
{
	typedef typename boost::make_unsigned<Index>::type UIndex;
	BOOST_MPL_ASSERT((boost::is_signed<Index>));

	bool inplace = (&result == &mapping2) || (&result == &mapping1);
	std::vector<Index> result_;
	std::vector<Index>* presult = (inplace)? &result_ : &result;

	unsigned int nelems1 = mapping1.size();
	unsigned int nelems2 = mapping2.size();
	presult->resize(nelems2);

	for(unsigned int i=0; i<nelems2; ++i)
	{
		Index ind = mapping2[i];
		if(ind == 0)
		{
			(*presult)[i] = 0;
		}
		else if(ind > 0)
		{
			ind-=1;
			(*presult)[i] = (ind < nelems1)? mapping1[ind] : 0;
		}
		else
		{
			ind=-1-ind;
			(*presult)[i] = (ind < nelems1)? -std::abs(mapping1[ind]) : 0;
		}
	}

	if(inplace)
		result = result_;
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
