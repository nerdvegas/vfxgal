#ifndef _DGAL_ALGO_CLIPPOLY2D__H_
#define _DGAL_ALGO_CLIPPOLY2D__H_

#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include "../poly2D.hpp"
#include "../ClippingContext.hpp"
#include "../ClippedPoly.hpp"


namespace dgal {

	/*
	 * @brief clipPoly2D
	 * Clip a poly against a cutting plane.
	 * @param points The polygon's points, in clockwise order.
	 * @param plane The cutting plane. Those vertices in the positive halfspace will be kept.
	 * @param result[out] The clip result. Polygon(s) generated from this clip (if any)
	 * will be appended.
	 * @note Keyholes are supported, as is some degree of complexity (ie we can deal with
	 * a little self-intersection).
	 */
	template<typename Points2D>
	IntersectType clipPoly2D(const Points2D& points,
		const Line2<typename points_adaptor<Points2D>::scalar>& plane,
		std::list<ClippedPoly<typename points_adaptor<Points2D>::scalar> >& result,
		detail::ClippingContext<typename points_adaptor<Points2D>::scalar>* ctxt = NULL);


///////////////////////// impl

template<typename Points2D>
IntersectType clipPoly2D(const Points2D& points,
	const Line2<typename points_adaptor<Points2D>::scalar>& plane,
	std::list<ClippedPoly<typename points_adaptor<Points2D>::scalar> >& result,
	detail::ClippingContext<typename points_adaptor<Points2D>::scalar>* ctxt)
{
	typedef points_adaptor<Points2D> 					Adaptor;
	typedef typename Adaptor::scalar					T;
	typedef typename Adaptor::elem_type					point_type;
	typedef typename Adaptor::const_elem_ref			const_point_ref;
	typedef ClippedPoly<T> 								polyclip_type;
	typedef typename polyclip_type::intersection		polyclip_intersection;

	typedef std::multimap<T, unsigned int>							onplane_lookup;
	typedef boost::unordered_multimap<unsigned int, unsigned int>	edge_lookup;
	typedef boost::unordered_set<unsigned int>						visited_verts_set;
	typedef boost::unordered_map<unsigned int, unsigned int>		shared_verts_map;

	Adaptor a(points);

	unsigned int npoints = a.size();
	assert(a.size() > 2);

	static const unsigned int shared_offset = std::numeric_limits<unsigned int>::max()/2;
	bool shared_vert_check = (npoints > 4);
	visited_verts_set visited_verts;
	shared_verts_map shared_vert_remap;

	unsigned int index1, index2;
	index1 = a.index(0);
	if(shared_vert_check)
		visited_verts.insert(index1);

	std::vector<polyclip_intersection> onplanePoints;
	std::vector<unsigned int> insidePoints;

	polyclip_intersection edgeInt;
	point_type line_dir = plane.dir();
	bool anyPtOutside = false;
	bool ptInsideNext;
	bool ptInside = (ctxt)? ctxt->isPointInside(index1) : plane.isRight(a[0]);

	onplane_lookup intersectionsOut, intersectionsIn;
	edge_lookup edges(npoints);

	// do the clip. Note that onplane points are indexed as i+npoints and are adjusted after
	for(unsigned int i=0; i<npoints; ++i, ptInside=ptInsideNext, index1=index2)
	{
		unsigned int j = (i+1) % npoints;
		const_point_ref pt_i = a[i];
		const_point_ref pt_j = a[j];

		index2 = a.index(j);
		anyPtOutside |= !ptInside;
		ptInsideNext = (ctxt)? ctxt->isPointInside(index2) : plane.isRight(pt_j);

		if((j>0) && shared_vert_check && !visited_verts.insert(index2).second)
		{
			unsigned int alias_index = shared_offset + shared_vert_remap.size();
			shared_vert_remap.insert(shared_verts_map::value_type(alias_index, index2));
			index2 = alias_index;
		}

		if(ptInsideNext != ptInside)
		{
			// calc edge intersection with line
			edgeInt.m_point1 = index1;
			edgeInt.m_point2 = index2;
			plane.intersect(pt_i, pt_j, edgeInt.m_u);

			// if the edge is almost exactly parallel to the plane it is possible to
			// get a spurious value due to float-pt inaccuracy - solve the prob in double
			if((edgeInt.m_u <= 0) || (edgeInt.m_u >= 1))
			{
				if(boost::is_same<T,double>::value)
					edgeInt.m_u = std::min(std::max(edgeInt.m_u, T(0.0)), T(1.0));
				else
				{
					Line2<double> plane_d(plane);
					Imath::V2d ptd_i(pt_i);
					Imath::V2d ptd_j(pt_j);
					double u;
					plane_d.intersect(ptd_i, ptd_j, u);
					u = std::min(std::max(u, 0.0), 1.0);
					edgeInt.m_u = static_cast<T>(u);
				}
			}

			// sort intersection wrt line dir
			point_type pt_int = pt_i + (pt_j-pt_i)*edgeInt.m_u;
			T dist = pt_int.dot(line_dir);

			unsigned int vert_int = onplanePoints.size() + npoints;
			onplanePoints.push_back(edgeInt);

			if(ptInside)
				intersectionsOut.insert(typename onplane_lookup::value_type(dist, vert_int));
			else
				intersectionsIn.insert(typename onplane_lookup::value_type(dist, vert_int));

			if(ptInside)
			{
				unsigned int vert1 = insidePoints.size();
				unsigned int vert2 = vert_int;
				insidePoints.push_back(index1);
				edges.insert(edge_lookup::value_type(vert1, vert2));
			}
			else
			{
				unsigned int vert1 = vert_int;
				unsigned int vert2 = (j==0)? 0 : insidePoints.size();
				edges.insert(edge_lookup::value_type(vert1, vert2));
			}
		}
		else if(ptInside)
		{
			unsigned int vert1 = insidePoints.size();
			unsigned int vert2 = (j==0)? 0 : vert1+1;
			insidePoints.push_back(index1);
			edges.insert(edge_lookup::value_type(vert1, vert2));
		}
	}

	if(insidePoints.empty())
		return INTERSECT_OUTSIDE;

	if(!anyPtOutside)
	{
		// poly is completely inside
		result.push_back(polyclip_type());
		polyclip_type& pclip = result.back();
		pclip.makeInside(points);
		if(ctxt)
			pclip.m_polyId = ctxt->m_currentPolyId;

		return INTERSECT_INSIDE;
	}

	assert(!intersectionsOut.empty());
	assert(intersectionsOut.size() == intersectionsIn.size());

	// turn each pair of intersections into an edge
	while(!intersectionsOut.empty())
	{
		typename onplane_lookup::iterator itOut = intersectionsOut.begin();
		typename onplane_lookup::iterator itIn = intersectionsIn.begin();
		edges.insert(edge_lookup::value_type(itOut->second, itIn->second));
		intersectionsOut.erase(itOut);
		intersectionsIn.erase(itIn);
	}

	// find each closed loop within edges and return as poly
	while(!edges.empty())
	{
		result.push_back(polyclip_type());
		polyclip_type& pclip = result.back();
		pclip.m_vertices.reserve(edges.size());
		if(ctxt)
			pclip.m_polyId = -std::abs(ctxt->m_currentPolyId);

		edge_lookup::iterator it = edges.begin();
		unsigned int first_vert = it->first;

		while(it != edges.end())
		{
			unsigned int vert = it->first;
			if(vert >= npoints)
			{
				pclip.m_vertices.push_back(pclip.m_onplanePoints.size() + npoints);
				pclip.m_onplanePoints.push_back(onplanePoints[vert - npoints]);
			}
			else
			{
				pclip.m_vertices.push_back(pclip.m_insidePoints.size());
				pclip.m_insidePoints.push_back(insidePoints[vert]);
			}

			unsigned int next_vert = it->second;
			edges.erase(it);
			it = edges.find(next_vert);
		}

		// remap shared verts, if any
		if(shared_vert_check && !shared_vert_remap.empty())
		{
			for(unsigned int i=0; i<pclip.m_insidePoints.size(); ++i)
			{
				unsigned int vert = pclip.m_insidePoints[i];
				if(vert >= shared_offset)
				{
					assert(shared_vert_remap.find(vert) != shared_vert_remap.end());
					pclip.m_insidePoints[i] = shared_vert_remap.find(vert)->second;
				}
			}

			for(unsigned int i=0; i<pclip.m_onplanePoints.size(); ++i)
			{
				polyclip_intersection& is = pclip.m_onplanePoints[i];
				if(is.m_point1 >= shared_offset)
				{
					assert(shared_vert_remap.find(is.m_point1) != shared_vert_remap.end());
					is.m_point1 = shared_vert_remap.find(is.m_point1)->second;
				}
				if(is.m_point2 >= shared_offset)
				{
					assert(shared_vert_remap.find(is.m_point2) != shared_vert_remap.end());
					is.m_point2 = shared_vert_remap.find(is.m_point2)->second;
				}
			}
		}

		// remap verts so that onplane verts are correctly offset by inside pt count
		assert(pclip.m_vertices.size() > 2);
		unsigned int adjust = npoints - pclip.m_insidePoints.size();
		for(unsigned int i=0; i<pclip.m_vertices.size(); ++i)
		{
			if(pclip.m_vertices[i] >= npoints)
				pclip.m_vertices[i] -= adjust;
		}
	}

	return INTERSECT_INTERSECTS;
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
