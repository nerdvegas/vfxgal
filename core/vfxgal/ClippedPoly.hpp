#ifndef _DGAL_CLIPPED_POLY__H_
#define _DGAL_CLIPPED_POLY__H_

#include "adaptors/points_indexer.hpp"
#include "EdgeIntersection.hpp"
#include <boost/mpl/assert.hpp>
#include <iostream>
#include <cassert>


namespace vfxgal {


	/*
	 * @class ClippedPoly
	 * @brief Structure to hold the result of a call to clip functions such as clipPoly2D,
	 * clipPoly3D. This class does not hold any physical points; instead, it indexes into
	 * the original clipped structure.
	 *
	 * @var m_insidePoints A list of indexes into the original list of points. These points
	 * are inside the clipping plane.
	 * @var m_onplanePoints A list of intersections between original points. These are the
	 * result of poly edges intersecting with the cutting plane.
	 * @var m_vertices The new polygon's vertices. Where the index is < len(m_insidePoints)
	 * it is indexing into m_insidePoints. Where >= len(m_insidePoints), it is indexing
	 * into the m_onplanePoints vector.
	 * @var m_polyId Indicates where this poly originated from wrt source geometry:
	 * id=0: The poly is a new face;
	 * id>0: The poly is unclipped and originates from poly id-1;
	 * id<0: The poly is a clipped portion of the original poly -1-id.
	 */
	template<typename T>
	struct ClippedPoly
	{
		typedef EdgeIntersection<unsigned int, T>	intersection;
		typedef std::vector<unsigned int>			index_vec;
		typedef std::vector<intersection>			intersection_vec;

		ClippedPoly():m_polyId(0){}

		inline unsigned int numInsideVertices() const 		{ return m_insidePoints.size(); }
		inline unsigned int numIntersectionVertices() const { return m_onplanePoints.size(); }
		inline unsigned int numVertices() const 			{ return m_vertices.size(); }

		/*
		 * @brief isIntersectionVertex
		 * @returns True if the given vertex points lies inside the clip, false if it is
		 * a plane-incident intersection point.
		 */
		inline bool isInside(unsigned int vert) const {
			return (m_vertices[vert] < m_insidePoints.size());
		}

		/*
		 * @brief getInsideIndex
		 * @returns The point index for this vertex.
		 * @note Assumes the vertex is inside.
		 */
		inline unsigned int getInsideIndex(unsigned int vert) const {
			assert(isInside(vert));
			return m_insidePoints[m_vertices[vert]];
		}

		/*
		 * @brief getIntersection
		 * @returns The intersection for this vertex.
		 * @note Assumes the vertex is not inside.
		 */
		inline const intersection& getIntersection(unsigned int vert) const {
			assert(!isInside(vert));
			return m_onplanePoints[m_vertices[vert] - m_insidePoints.size()];
		}

		/*
		 * @brief clear
		 * Clear this clipped poly.
		 */
		void clear()
		{
			m_insidePoints.clear();
			m_onplanePoints.clear();
			m_vertices.clear();
		}

		/*
		 * @brief makeInside
		 * Set this clip to represent the unclipped polygon.
		 */
		template<typename Points>
		void makeInside(const Points& points);

		/*
		 * @brief getPos
		 * Return the position of a point in the clipped poly, given the original points.
		 * @param points The point list initially used in the poly clip.
		 * @param vert The clipped poly vertex to get the position of (valid values are
		 * 0 .. len(m_vertices)-1).
		 * @param[out] p The vertex position.
		 */
		template<typename Points>
		void getPos(const Points& points, unsigned int vert,
			typename points_adaptor<Points>::elem_type& p) const;

		index_vec m_insidePoints;
		intersection_vec m_onplanePoints;
		index_vec m_vertices;
		int m_polyId;
	};

	template<typename T>
	std::ostream& operator <<(std::ostream& os, const ClippedPoly<T>& cpoly);


///////////////////////// impl

template<typename T>
template<typename Points>
void ClippedPoly<T>::makeInside(const Points& points)
{
	typedef points_adaptor<Points> Adaptor;
	Adaptor a(points);

	unsigned int npoints = a.size();
	m_onplanePoints.clear();
	m_insidePoints.resize(npoints);
	m_vertices.resize(npoints);
	for(unsigned int i=0; i<npoints; ++i)
	{
		m_insidePoints[i] = a.index(i);
		m_vertices[i] = i;
	}
}


template<typename T>
template<typename Points>
void ClippedPoly<T>::getPos(const Points& points, unsigned int vert,
	typename points_adaptor<Points>::elem_type& p) const
{
	BOOST_MPL_ASSERT_NOT((is_points_indexer<Points>));
	assert(vert < m_vertices.size());

	typedef points_adaptor<Points> Adaptor;
	Adaptor a(points);

	unsigned int i = m_vertices[vert];

	if(i < m_insidePoints.size())
		p = a[m_insidePoints[i]];
	else
	{
		const intersection& is = m_onplanePoints[i - m_insidePoints.size()];
		is.getPos(points, p);
	}
}


template<typename T>
std::ostream& operator <<(std::ostream& os, const typename ClippedPoly<T>::intersection& is)
{
	os << '(' << is.m_point1 << ',' << is.m_point2 << ':' << is.m_u << ')';
	return os;
}


template<typename T>
std::ostream& operator <<(std::ostream& os, const ClippedPoly<T>& cpoly)
{
	for(unsigned int i=0; i<cpoly.numVertices(); ++i)
	{
		if(cpoly.isInside(i))
			os << cpoly.getInsideIndex(i);
		else
		{
			const typename ClippedPoly<T>::intersection& is = cpoly.getIntersection(i);
			os << '(' << is.m_point1 << "->" << is.m_point2 << ':' << is.m_u << ')';
		}

		if(i < cpoly.numVertices()-1)
			os << " -> ";
	}
	return os;
}

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
