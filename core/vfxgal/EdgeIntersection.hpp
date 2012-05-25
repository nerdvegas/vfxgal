#ifndef _DGAL_EDGE_INTERSECTION__H_
#define _DGAL_EDGE_INTERSECTION__H_

#include "adaptors/points_indexer.hpp"


namespace vfxgal {

	/*
	 * @class EdgeIntersection
	 * @brief A fractional position between two points.
	 */
	template<typename Index, typename Scalar>
	struct EdgeIntersection
	{
		typedef Index		index_type;
		typedef Scalar		scalar_type;

		EdgeIntersection(){}

		EdgeIntersection(Index p1, Index p2, Scalar u)
		: m_point1(p1), m_point2(p2), m_u(u){}

		template<typename Points>
		inline void getPos(const Points& points,
			typename points_adaptor<Points>::elem_type& p) const;

		Index m_point1;
		Index m_point2;
		Scalar m_u;
	};


///////////////////////// impl

template<typename Index, typename Scalar>
template<typename Points>
void EdgeIntersection<Index,Scalar>::getPos(const Points& points,
	typename points_adaptor<Points>::elem_type& p) const
{
	typedef points_adaptor<Points> 				Adaptor;
	typedef typename Adaptor::const_elem_ref 	const_point_ref;

	Adaptor a(points);
	const_point_ref p1 = a[m_point1];
	p = p1 + ((a[m_point2] - p1) * m_u);
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
