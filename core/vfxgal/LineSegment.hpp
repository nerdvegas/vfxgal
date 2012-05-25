#ifndef _DGAL_LINESEGMENT__H_
#define _DGAL_LINESEGMENT__H_

#include <OpenEXR/ImathVec.h>
#include <limits>


namespace vfxgal {

	/*
	 * @class LineSegment
	 * @brief A line between two points.
	 */
	template<typename T>
	struct LineSegment
	{
		typedef T 						point_type;
		typedef typename T::BaseType	scalar;

		point_type m_start, m_end;

		LineSegment(const point_type& start, const point_type& end)
		: m_start(start),m_end(end){}

		inline scalar length() const { return (m_end-m_start).length(); }

		inline scalar length2() const { return (m_end-m_start).length2(); }

		inline point_type vector() const { return m_end-m_start; }

		/*
		 * @brief getClosestDistance
		 * @returns The closest distance between the line segment and point p.
		 */
		scalar getClosestDistance(const point_type& p) const;

		/*
		 * @brief getClosestPoint
		 * @returns The closest point on the line segment to point p.
		 */
		point_type getClosestPoint(const point_type& p) const;
	};


///////////////////////// impl

template<typename T>
typename LineSegment<T>::scalar
LineSegment<T>::getClosestDistance(const point_type& p) const
{
	point_type seg = m_end - m_start;
	scalar seglen2 = seg.length2();

	if(seglen2 < std::numeric_limits<scalar>::epsilon())
		return (p-m_start).length();

	point_type to_p = p-m_start;
	scalar t = seg.dot(to_p) / seglen2;

	if(t<=0)
		return to_p.length();
	else if(t>=1)
		return (p-m_end).length();

	return (to_p - (seg*t)).length();
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
