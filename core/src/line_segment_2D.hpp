#ifndef _DGAL_LINE_SEGMENT_2D__H_
#define _DGAL_LINE_SEGMENT_2D__H_

#include "LineSegment.hpp"
#include "Line2.hpp"


namespace vfxgal {

	/*
	 * @brief intersects
	 * @returns True if the line segments intersect, false otherwise.
	 */
	template<typename T>
	bool intersects(const LineSegment<Imath::Vec2<T> >& l1,
		const LineSegment<Imath::Vec2<T> >& l2);


///////////////////////// impl

template<typename T>
bool intersects(const LineSegment<Imath::Vec2<T> >& l1,
	const LineSegment<Imath::Vec2<T> >& l2)
{
	typedef Imath::Vec2<T> vec_type;

	vec_type r = l1.m_end - l1.m_start;
	vec_type s = l2.m_end - l2.m_start;

	T rxs = r.cross(s);
	if(std::abs(rxs) < std::numeric_limits<T>::epsilon())
		return false;

	vec_type qp = l2.m_start - l1.m_start;

	T t = qp.cross(s) / rxs;
	if((t<0) || (t>1))
		return false;

	T u = qp.cross(r) / rxs;
	return ((u>=0) && (u<=1));
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
