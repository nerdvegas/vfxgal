#ifndef _DGAL_POINTS__H_
#define _DGAL_POINTS__H_

#include "adaptors/points.hpp"
#include <OpenEXR/ImathBox.h>


namespace vfxgal {

	/*
	 * @brief get_bounds
	 * Get the bounding box of a set of points.
	 * @param points Point to calc bounds of.
	 * @param[out] b Bounding box.
	 * @param extend If true, the box is extended, if false, it is set.
	 */
	template<typename Points>
	void get_bounds(const Points& points,
		Imath::Box<typename points_adaptor<Points>::elem_type>& b, bool extend = false);


///////////////////////// impl

template<typename Points>
void get_bounds(const Points& points,
	Imath::Box<typename points_adaptor<Points>::elem_type>& b, bool extend)
{
	typedef points_adaptor<Points>			Adaptor;
	typedef typename Adaptor::elem_type		point_type;
	typedef Imath::Box<point_type>			box_type;
	Adaptor a(points);

	if(!extend)
		b.makeEmpty();

	unsigned int npoints = a.size();
	for(unsigned int i=0; i<npoints; ++i)
		b.extendBy(a[i]);
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
