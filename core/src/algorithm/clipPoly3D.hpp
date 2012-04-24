#ifndef _DGAL_ALGO_CLIPPOLY3D__H_
#define _DGAL_ALGO_CLIPPOLY3D__H_

#include "../poly3D.hpp"
#include "clipPoly2D.hpp"

namespace dgal {

	/*
	 * @brief clipPoly3D
	 * Clip a poly against a cutting plane.
	 * @note It is assumed that the given points are coplanar.
	 * @param points The polygon's points.
	 * @param plane The cutting plane. Those vertices in the positive halfspace will be kept.
	 * @param result[out] The clip result. Polygon(s) generated from this clip (if any)
	 * will be appended.
	 */
	template<typename Points3D>
	IntersectType clipPoly3D(const Points3D& points,
		const Imath::Plane3<typename points_adaptor<Points3D>::scalar>& plane,
		std::list<ClippedPoly<typename points_adaptor<Points3D>::scalar> >& result,
		detail::ClippingContext<typename points_adaptor<Points3D>::scalar>* ctxt = NULL);


///////////////////////// impl

template<typename Points3D>
IntersectType clipPoly3D(const Points3D& points,
	const Imath::Plane3<typename points_adaptor<Points3D>::scalar>& plane,
	std::list<ClippedPoly<typename points_adaptor<Points3D>::scalar> >& result,
	detail::ClippingContext<typename points_adaptor<Points3D>::scalar>* ctxt)
{
	typedef points_adaptor<Points3D> 			Adaptor;
	typedef typename Adaptor::scalar			T;
	typedef Imath::Vec2<T> 						vec2_type;
	typedef Imath::Vec3<T> 						vec3_type;
	typedef typename Adaptor::const_elem_ref	vec3_const_ref;
	typedef ClippedPoly<T>						polyclip_type;
	Adaptor a(points);

	vec3_type polyNorm;
	calc_normal(points, polyNorm);
	vec3_const_ref p0 = a[0];

	// project poly and cutting plane into 2D
	std::vector<vec2_type> points2D;
	Line2<T> plane2D;

	{
		Imath::Line3<T> planesInt;
		Imath::Plane3<T> polyPlane(p0, polyNorm);
		if(intersect(plane, polyPlane, planesInt))
		{
			Projection_2D<T> proj(polyNorm, &plane.normal);
			proj.project(planesInt, plane2D);
			proj.project(points, points2D);
		}
		else
		{
			// poly and cutting plane are parallel, so poly is either all out or all in
			if((ctxt)? ctxt->isPointInside(a.index(0)) : isInside(plane, p0))
			{
				result.push_back(polyclip_type());
				result.back().makeInside(points);
				return INTERSECT_INSIDE;
			}
			else
				return INTERSECT_OUTSIDE;
		}
	}

	// do the clip in 2D.
	if(a.is_indexed())
	{
		points_replacer<Points3D, std::vector<vec2_type> > points2D_(points, points2D, false);
		return clipPoly2D(points2D_, plane2D, result, ctxt);
	}
	else
		return clipPoly2D(points2D, plane2D, result, ctxt);
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
