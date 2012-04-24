#ifndef _DGAL_PLANE__H_
#define _DGAL_PLANE__H_

#include "enums.hpp"
#include "util.hpp"
#include "Projection_2D.hpp"
#include "ClippingContext.hpp"
#include "adaptors/points.hpp"
#include <limits>


namespace dgal {

	/*
	 * @brief inside
	 * @returns True if p is on the plane, or in the positive halfspace of the plane,
	 * and false otherwise.
	 */
	template<typename T>
	inline bool isInside(const Imath::Plane3<T>& plane, const Imath::Vec3<T>& p) {
		return (plane.distanceTo(p) >= 0);
	}


	/*
	 * @brief locatePoint
	 * @returns 0 if p is on the plane (within given epsilon distance)
	 * 1  if p is in the positive halfspace of the plane,
	 * -1  if p is in the negative halfspace of the plane.
	 */
	template<typename T>
	inline int locatePoint(const Imath::Plane3<T>& plane, const Imath::Vec3<T>& p, T epsilon)
	{
		T dist = plane.distanceTo(p);
		return (dist<-epsilon ?-1 : (dist> epsilon ? 1 : 0));
	}

	/*
	 * @brief project_point
	 * Project a 3D point onto a plane.
	 */
	template<typename T>
	inline void project_point(const Imath::Plane3<T>& plane, const Imath::Vec3<T>& p,
		Imath::Vec3<T>& pProj);

	/*
	 * @brief project_vector
	 * Project a 3D vector onto a plane.
	 */
	template<typename T>
	inline void project_vector(const Imath::Plane3<T>& plane, const Imath::Vec3<T>& v,
		Imath::Vec3<T>& vProj);

	/*
	 * @brief
	 * @returns The reversal of the given plane.
	 */
	template<typename T>
	inline Imath::Plane3<T> reverse(const Imath::Plane3<T>& plane) {
		return Imath::Plane3<T>(-plane.normal, -plane.distance);
	}

	/*
	 * @brief intersect
	 * Return the line resulting from the intersection of two planes.
	 * @param pl1 Plane 1.
	 * @param pl2 Plane 2.
	 * @param[out] line Intersection line.
	 * @returns False if there is no intersection, true otherwise.
	 */
	template<typename T>
	bool intersect(const Imath::Plane3<T>& pl1, const Imath::Plane3<T>& pl2, Imath::Line3<T>& line);

	/*
	 * @brief intersect
	 * Return the point resulting from the intersection of a plane, and a line travelling
	 * through points p1 and p2.
	 * @param plane Plane to intersect with
	 * @param p1,p2 Points that the line travels through.
	 * @param[out] p The point of intersection.
	 * @returns False if there is no intersection, true otherwise.
	 */
	template<typename T>
	bool intersect(const Imath::Plane3<T>& plane, const Imath::Vec3<T>& p1,
		const Imath::Vec3<T>& p2, Imath::Vec3<T>& p);

	/*
	 * @brief clip_test
	 * Test if a set of points is completely inside a halfspace, completely outside, or
	 * partially inside and outside the plane.
	 * @param points The set of points.
	 * @param plane The cutting plane. Points are considered inside if they are in the
	 * positive halfspace.
	 */
	template<typename Points3D>
	IntersectType clip_test(const Imath::Plane3<typename points_adaptor<Points3D>::scalar>& plane,
		const Points3D& points);

	namespace detail {

		template<typename Points3D>
		IntersectType clip_test(
			const ClippingContext<typename points_adaptor<Points3D>::scalar>& ctxt,
			const Points3D& points);

	}


///////////////////////// impl

template<typename T>
void project_point(const Imath::Plane3<T>& plane, const Imath::Vec3<T>& p,
	Imath::Vec3<T>& pProj)
{
	T d = p.dot(plane.normal);
	pProj = p - (plane.normal * (d - plane.distance));
}


template<typename T>
void project_vector(const Imath::Plane3<T>& plane, const Imath::Vec3<T>& v,
	Imath::Vec3<T>& vProj)
{
	vProj = v - (v.dot(plane.normal) * plane.normal);
}


// http://paulbourke.net/geometry/planeplane
template<typename T>
bool intersect(const Imath::Plane3<T>& pl1, const Imath::Plane3<T>& pl2, Imath::Line3<T>& line)
{
	T d = pl1.normal.dot(pl2.normal);
	T determinant = 1 - d*d;

	if(determinant < std::numeric_limits<T>::epsilon())
		return false;

	T c1 = (pl1.distance - (pl2.distance * d)) / determinant;
	T c2 = (pl2.distance - (pl1.distance * d)) / determinant;
	line.pos = (c1 * pl1.normal) + (c2 * pl2.normal);

	line.dir = pl2.normal.cross(pl1.normal);
	line.dir.normalize();

	return true;
}


template<typename T>
bool intersect(const Imath::Plane3<T>& plane, const Imath::Vec3<T>& p1,
	const Imath::Vec3<T>& p2, Imath::Vec3<T>& p)
{
	Imath::Vec3<T> p1_2 = p2 - p1;
	T d = p1_2.dot(plane.normal);
	if(std::abs(d) < std::numeric_limits<T>::epsilon())
		return false;

	T d1 = p1.dot(plane.normal);
	T frac = (plane.distance - d1) / d;
	p = p1 + (p1_2 * frac);
	return true;
}


template<typename Points3D>
IntersectType clip_test(const Imath::Plane3<typename points_adaptor<Points3D>::scalar>& plane,
	const Points3D& points)
{
	typedef points_adaptor<Points3D> 			Adaptor;
	typedef typename Adaptor::const_elem_ref	vec3_const_ref;
	Adaptor a(points);

	unsigned int npoints = a.size();
	bool all_inside = true;
	bool all_outside = true;

	for(unsigned int i=0; (i<npoints) && (all_inside || all_outside); ++i)
	{
		vec3_const_ref p = a[i];
		bool ptInside = isInside(plane, p);
		all_inside &= ptInside;
		all_outside &= !ptInside;
	}

	return (all_inside)?
		INTERSECT_INSIDE : ((all_outside)? INTERSECT_OUTSIDE : INTERSECT_INTERSECTS);
}


template<typename Points3D>
IntersectType detail::clip_test(
	const detail::ClippingContext<typename points_adaptor<Points3D>::scalar>& ctxt,
	const Points3D& points)
{
	typedef points_adaptor<Points3D> Adaptor;
	Adaptor a(points);

	unsigned int npoints = a.size();
	bool all_inside = true;
	bool all_outside = true;

	for(unsigned int i=0; (i<npoints) && (all_inside || all_outside); ++i)
	{
		assert(a.index(i) < ctxt.m_isPtInside.size());
		bool ptInside = ctxt.m_isPtInside[a.index(i)];
		all_inside &= ptInside;
		all_outside &= !ptInside;
	}

	return (all_inside)?
		INTERSECT_INSIDE : ((all_outside)? INTERSECT_OUTSIDE : INTERSECT_INTERSECTS);
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
