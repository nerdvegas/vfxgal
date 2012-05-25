#ifndef _DGAL_POLY3D__H_
#define _DGAL_POLY3D__H_

#include "adaptors/points_replacer.hpp"
#include "plane.hpp"
#include "algorithm/clipPoly2D.hpp"
#include "enums.hpp"
#include "util.hpp"
#include "LineSegment.hpp"
#include "Projection_2D.hpp"


/*
 * @note In all the following functions, a poly's points are assumed to be given in
 * clockwise winding order, when looking down at the poly from its positive normal side.
 */

namespace vfxgal {

	/*
	 * @brief calc_newell_normal
	 * Calculate the normal of a polygon using Newell's method.
	 * @note It is assumed that the given points are coplanar.
	 * @param points The polygon's points.
	 * @param normalize If true, the normal is normalized.
	 * @param[out] normal The calculated normal.
	 */
	template<typename Points3D>
	void calc_newell_normal(const Points3D& points,
		Imath::Vec3<typename points_adaptor<Points3D>::scalar>& normal,
		bool normalize = true);

	/*
	 * @brief calc_normal
	 * Calculate the normal of a polygon in a robust as possible way. This version of
	 * normal calculation is able to deal with slivers.
	 * @note It is assumed that the given points are coplanar.
	 * @param points The polygon's points.
	 * @param[out] normal The calculated normal.
	 * @returns False if the poly was extremely small area or a sliver. In these cases,
	 * the normal may not be the actual normal of the polygon. In the sliver case though,
	 * it will be guaranteed to be perpendicular to the longest edge.
	 */
	template<typename Points3D>
	bool calc_normal(const Points3D& points,
		Imath::Vec3<typename points_adaptor<Points3D>::scalar>& normal);

	/*
	 * @brief get_poly_plane
	 * Given a polygon, return the plane that it lies in, with the plane normal facing in
	 * the same direction as the polygon normal.
	 * @param points The polygon's points.
	 * @param[out] plane The polygon-incident plane.
	 */
	template<typename Points3D>
	void get_poly_plane(const Points3D& points,
		Imath::Plane3<typename points_adaptor<Points3D>::scalar>& plane);

	/*
	 * @brief intersect
	 * @returns True if the line segment intersects the polygon, false otherwise.
	 */
	template<typename Points3D>
	bool intersect(const Points3D& points,
		const LineSegment<typename points_adaptor<Points3D>::elem_type>& seg);

	/*
	 * @brief get_longest_edge_vector
	 */
	template<typename Points3D>
	void get_longest_edge_vector(const Points3D& points,
		Imath::Vec3<typename points_adaptor<Points3D>::scalar>& edge);

	/*
	 * @brief getGeneralBarycentricCoord
	 * Given a point inside a polygon, return the point's barycentric coordinate.
	 * @param points The polygon's points.
	 * @param p The point to test.
	 * @param[out] c The barycentric coefficients for p. The order and count will match
	 * the input vertices.
	 * @param assume_convex If true, the poly is assumed to be convex.
	 * @note The result is undefined if p lies outside of the polygon, or p and the
	 * polygon are not coplanar.
	 */
	template<typename Points3D>
	void getGeneralBarycentricCoord(const Points3D& points,
		const Imath::Vec3<typename points_adaptor<Points3D>::scalar>& p,
		std::vector<typename points_adaptor<Points3D>::scalar>& c,
		bool assume_convex = true);


///////////////////////// impl

template<typename Points3D>
void calc_newell_normal(const Points3D& points,
	Imath::Vec3<typename points_adaptor<Points3D>::scalar>& normal, bool normalize)
{
	typedef points_adaptor<Points3D> 			Adaptor;
	typedef typename Adaptor::scalar			T;
	typedef Imath::Vec3<T> 						vec3_type;
	typedef typename Adaptor::const_elem_ref	vec3_const_ref;
	Adaptor a(points);

	normal = vec3_type(0,0,0);
	unsigned int nverts = a.size();
	for(unsigned int i=0; i<nverts; ++i)
	{
		vec3_const_ref v1 = a[i];
		vec3_const_ref v2 = a[(i+1) % nverts];
		normal.x -= (v1.y - v2.y) * (v1.z + v2.z);
		normal.y -= (v1.z - v2.z) * (v1.x + v2.x);
		normal.z -= (v1.x - v2.x) * (v1.y + v2.y);
	}

	if(normalize)
		normal.normalize();
}


template<typename Points3D>
bool calc_normal(const Points3D& points,
	Imath::Vec3<typename points_adaptor<Points3D>::scalar>& normal)
{
	typedef points_adaptor<Points3D> 		Adaptor;
	typedef typename Adaptor::scalar		T;
	typedef Imath::Vec3<T>					vec3_type;

	calc_newell_normal(points, normal, false);
	if(normal.length() < std::numeric_limits<T>::epsilon())
	{
		// approx zero-area poly
		if(boost::is_same<T,double>::value)
		{
			// Assume it's a sliver and try to recover
			vec3_type long_edge;
			get_longest_edge_vector(points, long_edge);
			normal = long_edge.cross(axisVector<T>(getSmallestAxis(long_edge)));

			normal.normalize();
			return false;
		}
		else
		{
			// drop into double and try again
			std::vector<Imath::V3d> dpoints;
			convertPoints(points, dpoints);

			Imath::V3d dnorm;
			bool b = calc_normal(dpoints, dnorm);
			normal = vec3_type(dnorm);
			return b;
		}
	}

	normal.normalize();
	return true;
}


template<typename Points3D>
bool intersect(const Points3D& points,
	const LineSegment<typename points_adaptor<Points3D>::elem_type>& seg)
{
	typedef points_adaptor<Points3D> 		Adaptor;
	typedef typename Adaptor::scalar		T;
	typedef Imath::Vec2<T> 					vec2_type;
	typedef Imath::Vec3<T> 					vec3_type;
	typedef Imath::Plane3<T> 				plane3_type;

	vec3_type poly_norm;
	calc_newell_normal(points, poly_norm, false);
	if(poly_norm.length() < std::numeric_limits<T>::epsilon())
		return false;

	Adaptor a(points);
	plane3_type pl(a[0], poly_norm);

	vec3_type pt_intersect;
	if(!intersect(pl, seg.m_start, seg.m_end, pt_intersect))
		return false;

	// project into 2D and solve there
	Projection_2D<T> proj(poly_norm);
	std::vector<vec2_type> pts_2d;
	vec2_type pt_2d;

	proj.project(points, pts_2d);
	proj.project(pt_intersect, pt_2d);

	return isInside(pts_2d, pt_2d);
}


template<typename Points3D>
void get_longest_edge_vector(const Points3D& points,
	Imath::Vec3<typename points_adaptor<Points3D>::scalar>& edge)
{
	typedef points_adaptor<Points3D> 			Adaptor;
	typedef typename Adaptor::scalar			T;
	typedef typename Adaptor::const_elem_ref	vec3_const_ref;
	Adaptor a(points);

	T max_len = T(0);
	unsigned int nverts = a.size();
	for(unsigned int i=0; i<nverts; ++i)
	{
		vec3_const_ref v1 = a[i];
		vec3_const_ref v2 = a[(i+1) % nverts];
		T len = (v2-v1).length2();
		if(len > max_len)
		{
			max_len = len;
			edge = v2-v1;
		}
	}
}


template<typename Points3D>
void get_poly_plane(const Points3D& points,
	Imath::Plane3<typename points_adaptor<Points3D>::scalar>& plane)
{
	typedef points_adaptor<Points3D> 			Adaptor;
	typedef typename Adaptor::scalar			T;
	typedef Imath::Vec3<T> 						vec3_type;
	typedef Imath::Plane3<T> 					plane3_type;
	typedef typename Adaptor::const_elem_ref	vec3_const_ref;
	Adaptor a(points);

	vec3_type norm;
	calc_normal(points, norm);
	plane = plane3_type(a[0], norm);
}


template<typename Points3D>
void getGeneralBarycentricCoord(const Points3D& points,
	const Imath::Vec3<typename points_adaptor<Points3D>::scalar>& p,
	std::vector<typename points_adaptor<Points3D>::scalar>& c,
	bool assume_convex)
{
	typedef points_adaptor<Points3D> 				Adaptor;
	typedef typename Adaptor::scalar				T;
	typedef typename Adaptor::elem_type				vec3_type;
	typedef Imath::Vec2<T>							vec2_type;

	Adaptor a(points);
	unsigned int nverts = a.size();
	c.resize(nverts);

	if(nverts < 3)
		return;

	if(nverts == 3)
	{
		getBarycentricCoord(a[0], a[1], a[2], p, c[0], c[1], c[2]);
	}
	else
	{
		// project into 2D and solve
		vec3_type norm;
		calc_newell_normal(points, norm, false);
		if(norm.length() < std::numeric_limits<T>::epsilon())
		{
			// approx zero-area poly
			if(boost::is_same<T,double>::value)
			{
				// can't do anything more
				std::fill(c.begin(), c.end(), 0);
				c[0] = 1;
			}
			else
			{
				// drop into double and try again
				Imath::V3d dp = p;
				std::vector<Imath::V3d> dpoints;
				std::vector<double> dc;

				convertPoints(points, dpoints);
				getGeneralBarycentricCoord(dpoints, dp, dc);

				for(unsigned int i=0; i<nverts; ++i)
					c[i] = static_cast<T>(dc[i]);
			}

			return;
		}
		else
		{
			norm.normalize();

			std::vector<vec2_type> points_2d;
			vec2_type p_2d;

			Projection_2D<T> proj(norm);
			proj.project(p, p_2d);
			proj.project(points, points_2d);

			getGeneralBarycentricCoord(points_2d, p_2d, c, assume_convex);
		}
	}
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
