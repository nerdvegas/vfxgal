#ifndef _DGAL_POLY2D__H_
#define _DGAL_POLY2D__H_

#include <OpenEXR/ImathVec.h>
#include <OpenEXR/ImathBox.h>
#include <OpenEXR/ImathBoxAlgo.h>
#include <boost/mpl/assert.hpp>
#include <boost/type_traits/is_same.hpp>
#include <boost/unordered_map.hpp>
#include <list>
#include <set>
#include <map>
#include <limits>
#include <algorithm>
#include <iterator>
#include "Line2.hpp"
#include "LineSegment.hpp"
#include "line_segment_2D.hpp"
#include "edge.hpp"
#include "poly.hpp"
#include "points.hpp"


namespace dgal {

	/*
	 * @brief is_convex
	 * @returns True if the poly is convex, false otherwise.
	 */
	template<typename Points2D>
	bool is_convex(const Points2D& points);

	/*
	 * @brief isInside
	 * @param points The polygon's points, in clockwise order.
	 * @param p The point to test.
	 * @returns True if the point p is inside the polygon, false otherwise.
	 */
	template<typename Points2D>
	bool isInside(const Points2D& points,
		const Imath::Vec2<typename points_adaptor<Points2D>::scalar>& p);

	/*
	 * @brief signedArea
	 * @returns The signed area of a poly - positive if clockwise, negative if anticlockwise.
	 */
	template<typename Points2D>
	typename points_adaptor<Points2D>::scalar signedArea(const Points2D& points);

	/*
	 * @brief isClockwise
	 * @returns True if the poly is clockwise, false otherwise.
	 */
	template<typename Points2D>
	bool isClockwise(const Points2D& points) {
		return (signedArea(points) >= 0);
	}

	/*
	 * @brief getGeneralBarycentricCoord
	 * Given a point inside a polygon, return the point's barycentric coordinate.
	 * @param points The polygon's points.
	 * @param p The point to test.
	 * @param[out] c The barycentric coefficients for p. The order and count will match
	 * the input vertices.
	 * @param assume_convex If true, the poly is assumed to be convex.
	 * @note The result is undefined if p lies outside of the polygon.
	 */
	template<typename Points2D>
	void getGeneralBarycentricCoord(const Points2D& points,
		const Imath::Vec2<typename points_adaptor<Points2D>::scalar>& p,
		std::vector<typename points_adaptor<Points2D>::scalar>& c,
		bool assume_convex = true);


///////////////////////// impl

template<typename Points2D>
bool is_convex(const Points2D& points)
{
	typedef points_adaptor<Points2D> 			Adaptor;
	typedef typename Adaptor::const_elem_ref	const_point_ref;
	typedef typename Adaptor::scalar			T;
	Adaptor a(points);

	unsigned int npoints = a.size();
	if(npoints <= 3)
		return true;

	bool pos = true;
	bool neg = true;

	for(unsigned int i=0; (i<npoints) && (pos || neg); ++i)
	{
		const_point_ref p = a[i];
		const_point_ref pprev = a[(i+npoints-1) % npoints];
		const_point_ref pnext = a[(i+1) % npoints];
		T cross = (pprev - p).cross(pnext - p);
		pos &= (cross >= 0);
		neg &= (cross <= 0);
	}

	return (pos || neg);
}


template<typename Points2D>
bool isInside(const Points2D& points,
	const Imath::Vec2<typename points_adaptor<Points2D>::scalar>& p)
{
	typedef points_adaptor<Points2D> 			Adaptor;
	typedef typename Adaptor::scalar			T;
	typedef typename Adaptor::const_elem_ref	const_point_ref;

	Adaptor a(points);
	unsigned int nverts = a.size();

	// http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html
	unsigned int i, j;
	bool c = false;

	for (i=0, j=nverts-1; i<nverts; j=i++)
	{
		const_point_ref pi = a[i];
		const_point_ref pj = a[j];

		if(	((pi.y>p.y) != (pj.y>p.y)) &&
			(p.x < (pj.x-pi.x) * (p.y-pi.y) / (pj.y-pi.y) + pi.x) )
		{
			c = !c;
		}
	}

	return c;
}


template<typename Points2D>
typename points_adaptor<Points2D>::scalar signedArea(const Points2D& points)
{
	typedef points_adaptor<Points2D> 			Adaptor;
	typedef typename Adaptor::const_elem_ref	const_point_ref;
	typedef typename Adaptor::scalar			T;
	Adaptor a(points);

	T area = 0;
	unsigned int npoints = a.size();
	for(unsigned int i=0; i<npoints; ++i)
	{
		const_point_ref p1 = a[i];
		const_point_ref p2 = a[(i+1)%npoints];
		area += p2.cross(p1);
	}

	return area / 2;
}


// todo deal with non-convex case
template<typename Points2D>
void getGeneralBarycentricCoord(const Points2D& points,
	const Imath::Vec2<typename points_adaptor<Points2D>::scalar>& p,
	std::vector<typename points_adaptor<Points2D>::scalar>& c,
	bool assume_convex)
{
	typedef points_adaptor<Points2D> 				Adaptor;
	typedef typename Adaptor::scalar				T;
	typedef typename Adaptor::const_elem_ref		const_point_ref;
	typedef typename Adaptor::elem_type				vec2_type;
	typedef LineSegment<vec2_type>					lineseg_type;

	const T epsilon = std::numeric_limits<T>::epsilon();

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
		// Implementation of Wachspress method, which is valid for convex polys
		// <Wachspress>
		std::vector<T> triarea(nverts);

		for(unsigned int i=0; i<nverts; ++i)
		{
			unsigned int j = (i+1) % nverts;
			const_point_ref pi = a[i];
			const_point_ref pj = a[j];

			vec2_type edge = pj - pi;
			vec2_type to_p = p - pi;
			triarea[i] = std::abs(to_p.cross(edge));

			if(triarea[i] < epsilon)
			{
				if(boost::is_same<T,double>::value)
				{
					if(lineseg_type(pi, pj).getClosestDistance(p) < epsilon)
					{
						// p is on poly vert/edge
						std::fill(c.begin(), c.end(), 0);

						T edge_len2 = edge.length2();
						if(edge_len2 < (epsilon*epsilon))
							c[i] = 1;
						else
						{
							T frac = to_p.dot(edge) / edge_len2;
							frac = std::min(std::max(frac,T(0)), T(1));
							c[j] = frac;
							c[i] = 1 - frac;
						}

						return;
					}
					else
					{
						// edge case. p is colinear with an edge that it does not lie on
						// (this is caused by colinear/concave poly edges). Just keep going,
						// this vert's contribution will be dropped.
					}
				}
				else
				{
					// can't continue - drop into double and solve
					std::vector<Imath::V2d> pointsd;
					convertPoints(points, pointsd);

					Imath::V2d pd = p;
					std::vector<double> cd;
					getGeneralBarycentricCoord(pointsd, pd, cd);

					for(unsigned int k=0; k<nverts; ++k)
						c[k] = static_cast<T>(cd[k]);

					return;
				}
			}
		}

		T total = 0;
		for(unsigned int i=0; i<nverts; ++i)
		{
			unsigned int prev = (i+nverts-1) % nverts;
			unsigned int next = (i+1) % nverts;

			T area = std::abs((a[next] - a[prev]).cross(a[next] - a[i]));
			T a1_a2 = triarea[i] * triarea[prev];
			c[i] = (a1_a2 > epsilon)? area/a1_a2 : 0;
			total += c[i];
		}
		// </Wachspress>

		// normalise
		if(total < epsilon)
		{
			// something's gone wrong, give up and just give 100% weight to first vert
			std::fill(c.begin(), c.end(), 0);
			c[0] = 1;
		}
		else
		{
			for(unsigned int i=0; i<nverts; ++i)
				c[i] /= total;
		}
	}
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
