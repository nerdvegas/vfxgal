#ifndef _DGAL_POLY__H_
#define _DGAL_POLY__H_

#include "adaptors/points_indexer.hpp"
#include "util.hpp"
#include <iterator>
#include <iostream>


namespace dgal {

	/*
	 * @brief convertPoints
	 * Convert a set of points into a different type. For example, you might want to
	 * convert a set of float-based points into double-based points.
	 */
	template<typename Points, typename OutputContainer>
	void convertPoints(const Points& pts, OutputContainer& result);

	/*
	 * @brief getBarycentricCoord
	 * Given a point p in or on triangle p1,p2,p3, calculate the barycentric coord of p.
	 * This function works for both 2D and 3D polys, but in the 3D case it is assumed that
	 * the poly and test point will be coplanar.
	 * @note c1,c2,c3 are undefined if p is outside the triangle.
	 */
	template<typename Vec>
	void getBarycentricCoord(const Vec& p1, const Vec& p2, const Vec& p3, const Vec& p,
		typename Vec::BaseType& c1, typename Vec::BaseType& c2, typename Vec::BaseType& c3);

	/*
	 * @brief Print a poly
	 */
	template<typename Points>
	struct PolyPrinter
	{
		PolyPrinter(const Points& pts):m_points(pts){}
		const Points& m_points;
	};

	template<typename Points>
	std::ostream& operator <<(std::ostream& os, const PolyPrinter<Points>& pp);


///////////////////////// impl

template<typename Points, typename OutputContainer>
void convertPoints(const Points& pts, OutputContainer& result)
{
	typedef points_adaptor<Points> Adaptor;
	typedef typename OutputContainer::value_type out_vec_type;

	Adaptor a(pts);
	result.clear();
	unsigned int npoints = a.size();

	for(unsigned int i=0; i<npoints; ++i)
		result.push_back(out_vec_type(a[i]));
}


template<typename Vec>
void getBarycentricCoord(const Vec& p1, const Vec& p2, const Vec& p3, const Vec& p,
	typename Vec::BaseType& c1, typename Vec::BaseType& c2, typename Vec::BaseType& c3)
{
	typedef Vec												vec_type;
	typedef typename Vec::BaseType 							T;
	typedef typename imath_cross_product_traits<Vec>::type	cross_prod_type;
	typedef calc_length<cross_prod_type>					length_calc;
	typedef typename imath_rebind<Vec, double>::type		vecd_type;

	vec_type p12 = p2-p1;
	vec_type p13 = p3-p1;

	T area = std::abs(length_calc::get(p12.cross(p13))); // missing *0.5 intentional
	if(area < std::numeric_limits<T>::epsilon())
	{
		// drop into double and solve there
		if(boost::is_same<T,double>::value)
			c1 = c2 = c3 = T(1)/T(3);
		else
		{
			vecd_type pd1(p1);
			vecd_type pd2(p2);
			vecd_type pd3(p3);
			vecd_type pd(p);

			double cd1, cd2, cd3;
			getBarycentricCoord(pd1, pd2, pd3, pd, cd1, cd2, cd3);
			c1 = static_cast<T>(cd1);
			c2 = static_cast<T>(cd2);
			c3 = static_cast<T>(cd3);
		}
		return;
	}

	// c2
	vec_type p1p = p-p1;
	T subarea = std::abs(length_calc::get(p1p.cross(p13)));
	c2 = subarea/area;

	// c3
	subarea = std::abs(length_calc::get(p12.cross(p1p)));
	c3 = subarea/area;

	// c1
	c1 = 1 - c2 - c3;

	// normalise
	T n = 1.0 / (c1+c2+c3);
	c1 *= n;
	c2 *= n;
	c3 *= n;
}


template<typename Points>
std::ostream& operator <<(std::ostream& os, const PolyPrinter<Points>& pp)
{
	typedef points_adaptor<Points> Adaptor;

	Adaptor a(pp.m_points);
	unsigned int npoints = a.size();

	os << '(';
	for(unsigned int i=0; i<npoints; ++i)
	{
		os << a.index(i) << ':' << a[i];
		if(i < npoints-1)
			os << ' ';
	}
	os << ')';
	return os;
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
