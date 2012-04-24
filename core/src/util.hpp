#ifndef _DGAL_UTIL__H_
#define _DGAL_UTIL__H_

#include <OpenEXR/ImathVec.h>


namespace dgal {

	/*
	 * @brief getSmallestAxis
	 * Given a vector v, return the index of the smallest-length component of v.
	 */
	template<typename T>
	inline unsigned int getSmallestAxis(const Imath::Vec3<T>& v)
	{
		Imath::Vec3<T> v_ = v*v;
		return (v_.x<v_.y)?
			( (v_.x<v_.z)? 0 : 2 ) :
			( (v_.y<v_.z)? 1 : 2 );
	}


	/*
	 * @brief getLargestAxis
	 * Given a vector v, return the index of the largest-length component of v.
	 */
	template<typename T>
	inline unsigned int getLargestAxis(const Imath::Vec3<T>& v)
	{
		Imath::Vec3<T> v_ = v*v;
		return (v_.x>v_.y)?
			( (v_.x>v_.z)? 0 : 2 ) :
			( (v_.y>v_.z)? 1 : 2 );
	}


	/*
	 * @brief axisVector
	 * Given an axis, return a unit vector in the positive direction of that axis.
	 */
	template<typename T>
	inline Imath::Vec3<T> axisVector(unsigned int axis)
	{
		Imath::Vec3<T> v(0,0,0);
		v[axis%3] = 1;
		return v;
	}


	/*
	 * @brief Imath type rebinding
	 */
	template<typename T, typename Scalar>
	struct imath_rebind {
		typedef T type;
	};

	template<typename T, typename Scalar>
	struct imath_rebind<Imath::Vec2<T>, Scalar> {
		typedef Imath::Vec2<Scalar> type;
	};

	template<typename T, typename Scalar>
	struct imath_rebind<Imath::Vec3<T>, Scalar> {
		typedef Imath::Vec3<Scalar> type;
	};


	/*
	 * @brief Misc imath type traits
	 */
	template<typename T>
	struct imath_cross_product_traits {
		typedef T type;
	};

	template<typename T>
	struct imath_cross_product_traits<Imath::Vec2<T> > {
		typedef T type;
	};


	/*
	 * @brief Generic length calculation
	 */
	template<typename T>
	struct calc_length {
		static inline T get(const T& t) { return t; }
	};

	template<typename T>
	struct calc_length<Imath::Vec2<T> > {
		static inline T get(const Imath::Vec2<T>& t) { return t.length(); }
	};

	template<typename T>
	struct calc_length<Imath::Vec3<T> > {
		static inline T get(const Imath::Vec3<T>& t) { return t.length(); }
	};

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
