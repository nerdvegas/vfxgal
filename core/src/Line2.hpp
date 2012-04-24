#ifndef _DGAL_LINE__H_
#define _DGAL_LINE__H_

#include <OpenEXR/ImathVec.h>
#include <limits>


namespace dgal {

	/*
	 * @class Line2
	 * @brief
	 * Line class to complement Imath's Line3. Our representation is actually a 2D
	 * halfspace - the 'right' vector is the halfspace normal.
	 */
	template<typename T>
	class Line2
	{
	public:

		typedef Imath::Vec2<T> vec_type;
		vec_type right;
		T distance;

		Line2(){}

		template<typename S>
		Line2(const Line2<S>& rhs):right(rhs.right),distance(rhs.distance){}

		template<typename S>
		Line2(const Imath::Vec2<S>& pos, const Imath::Vec2<S>& dir) {
			right = vec_type(dir.y, -dir.x).normalized();
			distance = pos.dot(right);
		}

		/*
		 * @brief along
		 * @returns The line's directional vector.
		 */
		inline vec_type dir() const {
			return vec_type(-right.y, right.x);
		}

		/*
		 * @brief isRight
		 * @returns True if the given point is to the right of the line or is ON the line,
		 * and false otherwise. Because the 'right' side of the halfspace is in the positive
		 * normal area, isRight can also be considered an 'inside' test.
		 */
		inline bool isRight(const vec_type& p) const {
			return (p.x*right.x + p.y*right.y) >= distance;
		}

		/*
		 * @brief isLeft
		 * @returns True if the given point is to the left of the line, false otherwise.
		 */
		inline bool isLeft(const vec_type& p) const {
			return !isRight(p);
		}

		/*
		 * @brief signedDistance
		 * @returns The signed distance between p and the line. If positive, p is to the
		 * right of the line; if negative, to the left; and if zero, p is on the line.
		 */
		inline T signedDistance(const vec_type& p) const {
			return (p.x*right.x + p.y*right.y) - distance;
		}

		/*
		 * @brief reverse
		 * Reverse the direction of the halfspace, so the positive side becomes the
		 * negative and vice-versa.
		 */
		inline void reverse() {
			right=-right;
			distance=-distance;
		}

		/*
		 * @brief intersect
		 * Intersect a line passing through p1 and p2 with this line.
		 * @param p1,p2 The points the line passes through.
		 * @param[out] u The point of intersection as a fraction. p1+(p2-p1)*u will give the
		 * actual intersection point.
		 * @returns True if an intersection exists, false otherwise (the lines are parallel).
		 */
		bool intersect(const vec_type& p1, const vec_type& p2, T& u) const;
	};

	/*
	 * @brief Print a Line2
	 */
	template<typename T>
	std::ostream& operator <<(std::ostream& os, const Line2<T>& l);

	typedef Line2<float> 	Line2f;
	typedef Line2<double> 	Line2d;


///////////////////////// impl

template<typename T>
bool Line2<T>::intersect(const vec_type& p1, const vec_type& p2, T& u) const
{
	vec_type p1_2 = p2 - p1;
	T d = p1_2.dot(this->right);
	if(d*d < std::numeric_limits<T>::epsilon())
	{
		u = 0;
		return false;
	}

	T d1 = p1.dot(this->right);
	u = (this->distance - d1) / d;
	return true;
}


template<typename T>
std::ostream& operator <<(std::ostream& os, const Line2<T>& l)
{
	return os << '(' << l.right << ' ' << l.distance << ')';
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
