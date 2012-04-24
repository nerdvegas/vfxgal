#ifndef _DGAL_PROJECTION__H_
#define _DGAL_PROJECTION__H_

#include <OpenEXR/ImathPlane.h>
#include <OpenEXR/ImathVec.h>
#include <OpenEXR/ImathLine.h>
#include "adaptors/points.hpp"
#include "Line2.hpp"
#include "util.hpp"


namespace dgal {


	/*
	 * @class projection
	 * @brief
	 * Represents a projection from 3D into 2D with respect to a plane. The projection will
	 * occur as if looking down onto the plane from the normal-positive halfspace.
	 */
	template<typename T>
	struct Projection_2D
	{
		typedef Imath::Vec2<T> vec2_type;
		typedef Imath::Vec3<T> vec3_type;

		/*
		 * @brief
		 * See set().
		 */
		Projection_2D(const vec3_type& planeNormal, const vec3_type* up = NULL,
			const vec3_type* origin = NULL) {
			set(planeNormal, up);
		}

		/*
		 * @brief
		 * See set().
		 */
		Projection_2D(const Imath::Plane3<T>& plane, const vec3_type* up = NULL,
			const vec3_type* origin = NULL) {
			set(plane.normal, up);
		}

		/*
		 * @brief set
		 * @param planeNormal The plane to project onto.
		 * @param up Up vector used to orient the projection - this will align with [0,1]
		 * in the 2D coordinate space.
		 * @param origin If supplied, the projected 2D position of this point will become
		 * [0,0] in the 2D coordinate system.
		 */
		void set(const vec3_type& planeNormal, const vec3_type* up = NULL,
			const vec3_type* origin = NULL);

		/*
		 * @brief project
		 * Project a 3D point into 2D.
		 */
		inline void project(const vec3_type& p, vec2_type& p2D) const;

		/*
		 * @brief project
		 * Project a 3D line into 2D.
		 */
		inline void project(const Imath::Line3<T>& l, Line2<T>& l2D) const;

		/*
		 * @brief project
		 * Project a set of points into 2D.
		 */
		template<typename Points3D>
		void project(const Points3D& points, std::vector<vec2_type>& points2D);

		vec3_type m_xaxis;
		vec3_type m_yaxis;
	};

	/*
	 * @brief Print a Projection_2D
	 */
	template<typename T>
	std::ostream& operator <<(std::ostream& os, const Projection_2D<T>& proj);

	typedef Projection_2D<float> Projectionf_2D;
	typedef Projection_2D<double> Projectiond_2D;


///////////////////////// impl

template<typename T>
void Projection_2D<T>::set(const Imath::Vec3<T>& planeNormal, const Imath::Vec3<T>* up,
	const vec3_type* origin)
{
	// todo support origin
	if(up)
	{
		m_xaxis = up->cross(planeNormal);
		m_xaxis.normalize();
		m_yaxis = planeNormal.cross(m_xaxis);
		m_yaxis.normalize();
	}
	else
	{
		Imath::Vec3<T> up = axisVector<T>(getSmallestAxis(planeNormal));
		set(planeNormal, &up);
	}
}


template<typename T>
void Projection_2D<T>::project(const Imath::Vec3<T>& p, Imath::Vec2<T>& p2D) const
{
	p2D = vec2_type(p.dot(m_xaxis), p.dot(m_yaxis));
}


template<typename T>
void Projection_2D<T>::project(const Imath::Line3<T>& l, Line2<T>& l2D) const
{
	vec2_type pos, dir;
	project(l.pos, pos);
	project(l.dir, dir);
	l2D = Line2<T>(pos, dir);
}


template<typename T>
template<typename Points3D>
void Projection_2D<T>::project(const Points3D& points, std::vector<Imath::Vec2<T> >& points2D)
{
	typedef points_adaptor<Points3D> Adaptor;
	Adaptor a(points);

	unsigned int npoints = a.size();
	points2D.resize(npoints);
	for(unsigned int i=0; i<npoints; ++i)
	{
		typename Adaptor::const_elem_ref pt = a[i];
		this->project(pt, points2D[i]);
	}
}


template<typename T>
std::ostream& operator <<(std::ostream& os, const Projection_2D<T>& proj)
{
	return os << "(x" << proj.m_xaxis << " y" << proj.m_yaxis << ')';
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
