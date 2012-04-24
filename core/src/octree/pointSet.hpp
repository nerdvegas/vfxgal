#ifndef _DGAL_POINT_SET__H_
#define _DGAL_POINT_SET__H_

#include <OpenEXR/ImathVec.h>
#include <OpenEXR/ImathBox.h>
#include "cell.hpp"
#include "cube.hpp"
#include "../adaptors/points.hpp"
#include "../adaptors/points.hpp"

namespace dgal {
	/*
	 * @class Octree
	 * @brief
	 * acceleration structure for spatial queries on polygons.
	 */
	template<typename Points3D>
	class PointSet
	{
	public :

		// Convenient typedef
		typedef points_adaptor<Points3D> 			Adaptor;
		typedef typename Adaptor::scalar			T;
		typedef typename Adaptor::elem_type 		point_type;
		typedef Cube<point_type>					cube_type;

		// Ctor
		PointSet(const Points3D & pts);
		// Bounding volumes and stats
		inline void setBoundingBox(const Imath::Box<T> & bb);
		inline void getBoundingCube(cube_type& c) const;
		inline void getSizeRange(T& min, T& max, T& avg) const;
		// DataSet elements accessor
		inline unsigned 					size() const { return _points3d.size();}
		inline void							getBounds(unsigned elememIndex, Imath::Box<T>& box ) const;
		inline void							getBarycenter(unsigned elememIndex, point_type& center ) const;
		bool 								intersect(unsigned elememIndex, const Imath::Box<T>& box);

	private :
		inline void computeBoundingBox();
	private :
		bool 						_bUpdateBox;
		Imath::Box<point_type> 		_bbox;
		Adaptor 					_points3d;
		std::vector<point_type>     _points3d_01;

	};

//// Implementation
//-------------------------------------------------------------------------------------------------
template<typename Points3D>
PointSet<Points3D>::PointSet(const Points3D & pts) :
_points3d(pts)
{
	//
	computeBoundingBox();

	//map positions to [0,1]
	cube_type c;
	getBoundingCube(c);
	point_type tr = c.min();
	T scale = c.size();
	_points3d_01.resize(_points3d.size());
	point_type tmp;
	for (uint i=0;i<_points3d.size();i++)
	{
		tmp = _points3d[i];
		tmp -= tr;
		tmp *= 1/scale;
		_points3d_01[i] = tmp;
	}


}
//-------------------------------------------------------------------------------------------------
template<typename Points3D>
void PointSet<Points3D>::getBoundingCube(cube_type& c) const
{
	point_type diff = _bbox.max-_bbox.min;
	T maxlen = diff.x>diff.y ? diff.x : diff .y;
	maxlen = maxlen > diff.z ? maxlen : diff.z;
	c.set(_bbox.min,maxlen);
}
//-------------------------------------------------------------------------------------------------
template<typename Points3D>
void PointSet<Points3D>::setBoundingBox(const Imath::Box<T> & bbox)
{
	_bbox = bbox;
}
//-------------------------------------------------------------------------------------------------
template<typename Points3D>
void PointSet<Points3D>::computeBoundingBox()
{
	_bbox.makeEmpty();
	for (unsigned i=0; i<_points3d.size(); i++)
	{
		_bbox.extendBy(_points3d[i]);
	}
}
//-------------------------------------------------------------------------------------------------
template<typename Points3D>
void PointSet<Points3D>::getSizeRange(T& min, T& max, T& avg) const
{
	min =0;
	max =0;
	avg =0;
}

//-------------------------------------------------------------------------------------------------
template<typename Points3D>
bool PointSet<Points3D>::intersect(unsigned elememIndex,const Imath::Box<T>& box)
{
	return box.intersect(_points3d_01[elememIndex]);
}

//-------------------------------------------------------------------------------------------------
template<typename Points3D>
void PointSet<Points3D>::getBounds(unsigned elememIndex, Imath::Box<T>& box) const
{
	box.min =  _points3d_01[elememIndex];
	box.max =  _points3d_01[elememIndex];
}

//-------------------------------------------------------------------------------------------------
template<typename Points3D>
void PointSet<Points3D>::getBarycenter(unsigned elememIndex, point_type& center ) const
{
	center = _points3d_01[elememIndex];
}


}//dgal

#endif // _DGAL_POINT_SET__H_


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
