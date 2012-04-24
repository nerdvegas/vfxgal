#ifndef _DGAL_CUBE__H_
#define _DGAL_CUBE__H_

#include <OpenEXR/ImathVec.h>
#include <OpenEXR/ImathBox.h>
#include "../plane.hpp"

namespace vfxgal {

	/*
	 * @class Cube
	 * @brief
	 * Cube defined as a point and a size
	 * supporting simple transformations
	 */
	template<typename pointType>
	class Cube
	{
	public :
		typedef pointType 						vec3_type;
		typedef typename vec3_type::BaseType 	T;
		//
		Cube(vec3_type pMin, T size ) : _pMin(pMin),_size(size) {}
		Cube(vec3_type pMin, vec3_type pMax );
		Cube() : _pMin(0),_size(0){}
		//Set
		void set( const vec3_type& min, T size);
		//Get
		T size() const { return _size;}
		const vec3_type&  min() const { return _pMin;}
		//Contains
		inline bool contains(vec3_type p) const;
		inline bool contains(const Cube& other) const;
		inline bool contains(const Imath::Box<vec3_type>& box) const;
		//Intersect bbox, plane, AAline ...
		inline bool intersect(const Imath::Box<vec3_type>& box) const;
		int 		intersect(const Imath::Plane3<T>& plane,T epsilon ) const;
		bool 		intersect(const pointType& ptStart,short axis,bool castForward) const;
		//Transforms
		void translate(const vec3_type& v) { _pMin += v;}
		void scale(T s) { _size*=s;}

	private :
		vec3_type 			_pMin;
		T					_size;
		static float 		_epsilon;
	};


//// Implementation
template< typename pointType >
float Cube< pointType >::_epsilon= 1e-4f;
//-------------------------------------------------------------------------------------------------
template<typename pointType>
Cube<pointType>::Cube(vec3_type pMin, vec3_type pMax )
{
	vec3_type diff = pMax - pMin;
	assert( fabs(diff.x - diff.y)<1e-4f && fabs(diff.x -diff.z)<1e-4f );
	//
	_pMin = pMin;
	_size = diff.x;


}
//-------------------------------------------------------------------------------------------------
template<typename pointType>
bool Cube<pointType>::contains(vec3_type p) const
{
	if ( p.x < _pMin.x-_epsilon || p.x > _pMin.x+_size+_epsilon ||
		 p.y < _pMin.y-_epsilon || p.y > _pMin.y+_size+_epsilon ||
		 p.z < _pMin.z-_epsilon || p.z > _pMin.z+_size+_epsilon)
		return false;
	return true;
}
//-------------------------------------------------------------------------------------------------
template<typename pointType>
bool Cube<pointType>::contains(const Cube& other) const
{

	return contains(other._pMin) && contains(other._pMin + vec3_type(other._size,other._size,other._size));
}
//-------------------------------------------------------------------------------------------------
template<typename pointType>
bool Cube<pointType>::contains(const Imath::Box<vec3_type>& box) const
{

	return contains(box.min) && contains(box.max);
}
//-------------------------------------------------------------------------------------------------
template<typename pointType>
void Cube<pointType>::set( const vec3_type& min, T size)
{
	_pMin = min;
    _size = size;
}

//-------------------------------------------------------------------------------------------------
template<typename pointType>
bool Cube<pointType>::intersect(const Imath::Box<vec3_type>& box) const
{
	vec3_type pMax = _pMin + vec3_type(1,1,1)*_size;
	vec3_type pMin = _pMin;
	vec3_type bMin = box.min;
	vec3_type bMax = box.max;
	bool bNotIntersect = 	bMin.x > pMax.x || bMin.y >pMax.y || bMin.z >pMax.z ||
							bMax.x < pMin.x || bMax.y < pMin.y || bMax.z < pMin.z;
	return 	!bNotIntersect;
}

//-------------------------------------------------------------------------------------------------
template<typename pointType>
int Cube<pointType>::intersect(const Imath::Plane3<T>& plane,T epsilon ) const
{
	int nLocation =  	  locatePoint( plane, _pMin, epsilon )
					+ locatePoint( plane, _pMin + vec3_type(0,_size,0),epsilon)
					+ locatePoint( plane, _pMin + vec3_type(0,0,_size),epsilon)
					+ locatePoint( plane, _pMin + vec3_type(0,_size,_size),epsilon)
					+ locatePoint( plane, _pMin + vec3_type(_size,0,0),epsilon)
					+ locatePoint( plane, _pMin + vec3_type(_size,_size,0),epsilon)
					+ locatePoint( plane, _pMin + vec3_type(_size,0,_size),epsilon)
					+ locatePoint( plane, _pMin + vec3_type(_size,_size,_size),epsilon);

	return   (nLocation==-8) ? -1 : ( (nLocation == 8)?  1 : 0);
}

//-------------------------------------------------------------------------------------------------
template<typename pointType>
bool Cube<pointType>::intersect(const pointType& ptStart,short axis,bool castForward) const
{
	vec3_type pMax = _pMin + vec3_type(1,1,1)*_size;
	vec3_type pMin = _pMin;
	assert(axis<3);

	if (castForward)
	{
		if (ptStart[axis]>pMax[axis])
			return false;
	}
	else
	{
		if (ptStart[axis]<pMin[axis])
			return false;
	}

	short ax0 = (axis+1)>2 ? 0 :axis+1;
	short ax1 = (axis-1)<0 ? 2 :axis-1;

	return !( ptStart[ax0]<pMin[ax0] || ptStart[ax0]>pMax[ax0] ||
			  ptStart[ax1]<pMin[ax1] || ptStart[ax1]>pMax[ax1] );


}

}

#endif //_DGAL_CUBE__H_




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
