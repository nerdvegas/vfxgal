#ifndef _DGAL_POLYGON_SET__H_
#define _DGAL_POLYGON_SET__H_

#include <OpenEXR/ImathVec.h>
#include <OpenEXR/ImathBox.h>
#include "cell.hpp"
#include "cube.hpp"
#include "../adaptors/mesh.hpp"
#include "../adaptors/points.hpp"

namespace dgal {
	/*
	 * @class PolygonSet
	 * @brief
	 * a Collection of polygons.
	 */
	template<typename Mesh3D>
	class PolygonSet
	{
	public :

		// Convenient typedef
		typedef Mesh3D									mesh_type;
		typedef mesh_adaptor<Mesh3D>					MeshAdaptor;
		typedef typename MeshAdaptor::point_type		point_type;
		typedef typename MeshAdaptor::index_type		index_type;
		typedef typename MeshAdaptor::poly_type			poly_type;
		typedef typename MeshAdaptor::scalar			T;
		typedef Cube<point_type>						cube_type;

		// Ctor Dtor if components == NULL use all mesh faces
		PolygonSet(const Mesh3D& mesh,const cube_type* transform=NULL);

		const Mesh3D& mesh() const { return *_mesh3d; }

		// Bounding volumes and stats
		inline void 						getTotalBounds(Imath::Box<point_type>& box) const;
		inline void 						getBoundingCube(cube_type& c) const;
		inline void 						getSizeRange(T& min, T& max, T& avg) const;
		// DataSet elements accessor
		inline unsigned 					size() const { return  _pPerElemBox.size();}
		inline void							getBounds(unsigned elemIndex, Imath::Box<point_type>& box) const;
		inline void							getBarycenter(unsigned elemIndex, point_type& center ) const;

	private :
		void 								addElement(const points_adaptor<poly_type>& pAdapt,const point_type& tr,const T& scale);
	private :

		// Mesh
		const Mesh3D*			 				_mesh3d;
		Imath::Box<point_type> 					_bbox;
		//
		std::vector< Imath::Box<point_type> >	_pPerElemBox;
		T										_minElemSize;
		T										_maxElemSize;
		T										_avgElemSize;
	};


//-------------------------------------------------------------------------------------------------
template<typename Mesh3D>
void PolygonSet<Mesh3D>::addElement(const points_adaptor<poly_type>& pAdapt,const point_type& tr,const T& scale)
{
	Imath::Box<point_type> 	box;
	for (unsigned iv=0; iv<pAdapt.size() ; iv++)
	{
		typename points_adaptor<poly_type>::const_elem_ref 	p = pAdapt[iv];
		typename points_adaptor<poly_type>::elem_type	  	p01 = p;
		p01 -= tr;
		p01 *= 1/scale;

		box.extendBy(p01);
	}
	point_type offset = box.size()*0.001;
	box.min = box.min - offset;
	box.max = box.max + offset;
	//assert(box.size().x>0.001);
	//assert(box.size().y>0.001);
	//assert(box.size().z>0.001);
	//
	_pPerElemBox.push_back(box);
	//
	point_type s = box.size();
	T majLen = s[box.majorAxis()];

	_minElemSize = (_minElemSize<majLen) ? _minElemSize : majLen;
	_maxElemSize = (_maxElemSize>majLen) ? _maxElemSize : majLen;
	_avgElemSize += majLen;

}

//// Implementation
//-------------------------------------------------------------------------------------------------
template<typename Mesh3D>
PolygonSet<Mesh3D>::PolygonSet(const Mesh3D& mesh,const cube_type* transform)
{
	Imath::Box<point_type> 	bbox;
	_mesh3d= &mesh;

	//build a mesh adaptor
	MeshAdaptor meshAdaptor(mesh);
	//compute bbox.
	_bbox.makeEmpty();
	for (unsigned i=0; i<meshAdaptor.numPoints(); i++)
	{
		_bbox.extendBy(meshAdaptor.getPoint(i));
	}

	_bbox.min = _bbox.min - _bbox.size()*0.001;
	_bbox.max = _bbox.max + _bbox.size()*0.001;

	//build per comp bbox
	unsigned nPolys = meshAdaptor.numPolys();
	assert (std::numeric_limits<T>::is_integer ==false);
	_minElemSize = std::numeric_limits<T>::max();
	_maxElemSize = std::numeric_limits<T>::min();
	_avgElemSize = 0;

	//dataSet transformations
	T scale;
	point_type translate;
	if (transform!=NULL)
	{
		scale = transform->size();
		translate = transform->min();
	}
	else
	{
		cube_type c;
		getBoundingCube(c);
		scale = c.size();
		translate = c.min();
	}
	//add elements

	for (uint i=0; i<nPolys;i++)
	{
		points_adaptor<poly_type> pAdapt(meshAdaptor.getPoly(i));
		addElement(pAdapt,translate,scale);
	}
	if (nPolys>0)
	{
		_avgElemSize /= (T)nPolys;
	}
}



//-------------------------------------------------------------------------------------------------
template<typename Mesh3D>
void PolygonSet<Mesh3D>::getSizeRange(T& min, T& max, T& avg) const
{
	min = _minElemSize;
	max = _maxElemSize;
	avg = _avgElemSize;
}


//-------------------------------------------------------------------------------------------------
template<typename Mesh3D>
void PolygonSet<Mesh3D>::getTotalBounds(Imath::Box<point_type>& box) const
{
	box = _bbox;
}


//-------------------------------------------------------------------------------------------------
template<typename Mesh3D>
void PolygonSet<Mesh3D>::getBounds(unsigned elemIndex,Imath::Box<point_type>& box ) const
{
	assert(elemIndex<_pPerElemBox.size());
	box = _pPerElemBox[elemIndex];
}


//-------------------------------------------------------------------------------------------------
template<typename Mesh3D>
void PolygonSet<Mesh3D>::getBarycenter(unsigned elemIndex, point_type& center ) const
{
	assert(elemIndex<_pPerElemBox.size());
	//Todo : return the face barycenter instead of the elem bbox center
	center = _pPerElemBox[elemIndex].center();
}
//-------------------------------------------------------------------------------------------------
template<typename Points3D>
void PolygonSet<Points3D>::getBoundingCube(cube_type& c) const
{

	point_type diff = _bbox.max-_bbox.min;
	T maxlen = diff.x>diff.y ? diff.x : diff .y;
	maxlen = maxlen > diff.z ? maxlen : diff.z;
	c.set(_bbox.min,maxlen);
}

}//dgal

#endif // _DGAL_POLYGON_SET__H_


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
