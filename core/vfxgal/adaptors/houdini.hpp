#ifndef _DGAL_ADAPTORS_HOUDINI__H_
#define _DGAL_ADAPTORS_HOUDINI__H_

#include <GEO/GEO_PointList.h>
#include <GEO/GEO_PrimPoly.h>
#include "points.hpp"
#include "mesh.hpp"

namespace vfxgal {


	/*
	 * @brief GEO_PointList points specialization
	 */
	template<>
	struct points_adaptor<GEO_PointList>
	{
		typedef Imath::V3f		elem_type;
		typedef Imath::V3f		const_elem_ref;
		typedef GEO_PointList	target_type;
		typedef float			scalar;

		points_adaptor(const target_type& t):m_target(t){}
		inline unsigned int size() const { return m_target.entries(); }
		inline unsigned int index(unsigned int i) const { return i; }
		inline bool is_indexed() const { return false; }

		inline const_elem_ref operator[](unsigned int i) const
		{
			assert(i < m_target.entries());
			const UT_Vector4& p = m_target[i]->getPos();
			return const_elem_ref(p.x(), p.y(), p.z());
		}

		const target_type& m_target;
	};


	/*
	 * @brief GEO_PrimPoly points specialization
	 */
	template<>
	struct points_adaptor<GEO_PrimPoly>
	{
		typedef Imath::V3f		elem_type;
		typedef Imath::V3f		const_elem_ref;
		typedef GEO_PrimPoly	target_type;
		typedef float			scalar;

		points_adaptor(const target_type& t):m_target(t){}
		inline unsigned int size() const { return m_target.getVertexCount(); }
		inline bool is_indexed() const { return true; }

		inline unsigned int index(unsigned int i) const {
			return m_target.getVertex(i).getPt()->getNum();
		}

		inline const_elem_ref operator[](unsigned int i) const {
			const UT_Vector4& p = m_target.getVertex(i).getPt()->getPos();
			return const_elem_ref(p.x(), p.y(), p.z());
		}

		const target_type& m_target;
	};


	/*
	 * @brief GEO_Detail mesh specialization
	 */
	template<>
	struct mesh_adaptor<GEO_Detail>
	{
		typedef GEO_Detail						target_type;
		typedef float							scalar;
		typedef unsigned int					index_type;
		typedef Imath::V3f						point_type;
		typedef Imath::V3f						const_point_ref;
		typedef GEO_PrimPoly					poly_type;
		typedef const GEO_PrimPoly&				const_poly_ref;

		mesh_adaptor(const target_type& t):m_target(t){}

		inline unsigned int numPoints() const { return m_target.points().entries(); }
		inline unsigned int numPolys() const { return m_target.primitives().entries(); }

		inline const_point_ref getPoint(unsigned int i) const
		{
			assert(i < m_target.points().entries());
			const UT_Vector4& p = m_target.points()[i]->getPos();
			return const_point_ref(p.x(), p.y(), p.z());
		}

		inline const_poly_ref getPoly(unsigned int i) const {
			return *(reinterpret_cast<const GEO_PrimPoly*>(m_target.primitives()[i]));
		}

		const target_type& m_target;
	};

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
