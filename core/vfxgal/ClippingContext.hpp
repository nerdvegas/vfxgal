#ifndef _DGAL_CLIPPING_CONTEXT__H_
#define _DGAL_CLIPPING_CONTEXT__H_

#include "enums.hpp"
#include "Line2.hpp"
#include <cassert>
#include <map>
#include <OpenEXR/ImathPlane.h>


namespace vfxgal { namespace detail {

	template<typename T>
	struct ClippingContext
	{
		struct poly_data
		{
			IntersectType m_intersectType;
			bool m_cannotIntersectPlane;
		};

		inline bool isPointInside(unsigned int index) const {
			assert(index < m_isPtInside.size());
			return m_isPtInside[index];
		}

		std::vector<bool> m_isPtInside;
		std::vector<poly_data> m_polyData;
		int m_currentPolyId;
	};

} }

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
