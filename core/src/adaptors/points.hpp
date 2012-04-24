#ifndef _DGAL_ADAPTORS_POINTS__H_
#define _DGAL_ADAPTORS_POINTS__H_

#include <vector>
#include <OpenEXR/ImathVec.h>


namespace vfxgal {

	/*
	 * @class points_adaptor
	 * @brief
	 * Adaptor for plugging various data sources into vfxgal algorithms to represent a list
	 * of points. The default implementation can be templatised on std::vector<U>, where
	 * U is any Imath Vec type.
	 * @note A list of points is often also used to represent a polygon.
	 */
	template<typename T>
	struct points_adaptor
	{
		typedef T								target_type;
		typedef typename T::value_type			elem_type;
		typedef const elem_type&				const_elem_ref;
		typedef typename elem_type::BaseType	scalar;

		points_adaptor(const target_type& t):m_target(t){}
		inline unsigned int size() const { return m_target.size(); }
		inline const_elem_ref operator[](unsigned int i) const { return m_target[i]; }
		inline unsigned int index(unsigned int i) const { return i; }
		inline bool is_indexed() const { return false; }

		const target_type& m_target;
	};


	namespace detail {

		template<typename T>
		struct custom_points_adaptor
		{
			typedef typename T::elem_type		elem_type;
			typedef typename T::const_elem_ref	const_elem_ref;
			typedef typename T::scalar			scalar;

			custom_points_adaptor(const T& t):m_target(t){}
			inline unsigned int size() const 					{ return m_target.size(); }
			inline unsigned int index(unsigned int i) const 	{ return m_target.index(i); }
			inline bool is_indexed() const 						{ return m_target.is_indexed(); }

			inline const_elem_ref operator[](unsigned int i) const
			{
				assert(i < m_target.size());
				return m_target[i];
			}

			const T& m_target;
		};

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
