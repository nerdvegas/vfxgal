#ifndef _DGAL_ADAPTORS_POINTS_INDEXER__H_
#define _DGAL_ADAPTORS_POINTS_INDEXER__H_

#include "points.hpp"
#include <boost/mpl/bool.hpp>
#include <assert.h>

namespace dgal {

	/*
	 * @class points_indexer
	 * @brief
	 * This class and the following points_adaptor specialization allow us to pass a set of
	 * indexed points into dgal, rather than the points directly.
	 */
	template<typename Points>
	struct points_indexer
	{
		typedef points_adaptor<Points> 				Adaptor;
		typedef typename Adaptor::scalar			scalar;
		typedef typename Adaptor::elem_type			elem_type;
		typedef typename Adaptor::const_elem_ref	const_elem_ref;

		points_indexer(const unsigned int* begin, const unsigned int* end,
			const Points& points, bool remap_reads = true)
		: m_begin(begin), m_size(end-begin), m_remapReads(remap_reads), m_a(points){}

		inline unsigned int size() const 				{ return m_size; }
		inline bool is_indexed() const 					{ return true; }

		inline unsigned int index(unsigned int i) const {
			assert(i < m_size);
			return m_begin[i];
		}

		inline const_elem_ref operator[](unsigned int i) const {
			return m_a[(m_remapReads)? m_begin[i] : i];
		}

		Adaptor m_a;
		const unsigned int* m_begin;
		bool m_remapReads;
		unsigned int m_size;
	};


	template<typename T>
	struct is_points_indexer : public boost::mpl::false_{};

	template<typename T>
	struct is_points_indexer<points_indexer<T> > : public boost::mpl::true_{};


	template<typename Points>
	struct points_adaptor<points_indexer<Points> >
	: public detail::custom_points_adaptor<points_indexer<Points> >
	{
		typedef points_indexer<Points> T;
		points_adaptor(const T& t):detail::custom_points_adaptor<T>(t){}
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
