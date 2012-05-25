#ifndef _DGAL_ADAPTORS_POINTS_REPLACER__H_
#define _DGAL_ADAPTORS_POINTS_REPLACER__H_

#include "points.hpp"
#include <boost/mpl/bool.hpp>


namespace vfxgal {

	/*
	 * @class points_replacer
	 * @brief
	 * This class and the following points_adaptor specialization allow us to override an
	 * existing points_adaptor so that it uses a different set of points, but the same
	 * indexing information.
	 */
	template<typename Points, typename ReplPts>
	struct points_replacer
	{
		typedef points_adaptor<Points> 					Adaptor;
		typedef points_adaptor<ReplPts> 				ReplAdaptor;
		typedef typename ReplAdaptor::scalar			scalar;
		typedef typename ReplAdaptor::elem_type			elem_type;
		typedef typename ReplAdaptor::const_elem_ref	const_elem_ref;

		points_replacer(const Points& points, const ReplPts& replPts, bool remap_reads = true)
		: m_a(points), m_replA(replPts), m_remapReads(remap_reads){}

		inline unsigned int size() const 				{ return m_a.size(); }
		inline unsigned int index(unsigned int i) const { return m_a.index(i); }
		inline bool _is_indexed() const 				{ return m_a.is_indexed(); }

		inline const_elem_ref operator[](unsigned int i) const {
			return m_replA[(m_remapReads)? m_a.index(i) : i];
		}

		Adaptor m_a;
		ReplAdaptor m_replA;
		bool m_remapReads;
	};


	template<typename Points, typename ReplPts>
	struct points_adaptor<points_replacer<Points,ReplPts> >
	: public detail::custom_points_adaptor<points_replacer<Points,ReplPts> >
	{
		typedef points_replacer<Points,ReplPts> T;
		points_adaptor(const T& t):detail::custom_points_adaptor<T>(t){}
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
