#ifndef _DGAL_EDGE__H_
#define _DGAL_EDGE__H_

#include <utility>
#include <boost/functional/hash.hpp>


namespace vfxgal {

	/*
	 * @class edge
	 * @brief A directional edge.
	 */
	template<typename T>
	struct edge : public std::pair<T,T>
	{
		edge(){}

		edge(const T& a, const T& b)
		: std::pair<T,T>(a,b){}

		inline edge reversed() const {
			return edge(this->second, this->first);
		}
	};


	/*
	 * @class bidirectional_edge
	 * @brief A bidirectional edge is a connection between two values with no direction
	 * information. Note that bidirectional_edge(A,B) == bidirectional_edge(B,A).
	 */
	template<typename T>
	class bidirectional_edge
	{
	public:
		bidirectional_edge(){}

		bidirectional_edge(const T& a, const T& b) { set(a,b); }

		inline void set(const T& a, const T& b) {
			m_pair.first = std::min(a,b);
			m_pair.second = std::max(a,b);
		}

		bool operator<(const bidirectional_edge& rhs) const {
			return (m_pair < rhs.m_pair);
		}

		bool operator==(const bidirectional_edge& rhs) const {
			return (m_pair == rhs.m_pair);
		}

		bool operator!=(const bidirectional_edge& rhs) const {
			return (m_pair != rhs.m_pair);
		}

		inline const T& first() const 		{ return m_pair.first; }
		inline const T& second() const 		{ return m_pair.second; }

	protected:
		std::pair<T,T> m_pair;
	};


	// boost hash support
	template<typename T>
	std::size_t hash_value(const bidirectional_edge<T>& e)
	{
        std::size_t seed = 0;
        boost::hash_combine(seed, e.first());
        boost::hash_combine(seed, e.second());
        return seed;
	}


	typedef edge<unsigned int>					u_edge;
	typedef bidirectional_edge<unsigned int> 	u_bi_edge;

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
