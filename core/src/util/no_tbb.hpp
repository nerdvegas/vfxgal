#ifndef _DGAL_UTILS_NO_TBB__H_
#define _DGAL_UTILS_NO_TBB__H_

#ifdef DGAL_NO_TBB

#include <vector>


/*
 * @brief This file mimicks the tbb API but doesn't use tbb. This allows us to turn
 * tbb on/off with a compile-time define.
 */

namespace vfxgal { namespace no_tbb {

	// blocked_range
	template<typename T>
	struct blocked_range
	{
		blocked_range(T begin, T end):m_begin(begin),m_end(end){}
		T begin() const { return m_begin; }
		T end() const { return m_end; }

		T m_begin, m_end;
	};


	// enumerable_thread_specific
	template<typename T>
	struct enumerable_thread_specific
	{
		typedef T& 									reference;
		typedef typename std::vector<T>::iterator	iterator;

		enumerable_thread_specific():m_data(1){}
		T& local() const { return const_cast<T&>(m_data[0]); }
		iterator begin() { return m_data.begin(); }
		iterator end() { return m_data.end(); }

		std::vector<T> m_data;
	};


	// parallel_for
	template<typename Range, typename Body>
	void parallel_for(const Range& r, const Body& b) { b(r); }

} }

#endif

#endif
