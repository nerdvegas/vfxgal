#ifndef _DGAL_ADDPOLYEDGES__H_
#define _DGAL_ADDPOLYEDGES__H_

#include "../adaptors/points.hpp"
#include <boost/unordered_set.hpp>
#include <map>

namespace dgal {

	/*
	 * @brief addPolyEdges
	 * Split a poly with the given edges, creating a set of sub-polys.
	 * @param points The polygon to split.
	 * @param new_edges The edges that split the poly. Edges index into original points,
	 * not poly vertices (although if the poly is not indexed, these are identical).
	 * @param[out] new_polys The generated polygons. Note that this is appended to.
	 * @note The poly's physical vertex positions are not used - this is a purely
	 * topological algorithm.
	 */
	template<typename Points, typename Index>
	void addPolyEdges(const Points& points,
		const std::vector<std::pair<Index,Index> >& new_edges,
		std::vector<std::vector<Index> >& new_polys);

	namespace detail {

		template<typename Index>
		bool getShortestEdgeLoop(const std::multimap<Index,Index>& edges,
			std::vector<Index>& loop);

	}

///////////////////////// impl

template<typename Points, typename Index>
void addPolyEdges(const Points& points,
	const std::vector<std::pair<Index,Index> >& new_edges,
	std::vector<std::vector<Index> >& new_polys)
{
	typedef points_adaptor<Points> 			Adaptor;
	typedef std::pair<Index,Index>			edge_type;
	typedef std::vector<Index>				index_vector;
	typedef std::multimap<Index,Index> 		edge_multimap;

	Adaptor a(points);
	unsigned int nverts = a.size();
	edge_multimap edges;

	// add poly edges to edge soup
	for(unsigned int i=0; i<nverts; ++i)
	{
		unsigned int j = (i+1)%nverts;
		edges.insert(typename edge_multimap::value_type(a.index(i), a.index(j)));
	}

	// add new edges to edge soup, each new edge is actually an equal-but-opposite pair.
	// Don't bother for tris, no edges can be added
	if(nverts > 3)
	{
		for(typename std::vector<edge_type>::const_iterator it=new_edges.begin();
			it!=new_edges.end(); ++it)
		{
			edges.insert(typename edge_multimap::value_type(it->first, it->second));
			edges.insert(typename edge_multimap::value_type(it->second, it->first));
		}
	}

	// extract shortest loop until nothing left
	index_vector loop;
	while(!edges.empty())
	{
		loop.resize(1);
		loop[0] = edges.begin()->first;
		bool b = detail::getShortestEdgeLoop(edges, loop);
		assert(b);

		// remove loop from edge soup
		for(unsigned int i=0; i<loop.size(); ++i)
		{
			unsigned int j = (i+1)%loop.size();
			typename edge_multimap::iterator it = edges.find(loop[i]);
			while(it->second != loop[j])
				++it;
			edges.erase(it);
		}

		// add to output
		new_polys.push_back(loop);
	}
}


template<typename Index>
bool detail::getShortestEdgeLoop(const std::multimap<Index,Index>& edges,
	std::vector<Index>& loop)
{
	typedef std::vector<Index>				index_vector;
	typedef std::multimap<Index,Index> 		edge_multimap;

	assert(!loop.empty());
	boost::unordered_set<Index> visited_pts(loop.begin(), loop.end());

	while(1)
	{
		typename edge_multimap::const_iterator it = edges.find(loop.back());
		if(it == edges.end())
			return false;

		typename edge_multimap::const_iterator it2 = --edges.upper_bound(loop.back());
		if(it == it2)
		{
			if(it->second == loop.front())
				return true; // found loop

			if(visited_pts.find(it->second) != visited_pts.end())
				return false; // can't make a loop

			loop.push_back(it->second);
		}
		else
		{
			// traverse each edge, and choose the shortest loop
			index_vector shortest_loop;

			for(++it2; it!=it2; ++it)
			{
				// avoid doubling directly back onto the equal-but-opposite edge
				if((loop.size() > 1) && (it->second == loop[loop.size()-2]))
					continue;

				if(it->second == loop.front())
				{
					// this edge closes the loop, so we're already at the shortest loop
					return true;
				}

				if(visited_pts.find(it->second) != visited_pts.end())
					continue; // can't make a loop

				index_vector loop2 = loop;
				loop2.push_back(it->second);
				if(!getShortestEdgeLoop(edges, loop2))
					continue; // can't make a loop

				if(shortest_loop.empty() || (loop2.size() < shortest_loop.size()))
					shortest_loop = loop2;
			}

			if(shortest_loop.empty())
				return false;

			loop = shortest_loop;
			return true;
		}
	}
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
