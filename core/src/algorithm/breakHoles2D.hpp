#ifndef DGAL_ALGO_BREAKHOLES__H_
#define DGAL_ALGO_BREAKHOLES__H_

#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include "../poly2D.hpp"


namespace dgal {

	/*
	 * @brief break_holes
	 * Given a set of polys (clockwise vertex loops) and holes (anticlockwise vertex
	 * loops), calculate a set of polys which remove the holes by linking them to their
	 * enclosing poly via a degenerate edge.
	 * @param polys The clockwise vertex loops.
	 * @param holes The anticlockwise vertex loops (holes)
	 * @param[out] result The resulting polys. Note that this is appended to. Each index
	 * in a returned loop points into the original loops (imagine 'polys' and 'holes' have
	 * been flattened into a single list).
	 */
	template<typename T>
	void breakHoles2D(std::vector<std::vector<Imath::Vec2<T> > >& polys,
		std::vector<std::vector<Imath::Vec2<T> > >& holes,
		std::vector<std::vector<unsigned int> >& result);


	namespace detail {

		template<typename T>
		struct poly_or_hole_2d
		{
			std::vector<unsigned int> m_verts;
			Imath::Box<Imath::Vec2<T> > m_bounds;
		};

	}


///////////////////////// impl

template<typename T>
void breakHoles2D(
	std::vector<std::vector<Imath::Vec2<T> > >& polys_,
	std::vector<std::vector<Imath::Vec2<T> > >& holes_,
	std::vector<std::vector<unsigned int> >& result)
{
	typedef Imath::Vec2<T>							vec_type;
	typedef Imath::Box<vec_type>					box_type;
	typedef std::vector<vec_type>					vec_loop;
	typedef LineSegment<vec_type>					lineseg_type;
	typedef std::vector<unsigned int>				uint_loop;
	typedef detail::poly_or_hole_2d<T>				loop;
	typedef std::multimap<T, loop>					loop_map;
	typedef typename loop_map::iterator				loop_map_it;
	typedef std::list<loop*>						hole_list;
	typedef boost::unordered_set<loop*>				poly_set;
	typedef typename hole_list::iterator			hole_list_it;
	typedef boost::unordered_map<loop*, hole_list>	hole_map;

	if(polys_.empty() && holes_.empty())
		return;

	// create loops, sorted by area
	std::vector<vec_type> points;
	loop_map polys, holes;
	uint_loop mergedHole;
	std::vector<loop*> raytest_loops;

	for(unsigned int i=0; i<2; ++i)
	{
		const std::vector<vec_loop>& loopsSrc = (i)? holes_ : polys_;
		loop_map& loopsDest = (i)? holes : polys;

		for(unsigned int j=0; j<loopsSrc.size(); ++j)
		{
			const vec_loop& loopSrc = loopsSrc[j];
			T area = std::abs(signedArea(loopSrc));

			loop& loopDest = loopsDest.insert(
				typename loop_map::value_type(area, loop()))->second;

			unsigned int nverts = loopSrc.size();
			loopDest.m_verts.resize(nverts);
			for(unsigned int k=0; k<nverts; ++k)
			{
				loopDest.m_verts[k] = points.size();
				points.push_back(loopSrc[k]);
			}

			get_bounds(loopSrc, loopDest.m_bounds);
		}
	}

	// for each hole, find the parent poly. The parent poly will be the smallest-area
	// poly that encloses the hole
	hole_map polyHoles;
	poly_set polys_with_holes;

	for(loop_map_it itHole=holes.begin(); itHole!=holes.end(); ++itHole)
	{
		T holeArea = itHole->first;
		loop& hole = itHole->second;

		loop_map_it itPoly = polys.begin();
		for(; itPoly!=polys.end(); ++itPoly)
		{
			if(	(itPoly->first >= holeArea) &&
				(itPoly->second.m_bounds.intersects(hole.m_bounds)) )
			{
				// just check for at least one hole vert in the poly
				uint_loop& polyVerts = itPoly->second.m_verts;

				points_indexer<std::vector<vec_type> > ipoly(
					&polyVerts[0], &polyVerts[0]+polyVerts.size(), points);

				uint_loop::iterator it = hole.m_verts.begin();
				for(; it!=hole.m_verts.end(); ++it)
				{
					assert(*it < points.size());
					if(isInside(ipoly, points[*it]))
						break;
				}

				if(it != hole.m_verts.end())
				{
					loop* enclosing_poly = &(itPoly->second);
					polyHoles[enclosing_poly].push_back(&hole);
					polys_with_holes.insert(enclosing_poly);
					break;
				}
			}
		}

		if(itPoly == polys.end())
		{
			// couldn't find parent poly for this hole. Not much we can do now... let's
			// just flip the hole and output it
			std::reverse(hole.m_verts.begin(), hole.m_verts.end());
			result.push_back(hole.m_verts);
		}
	}

	// output polys without holes in them
	for(loop_map_it it=polys.begin(); it!=polys.end(); ++it)
	{
		loop* ppoly = &(it->second);
		if(polys_with_holes.find(ppoly) == polys_with_holes.end())
			result.push_back(ppoly->m_verts);
	}

	// for each (poly, holes) pair, merge the holes into the poly by introducing a
	// degenerate edge between each hole and the poly
	for(typename hole_map::iterator it=polyHoles.begin(); it!=polyHoles.end(); ++it)
	{
		loop& parent_poly = *(it->first);
		hole_list& child_holes = it->second;
		uint_loop& polyVerts = parent_poly.m_verts;
		unsigned int currPolyVert = 0;
		boost::unordered_set<unsigned int> bannedVerts;

		while(!child_holes.empty())
		{
			bool found_ray = false;

			vec_type ray_box_padding(std::numeric_limits<T>::epsilon());
			unsigned int nPolyVerts = polyVerts.size();
			unsigned int stagger = nPolyVerts / 2;

			for(unsigned int i=0; (i<nPolyVerts) && !found_ray; ++i)
			{
				// do a staggered iteration over poly verts, should mean we find a
				// directly visible hole sooner on average
				unsigned int polyVertIndex = ((i%2)==0)? (i/2)+stagger : (i/2);
				polyVertIndex += currPolyVert;
				polyVertIndex %= nPolyVerts;

				unsigned int polyVert = polyVerts[polyVertIndex];
				const vec_type& poly_pt = points[polyVert];

				if(bannedVerts.find(polyVert) != bannedVerts.end())
					continue;

				// sort holes wrt distance - closer = better chance of clear connection
				typedef std::multimap<T, hole_list_it> hole_dist_map;
				hole_dist_map holemap;

				for(hole_list_it it2=child_holes.begin(); it2!=child_holes.end(); ++it2)
				{
					T dist = (Imath::closestPointInBox(poly_pt, (*it2)->m_bounds) - poly_pt).length2();
					holemap.insert(typename hole_dist_map::value_type(dist, it2));
				}

				// iterate over holes
				for(typename hole_dist_map::iterator it2=holemap.begin();
					(it2!=holemap.end()) && !found_ray; ++it2)
				{
					loop& hole = *(*(it2->second));
					uint_loop& holeVerts = hole.m_verts;
					unsigned int nHoleVerts = holeVerts.size();
					unsigned int stagger2 = nHoleVerts / 2;

					// iterate over the verts of this hole. Jump back and forth in a staggered
					// fashion, this will get us to a vert on the facing side of the hole sooner
					for(unsigned int j=0; j<nHoleVerts; ++j)
					{
						unsigned int holeVertIndex = ((j%2)==0)? (j/2)+stagger2 : (j/2);
						assert(holeVertIndex < nHoleVerts);
						unsigned int holeVert = holeVerts[holeVertIndex];
						const vec_type& hole_pt = points[holeVert];

						if(bannedVerts.find(holeVert) != bannedVerts.end())
							continue;

						// we have a contender ray
						lineseg_type ray(poly_pt, hole_pt);
						box_type ray_box;
						ray_box.extendBy(poly_pt);
						ray_box.extendBy(hole_pt);
						ray_box.min -= ray_box_padding;
						ray_box.max += ray_box_padding;

						// test if any edge blocks the ray or any point is too close.
						// Search sorted holes here too, since closer = better chance
						// of blocking ray. Search the poly itself last.
						bool ray_blocked = false;
						raytest_loops.clear();

						for(typename hole_dist_map::iterator it3=holemap.begin();
							it3!=holemap.end(); ++it3)
						{
							raytest_loops.push_back(*(it3->second));
						}
						raytest_loops.push_back(&parent_poly);

						for(typename std::vector<loop*>::iterator it3=raytest_loops.begin();
							(it3!=raytest_loops.end()) && !ray_blocked; ++it3)
						{
							loop& testloop = *(*it3);
							if(ray_box.intersects(testloop.m_bounds))
							{
								// test each edge to see if it blocks the ray
								uint_loop& testverts = testloop.m_verts;
								unsigned int ntestverts = testverts.size();

								for(unsigned int k=0; k<ntestverts; ++k)
								{
									unsigned int vert1 = testverts[k];
									unsigned int vert2 = testverts[(k+1)%ntestverts];

									if((vert1 == polyVert) || (vert1 == holeVert) ||
										(vert2 == polyVert) || (vert2 == holeVert))
										continue;

									const vec_type& testPt1 = points[vert1];
									const vec_type& testPt2 = points[vert2];
									lineseg_type edge(testPt1, testPt2);

									if(intersects(ray, edge))
									{
										ray_blocked = true;
										break;
									}
								}
							}
						}

						if(!ray_blocked)
						{
							found_ray = true;

							// merge the hole with the poly by linking them with a degenerate edge
							mergedHole.resize(nHoleVerts+2);

							for(unsigned int k=0; k<nHoleVerts; ++k)
								mergedHole[k] = holeVerts[(holeVertIndex+k) % nHoleVerts];

							mergedHole[nHoleVerts] = mergedHole[0];
							mergedHole[nHoleVerts+1] = polyVert;

							uint_loop::iterator it3 = polyVerts.begin() + polyVertIndex;
							polyVerts.insert(++it3, mergedHole.begin(), mergedHole.end());

							child_holes.erase(it2->second);
							bannedVerts.insert(holeVert);
							bannedVerts.insert(polyVert);

							break;
						}
					}
				}
			}

			if(!found_ray)
			{
				// we couldn't find a clear ray to connect to any hole at all. Give up,
				// and just delete the last hole.
				child_holes.pop_back();
			}

			// move to a new poly vert to try and find another hole connection. 27/80
			// seems to give a pretty even distribution around the poly. This is to avoid
			// multiple rays seeding from the same area on a poly border.
			currPolyVert = (currPolyVert + 1 + polyVerts.size()*27/80) % polyVerts.size();
		}

		// output the merged poly+holes
		result.push_back(parent_poly.m_verts);
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
