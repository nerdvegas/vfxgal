#ifndef _DGAL_ALGO_CLIPMESH3D__H_
#define _DGAL_ALGO_CLIPMESH3D__H_

#include <algorithm>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include <boost/utility.hpp>
#include "../adaptors/mesh_points.hpp"
#include "../adaptors/octree.hpp"
#include "../ClippingContext.hpp"
#include "../edge.hpp"
#include "../mesh.hpp"
#include "../simple_mesh.hpp"
#include "clipPoly3D.hpp"
#include "breakHoles2D.hpp"
#include "cleanMesh.hpp"


/*
 * Note that in the following functions, if 'closed' clipping is enabled then the mesh is
 * assumed to be manifold, and each faces' points coplanar. If this requirement is not met
 * then clipping errors can occur.
 */

namespace vfxgal {

	/*
	 * @brief clipMesh3D
	 * Clip a mesh against a cutting plane.
	 * @param mesh The input mesh.
	 * @param plane The cutting plane. Those polys in the positive halfspace will be kept.
	 * @param closed If true, new faces created by the cutting plane will be added to the result.
	 * @param break_holes If true, holes inside polygons will be removed by linking holes
	 * to their surrounding polygon via a degenerate edge (sometimes called 'keyholing').
	 * @param clip_result[out] The clip result. Any polys which have been clipped, and any
	 * new polys which have been generated, will be appended.
	 * @param inside_result[out] Indices of polys which are unclipped and entirely inside
	 * the cutting plane. Note that this vector is appended. Also, if the mesh is entirely
	 * inside the cutting plane, nothing is appended.
	 * @returns The intersection type. clip_result, inside_result will only be appended if
	 * INTERSECT_INTERSECTS is returned.
	 */
	template<typename Mesh3D>
	IntersectType clipMesh3D(const Mesh3D& mesh,
		const Imath::Plane3<typename mesh_adaptor<Mesh3D>::scalar>& plane,
		bool closed, bool break_holes,
		std::list<ClippedPoly<typename mesh_adaptor<Mesh3D>::scalar> >& clip_result,
		std::vector<unsigned int>& inside_result,
		detail::ClippingContext<typename mesh_adaptor<Mesh3D>::scalar>* ctxt = NULL);

	/*
	 * @brief clipMesh3D
	 * Clip a mesh against an ordered set of cutting planes.
	 * @param mesh The input mesh.
	 * @param planesBegin, planesEnd The cutting planes. Those polys in the positive
	 * halfspace will be kept.
	 * @param closed If true, new faces created by the cutting plane will be added to the result.
	 * @param break_holes If true, holes inside polygons will be removed by linking holes
	 * to their surrounding polygon via a degenerate edge.
	 * @param[out] result If a mesh is produced, it will be appended to this list. Only zero
	 * or one meshes can be produced.
	 * @param[out] pointIDs See remapMesh.MeshRemapSettings.pointMapping.
	 * @param[out] polyIDs See remapMesh.MeshRemapSettings.polyMapping.
	 * @param minEdgeLength Edges shorter than this length will be collapsed into points, as
	 * part of the clipping phase which removes degenerate features. If < 0, the function
	 * will calculate a reasonable epsilon for you.
	 * @returns The intersection type. 'result', will only be appended if INTERSECT_INTERSECTS
	 * is returned. Note that if there are no planes, INTERSECT_INSIDE is returned.
	 */
	template<typename Mesh3D, typename ConstPlaneIterator>
	IntersectType clipMesh3D(const Mesh3D& mesh,
		ConstPlaneIterator planesBegin, ConstPlaneIterator planesEnd,
		bool closed, bool break_holes,
		simple_mesh<typename mesh_adaptor<Mesh3D>::point_type>& result,
		std::vector<int>* pointIDs = NULL,
		std::vector<int>* polyIDs = NULL,
		typename mesh_adaptor<Mesh3D>::scalar minEdgeLength = -1,
		detail::ClippingContext<typename mesh_adaptor<Mesh3D>::scalar>* ctxt = NULL);


	namespace detail {

		// adaptor class so we can share a bunch of code between octree/non-octree clipping
		template<typename T>
		struct poly_iterator
		{
			poly_iterator(T npolys): m_npolys(npolys), m_currPoly(0){}
			bool next(T& index)
			{
				if(m_currPoly < m_npolys)
				{
					index = m_currPoly++;
					return true;
				}
				else
					return false;
			}

			void reset() { m_currPoly=0; }

			T m_npolys;
			T m_currPoly;
		};

		template<typename T>
		struct poly_iterator<boost::unordered_set<T> >
		{
			poly_iterator(const boost::unordered_set<T>& s):m_s(s), m_it(m_s.begin()){}
			bool next(T& index)
			{
				if(m_it == m_s.end())
					return false;
				else
				{
					index = *(m_it++);
					return true;
				}
			}

			void reset() { m_it=m_s.begin(); }

			const boost::unordered_set<T>& m_s;
			typename boost::unordered_set<T>::const_iterator m_it;
		};

		template<typename Points>
		bool markCtxtPointsAsInside(
			ClippingContext<typename points_adaptor<Points>::scalar>& ctxt,
			const Points& points);

		// clip a non-octree mesh against a plane
		template<typename Mesh3D>
		typename boost::disable_if<is_octree<Mesh3D>, IntersectType>::type
		/*IntersectType*/ clipMesh3D_(const Mesh3D& mesh,
			const Imath::Plane3<typename mesh_adaptor<Mesh3D>::scalar>& plane,
			bool closed, bool break_holes,
			std::list<ClippedPoly<typename mesh_adaptor<Mesh3D>::scalar> >& clip_result,
			std::vector<unsigned int>& inside_result,
			ClippingContext<typename mesh_adaptor<Mesh3D>::scalar>* ctxt);

		// clip an octree mesh against a plane
		template<typename Mesh3D>
		typename boost::enable_if<is_octree<Mesh3D>, IntersectType>::type
		/*IntersectType*/ clipMesh3D_(const Mesh3D& mesh,
			const Imath::Plane3<typename mesh_adaptor<Mesh3D>::scalar>& plane,
			bool closed, bool break_holes,
			std::list<ClippedPoly<typename mesh_adaptor<Mesh3D>::scalar> >& clip_result,
			std::vector<unsigned int>& inside_result,
			ClippingContext<typename mesh_adaptor<Mesh3D>::scalar>* ctxt);

		// code common to octree/non-octree plane clipping funcs above
		template<typename Mesh3D, typename PolyIterator>
		void clipMesh3D_impl(const Mesh3D& mesh, PolyIterator& poly_it,
			const Imath::Plane3<typename mesh_adaptor<Mesh3D>::scalar>& plane,
			bool closed, bool break_holes,
			std::list<ClippedPoly<typename mesh_adaptor<Mesh3D>::scalar> >& clip_result,
			std::vector<unsigned int>& inside_result,
			ClippingContext<typename mesh_adaptor<Mesh3D>::scalar>& ctxt);

		// clip a mesh against a plane and write the result to a simple_mesh. Note that for
		// performance reasons, points are simply added to the original mesh, leaving floating
		// points included. It's for this reason that point remapping args are not used.
		template<typename Mesh3D>
		IntersectType clipMesh3D_to_simple(const Mesh3D& mesh,
			const std::vector<int>* polyIDs,
			const Imath::Plane3<typename mesh_adaptor<Mesh3D>::scalar>& plane,
			bool closed, bool break_holes,
			simple_mesh<typename mesh_adaptor<Mesh3D>::point_type>& result,
			std::vector<int>* resultPolyIDs,
			ClippingContext<typename mesh_adaptor<Mesh3D>::scalar>* ctxt);

	}


///////////////////////// impl

template<typename Points>
bool detail::markCtxtPointsAsInside(
	ClippingContext<typename points_adaptor<Points>::scalar>& ctxt,
	const Points& points)
{
	typedef points_adaptor<Points> Adaptor;
	Adaptor a(points);

	bool any_changed = false;
	for(unsigned int i=0; i<a.size(); ++i)
	{
		unsigned int index = a.index(i);
		assert(index < ctxt.m_isPtInside.size());
		any_changed |= !ctxt.m_isPtInside[index];
		ctxt.m_isPtInside[index] = true;
	}

	return any_changed;
}


template<typename Mesh3D>
IntersectType clipMesh3D(const Mesh3D& mesh,
	const Imath::Plane3<typename mesh_adaptor<Mesh3D>::scalar>& plane,
	bool closed, bool break_holes,
	std::list<ClippedPoly<typename mesh_adaptor<Mesh3D>::scalar> >& clip_result,
	std::vector<unsigned int>& inside_result,
	detail::ClippingContext<typename mesh_adaptor<Mesh3D>::scalar>* ctxt)
{
	return detail::clipMesh3D_(mesh, plane, closed, break_holes,
		clip_result, inside_result, ctxt);
}


// specialisation for non-octree clipping
template<typename Mesh3D>
typename boost::disable_if<is_octree<Mesh3D>, IntersectType>::type
/*IntersectType*/ detail::clipMesh3D_(const Mesh3D& mesh,
	const Imath::Plane3<typename mesh_adaptor<Mesh3D>::scalar>& plane,
	bool closed, bool break_holes,
	std::list<ClippedPoly<typename mesh_adaptor<Mesh3D>::scalar> >& clip_result,
	std::vector<unsigned int>& inside_result,
	ClippingContext<typename mesh_adaptor<Mesh3D>::scalar>* ctxt_)
{
	typedef mesh_adaptor<Mesh3D> 					Adaptor;
	typedef typename Adaptor::scalar				T;
	typedef Imath::Vec2<T>							vec2_type;
	typedef Imath::Vec3<T>							vec3_type;
	typedef Imath::Plane3<T>						plane3_type;
	typedef typename Adaptor::const_point_ref		const_point_ref;
	typedef typename Adaptor::const_poly_ref		const_poly_ref;
	typedef ClippedPoly<T> 							polyclip_type;
	typedef detail::ClippingContext<T> 				ctxt_type;
	typedef typename ctxt_type::poly_data			ctxt_poly;
	typedef typename polyclip_type::intersection	polyclip_intersection;

	Adaptor a(mesh);
	unsigned int npoints = a.numPoints();
	unsigned int npolys = a.numPolys();

	ctxt_type ctxt_local;
	ctxt_type& ctxt = (ctxt_)? *ctxt_ : ctxt_local;

	// do inside/outside test on all points. Trivially reject mesh if all inside/outside
	{
		bool all_inside = true;
		bool all_outside = true;
		ctxt.m_isPtInside.resize(npoints);

		for(unsigned int i=0; i<npoints; ++i)
		{
			bool inside = isInside(plane, a.getPoint(i));
			ctxt.m_isPtInside[i] = inside;
			all_inside &= inside;
			all_outside &= !inside;
		}

		if(all_outside)
			return INTERSECT_OUTSIDE;
		else if(all_inside)
			return INTERSECT_INSIDE;
	}

	// do all of the things
	ctxt.m_polyData.resize(npolys);
	detail::poly_iterator<unsigned int> poly_it(npolys);
	detail::clipMesh3D_impl(mesh, poly_it, plane, closed, break_holes,
		clip_result, inside_result, ctxt);

	return INTERSECT_INTERSECTS;
}


// specialisation for octree clipping
template<typename Mesh3D>
typename boost::enable_if<is_octree<Mesh3D>, IntersectType>::type
/*IntersectType*/ detail::clipMesh3D_(const Mesh3D& mesh,
	const Imath::Plane3<typename mesh_adaptor<Mesh3D>::scalar>& plane,
	bool closed, bool break_holes,
	std::list<ClippedPoly<typename mesh_adaptor<Mesh3D>::scalar> >& clip_result,
	std::vector<unsigned int>& inside_result,
	ClippingContext<typename mesh_adaptor<Mesh3D>::scalar>* ctxt_)
{
	typedef Mesh3D									octree_type;
	typedef typename octree_type::dataset_type		dataset_type;
	typedef typename dataset_type::mesh_type		mesh_type;
	typedef mesh_adaptor<mesh_type>					Adaptor;
	typedef typename Adaptor::scalar				T;
	typedef Imath::Plane3<T>						plane_type;
	typedef typename Adaptor::poly_type				poly_type;
	typedef points_adaptor<poly_type>				PointsAdaptor;
	typedef detail::ClippingContext<T> 				ctxt_type;
	typedef boost::unordered_set<unsigned int>		uint_set;

	ctxt_type ctxt_local;
	ctxt_type& ctxt = (ctxt_)? *ctxt_ : ctxt_local;

	Adaptor a(mesh.datasets()[0]->mesh());
	unsigned int npoints = a.numPoints();
	unsigned int npolys = a.numPolys();

	// do octree-plane intersection test. Note that the plane is given a thickness, this
	// is to allow polys which are both parallel and straddling the plane to possibly be
	// moved inside by their neighbour polygons.
	std::vector<uint_set> insidePolys_(1);
	std::vector<uint_set> intersectingPolys_(1);
	mesh.query(plane, NULL, &insidePolys_, &intersectingPolys_,
		std::numeric_limits<T>::epsilon()*10);

	const uint_set& insidePolys = insidePolys_[0];
	const uint_set& intersectingPolys = intersectingPolys_[0];

	if(intersectingPolys.empty())
		return insidePolys.empty()? INTERSECT_OUTSIDE : INTERSECT_INSIDE;

	// do inside/outside test on all points. Trivially reject mesh if all inside/outside
	{
		uint_set visited_pts;
		bool all_inside = true;
		bool all_outside = true;
		ctxt.m_isPtInside.resize(npoints);

		for(uint_set::const_iterator it=intersectingPolys.begin(); it!=intersectingPolys.end(); ++it)
		{
			PointsAdaptor pa(a.getPoly(*it));
			for(unsigned int i=0; i<pa.size(); ++i)
			{
				unsigned int ptnum = pa.index(i);
				if(visited_pts.find(ptnum) == visited_pts.end())
				{
					bool inside = isInside(plane, pa[i]);
					ctxt.m_isPtInside[ptnum] = inside;
					all_inside &= inside;
					all_outside &= !inside;
					visited_pts.insert(ptnum);
				}
			}
		}

		if(all_outside)
		{
			assert(insidePolys.empty());
			return INTERSECT_OUTSIDE;
		}
		else if(all_inside)
			return INTERSECT_INSIDE;
	}

	// output inside polys
	{
		unsigned int sz = inside_result.size();
		inside_result.resize(sz + insidePolys.size());
		std::copy(insidePolys.begin(), insidePolys.end(), inside_result.begin()+sz);
	}

	// do all of the things
	ctxt.m_polyData.resize(npolys);
	detail::poly_iterator<uint_set> poly_it(intersectingPolys);
	detail::clipMesh3D_impl(mesh, poly_it, plane, closed, break_holes,
		clip_result, inside_result, ctxt);

	return INTERSECT_INTERSECTS;
}


// code shared by both octree- and non-octree- clipMesh3D functions
template<typename Mesh3D, typename PolyIterator>
void detail::clipMesh3D_impl(const Mesh3D& mesh, PolyIterator& poly_it,
	const Imath::Plane3<typename mesh_adaptor<Mesh3D>::scalar>& plane,
	bool closed, bool break_holes,
	std::list<ClippedPoly<typename mesh_adaptor<Mesh3D>::scalar> >& clip_result,
	std::vector<unsigned int>& inside_result,
	ClippingContext<typename mesh_adaptor<Mesh3D>::scalar>& ctxt)
{
	typedef mesh_adaptor<Mesh3D> 					Adaptor;
	typedef typename Adaptor::scalar				T;
	typedef Imath::Vec2<T>							vec2_type;
	typedef Imath::Vec3<T>							vec3_type;
	typedef Imath::Plane3<T>						plane3_type;
	typedef typename Adaptor::const_point_ref		const_point_ref;
	typedef typename Adaptor::const_poly_ref		const_poly_ref;
	typedef ClippedPoly<T> 							polyclip_type;
	typedef detail::ClippingContext<T> 				ctxt_type;
	typedef typename ctxt_type::poly_data			ctxt_poly;
	typedef typename polyclip_type::intersection	polyclip_intersection;

	Adaptor a(mesh);
	mesh_points<Mesh3D> meshpoints(mesh);
	plane3_type r_plane = reverse(plane);

	// calc every poly's plane and intersection type wrt cutting plane. Detect special
	// cases where the poly will need to be forced either all-in or all-out
	poly_it.reset();
	unsigned int i;
	while(poly_it.next(i))
	{
		const_poly_ref poly = a.getPoly(i);
		ctxt_poly& data = ctxt.m_polyData[i];

		data.m_intersectType = detail::clip_test(ctxt, poly);
		if(data.m_intersectType != INTERSECT_INSIDE)
		{
			vec3_type v;
			calc_newell_normal(poly, v, false);

			if(v.length() < std::numeric_limits<T>::epsilon())
			{
				// zero-area poly - we can still clip if it's a non-parallel sliver
				get_longest_edge_vector(poly, v);
				if(v.length() < std::numeric_limits<T>::epsilon())
				{
					// it's not a sliver, just very small
					data.m_cannotIntersectPlane = true;
				}
				else
				{
					// it's a sliver - is it parallel to the cutting plane?
					T vdot = v.normalized().dot(plane.normal);
					if(std::abs(vdot) < std::numeric_limits<T>::epsilon())
					{
						// yes it's parallel
						data.m_cannotIntersectPlane = true;
					}
					else
					{
						// it's a non-parallel sliver, we can still clip this
						data.m_cannotIntersectPlane = false;
					}
				}
			}
			else
			{
				// non-zero-area poly - don't intersect if parallel with plane. The
				// squaring of normcross is important, this is to guarantee that if this
				// poly is submitted for a plane-clip, the clip will succeed.
				T normcross = v.normalized().cross(plane.normal).length();
				data.m_cannotIntersectPlane =
					(normcross*normcross < std::numeric_limits<T>::epsilon()*2);
			}
		}
	}

	// detect polys which intersect plane and are parallel/zero area, and move all points
	// inside. This is done iteratively since an intersecting poly can drag its
	// neighbour into the same case.
	{
		bool any_pts_changed = true;
		while(any_pts_changed)
		{
			any_pts_changed = false;

			poly_it.reset();
			unsigned int i;
			while(poly_it.next(i))
			{
				ctxt_poly& data = ctxt.m_polyData[i];
				if(data.m_intersectType != INTERSECT_INSIDE)
				{
					const_poly_ref poly = a.getPoly(i);
					data.m_intersectType = detail::clip_test(ctxt, poly);

					if(data.m_cannotIntersectPlane && (data.m_intersectType == INTERSECT_INTERSECTS))
					{
						detail::markCtxtPointsAsInside(ctxt, poly);
						data.m_intersectType = INTERSECT_INSIDE;
						any_pts_changed = true;
					}
				}
			}
		}
	}

	// clip each poly in the mesh
	unsigned int nclips_old = clip_result.size();
	typename std::list<polyclip_type>::const_iterator itClip = clip_result.end();
	if(nclips_old > 0)
		--itClip;

	poly_it.reset();
	while(poly_it.next(i))
	{
		const_poly_ref poly = a.getPoly(i);
		const ctxt_poly& data = ctxt.m_polyData[i];

		if(data.m_intersectType == INTERSECT_INSIDE)
			inside_result.push_back(i);
		else if(data.m_intersectType == INTERSECT_INTERSECTS)
		{
			ctxt.m_currentPolyId = i + 1;
			IntersectType inttest2 = clipPoly3D(poly, plane, clip_result, &ctxt);
			assert(inttest2 == INTERSECT_INTERSECTS);
		}
	}

	// construct new polys incident to the cutting plane
	unsigned int nclips = clip_result.size();
	if(closed && (nclips > nclips_old))
	{
		typedef boost::unordered_map<u_bi_edge, const polyclip_intersection*> 	intersection_map;
		typedef boost::unordered_multimap<u_bi_edge, u_bi_edge>					edge_map;

		intersection_map intersections;
		edge_map edges;

		if(nclips_old == 0)
			itClip = clip_result.begin();
		else
			++itClip;

		for(; itClip!= clip_result.end(); ++itClip)
		{
			const polyclip_type& cpoly = *itClip;

			// merge intersections of the same-but-opposite edge into single intersections
			for(unsigned int j=0; j<cpoly.m_onplanePoints.size(); ++j)
			{
				const polyclip_intersection& ints = cpoly.m_onplanePoints[j];
				u_bi_edge e(ints.m_point1, ints.m_point2);

				assert(e.first() != e.second());
				typename intersection_map::const_iterator it = intersections.find(e);
				if(it == intersections.end())
					intersections.insert(typename intersection_map::value_type(e, &ints));
			}

			// find edges incident to cutting plane
			unsigned int nverts = cpoly.numVertices();
			assert(nverts > 2);

			for(unsigned int j=0; j<nverts; ++j)
			{
				unsigned int k = (j+1)%nverts;
				if(!cpoly.isInside(j) && !cpoly.isInside(k))
				{
					const polyclip_intersection& ints_j = cpoly.getIntersection(j);
					const polyclip_intersection& ints_k = cpoly.getIntersection(k);
					u_bi_edge e_j(ints_j.m_point1, ints_j.m_point2);
					u_bi_edge e_k(ints_k.m_point1, ints_k.m_point2);

					// keyhole can result in edge with same start/end intersection, we
					// can safely skip this when it happens
					if(e_j != e_k)
					{
						// reverse edges to get correct winding
						edges.insert(edge_map::value_type(e_k, e_j));
					}
				}
			}
		}

		// find edge loops. Each of these will be a polygon, or a hole inside a polygon.
		typedef boost::unordered_set<u_bi_edge>		reversed_edge_set;
		typedef std::vector<u_bi_edge>				loop;
		typedef std::list<loop>						loop_list;

		loop l;
		loop_list cw_loops, ccw_loops;

		while(!edges.empty())
		{
			l.clear();

			edge_map::iterator it = edges.begin();
			u_bi_edge first_vert = it->first;

			while(it != edges.end())
			{
				l.push_back(it->first);
				u_bi_edge next_vert = it->second;
				edges.erase(it);
				it = edges.find(next_vert);
			}

			if(l.size() > 2)
			{
				if(break_holes)
				{
					// don't incur the cost of calculating winding until there's > 1 loop
					unsigned int nloops = cw_loops.size() + ccw_loops.size();

					if(nloops == 0)
						cw_loops.push_back(l);
					else
					{
						if(nloops == 1)
						{
							// categorise the first loop
							const loop& l_first = *(cw_loops.begin());
							unsigned int nverts = l_first.size();

							std::vector<vec3_type> l2(nverts);
							for(unsigned int i=0; i<nverts; ++i)
							{
								const u_bi_edge& is = l_first[i];
								assert(intersections.find(is) != intersections.end());
								intersections.find(is)->second->getPos(meshpoints, l2[i]);
							}

							vec3_type norm;
							calc_normal(l2, norm);
							if(r_plane.normal.dot(norm) < 0)
								ccw_loops.splice(ccw_loops.end(), cw_loops);
						}

						// categorise this loop
						unsigned int nverts = l.size();
						std::vector<vec3_type> l2(nverts);

						for(unsigned int i=0; i<nverts; ++i)
						{
							const u_bi_edge& is = l[i];
							assert(intersections.find(is) != intersections.end());
							intersections.find(is)->second->getPos(meshpoints, l2[i]);
						}

						vec3_type norm;
						calc_normal(l2, norm);
						if(r_plane.normal.dot(norm) < 0)
							ccw_loops.push_back(l);
						else
							cw_loops.push_back(l);
					}
				}
				else
					cw_loops.push_back(l);
			}
		}

		if(break_holes && !ccw_loops.empty())
		{
			typedef std::vector<vec2_type> 				loop_2d;
			typedef std::vector<unsigned int> 			new_loop;
			typedef boost::unordered_map<unsigned int, u_bi_edge>	edge_map;

			// project all loops onto the clipping plane. Note that we actually
			// project onto the reverse plane, so that cw polys are cw in 2D also.
			Projection_2D<T> proj(r_plane.normal);
			std::vector<loop_2d> loops_2d[2];
			edge_map remap;
			unsigned int index_2d = 0;

			{
				const loop_list* loops[2] = { &cw_loops, &ccw_loops };

				for(unsigned int j=0; j<2; ++j)
				{
					for(loop_list::const_iterator it=loops[j]->begin(); it!=loops[j]->end(); ++it)
					{
						const loop& l = *it;
						unsigned int nverts = l.size();

						loops_2d[j].push_back(loop_2d(nverts));
						loop_2d& l2d = loops_2d[j].back();
						vec3_type p;

						for(unsigned int i=0; i<nverts; ++i)
						{
							const u_bi_edge& is = l[i];
							remap.insert(edge_map::value_type(index_2d++, is));

							assert(intersections.find(is) != intersections.end());
							intersections.find(is)->second->getPos(meshpoints, p);
							proj.project(p, l2d[i]);
						}
					}
				}
			}

			cw_loops.clear();
			ccw_loops.clear();

			// break holes in 2D
			std::vector<new_loop> newloops;
			breakHoles2D(loops_2d[0], loops_2d[1], newloops);

			// convert back to 3d loops
			for(std::vector<new_loop>::const_iterator it=newloops.begin();
				it!=newloops.end(); ++it)
			{
				const new_loop& new_l = *it;
				unsigned int nverts = new_l.size();

				cw_loops.push_back(loop(nverts));
				loop& l = cw_loops.back();

				for(unsigned int i=0; i<nverts; ++i)
				{
					unsigned int vert = new_l[i];
					assert(remap.find(vert) != remap.end());
					l[i] = remap.find(vert)->second;
				}
			}
		}

		// add final incident polys
		for(loop_list::const_iterator it=cw_loops.begin(); it!=cw_loops.end(); ++it)
		{
			const loop& poly = *it;
			clip_result.push_back(polyclip_type());
			polyclip_type& cpoly = clip_result.back();

			for(loop::const_iterator it2=poly.begin(); it2!=poly.end(); ++it2)
			{
				typename intersection_map::const_iterator it3 = intersections.find(*it2);
				assert(it3 != intersections.end());
				cpoly.m_vertices.push_back(cpoly.m_onplanePoints.size());
				cpoly.m_onplanePoints.push_back(*(it3->second));
			}
		}
	}
}


template<typename Mesh3D>
IntersectType detail::clipMesh3D_to_simple(const Mesh3D& mesh,
	const std::vector<int>* polyIDs,
	const Imath::Plane3<typename mesh_adaptor<Mesh3D>::scalar>& plane,
	bool closed, bool break_holes,
	simple_mesh<typename mesh_adaptor<Mesh3D>::point_type>& result,
	std::vector<int>* resultPolyIDs,
	ClippingContext<typename mesh_adaptor<Mesh3D>::scalar>* ctxt_)
{
	typedef mesh_adaptor<Mesh3D> 					Adaptor;
	typedef typename Adaptor::scalar				T;
	typedef typename Adaptor::poly_type				poly_type;
	typedef typename Adaptor::const_poly_ref		const_poly_ref;
	typedef ClippedPoly<T>							polyclip_type;
	typedef typename polyclip_type::intersection	polyclip_intersection;
	typedef Imath::Plane3<T>						plane3_type;
	typedef Imath::Vec3<T>							vec3_type;
	typedef simple_mesh<vec3_type>					simple_mesh_type;
	typedef detail::ClippingContext<T> 				ctxt_type;

	typedef points_adaptor<poly_type>				PointsAdaptor;
	typedef boost::unordered_map<u_bi_edge, unsigned int>		onplane_map;

	ctxt_type ctxt_local;
	ctxt_type& ctxt = (ctxt_)? *ctxt_ : ctxt_local;

	std::vector<unsigned int> inside_polys;
	std::list<polyclip_type> cpolys;

	// do the plane clip
	IntersectType inters = clipMesh3D(mesh, plane, closed, break_holes,
		cpolys, inside_polys, &ctxt);

	if(inters != INTERSECT_INTERSECTS)
		return inters;

	// convert (inmesh, clip results) -> outmesh. Don't remap points, just add to
	// existing - this results in unused points, but better performance.
	Adaptor mesh_in(mesh);
	mesh_points<Mesh3D> meshpoints(mesh);

	result.clear();
	unsigned int num_newpolys = inside_polys.size() + cpolys.size();
	result.m_polys.resize(num_newpolys);
	if(resultPolyIDs)
		resultPolyIDs->resize(num_newpolys);

	{
		unsigned int npoints = mesh_in.numPoints();
		result.m_points.resize(npoints);
		for(unsigned int i=0; i<npoints; ++i)
			result.m_points[i] = mesh_in.getPoint(i);
	}

	onplane_map intersections;
	unsigned int i = 0;

	for(std::vector<unsigned int>::const_iterator it2=inside_polys.begin();
		it2!=inside_polys.end(); ++it2, ++i)
	{
		PointsAdaptor poly_in(mesh_in.getPoly(*it2));
		std::vector<unsigned int>& resultPoly = result.m_polys[i];

		unsigned int nverts = poly_in.size();
		resultPoly.resize(nverts);
		for(unsigned int j=0; j<nverts; ++j)
			resultPoly[j] = poly_in.index(j);

		if(resultPolyIDs)
		{
			assert(!polyIDs || (*it2 < polyIDs->size()));
			(*resultPolyIDs)[i] = (polyIDs)? (*polyIDs)[*it2] : *it2 + 1;
		}
	}

	for(typename std::list<polyclip_type>::const_iterator it2=cpolys.begin();
		it2!=cpolys.end(); ++it2, ++i)
	{
		std::vector<unsigned int>& resultPoly = result.m_polys[i];
		const polyclip_type& cpoly = *it2;

		for(unsigned int j=0; j<cpoly.numVertices(); ++j)
		{
			if(cpoly.isInside(j))
				resultPoly.push_back(cpoly.getInsideIndex(j));
			else
			{
				const polyclip_intersection& is = cpoly.getIntersection(j);
				u_bi_edge e(is.m_point1, is.m_point2);
				onplane_map::const_iterator it3 = intersections.find(e);
				if(it3 == intersections.end())
				{
					vec3_type pos;
					cpoly.getPos(meshpoints, j, pos);
					it3 = intersections.insert(onplane_map::value_type(e, result.m_points.size())).first;
					result.m_points.push_back(pos);
				}

				resultPoly.push_back(it3->second);
			}
		}

		if(resultPolyIDs)
		{
			int id = it2->m_polyId;
			if(id == 0)
				(*resultPolyIDs)[i] = 0;
			else if(polyIDs)
			{
				if(id > 0)
				{
					assert(id <= polyIDs->size());
					(*resultPolyIDs)[i] = (*polyIDs)[id-1];
				}
				else
				{
					id = -1-id;
					assert(id < polyIDs->size());
					(*resultPolyIDs)[i] = -std::abs((*polyIDs)[id]);
				}
			}
			else
				(*resultPolyIDs)[i] = id;
		}
	}

	return INTERSECT_INTERSECTS;
}


template<typename Mesh3D, typename ConstPlaneIterator>
IntersectType clipMesh3D(const Mesh3D& mesh,
	ConstPlaneIterator planesBegin, ConstPlaneIterator planesEnd,
	bool closed, bool break_holes,
	simple_mesh<typename mesh_adaptor<Mesh3D>::point_type>& result,
	std::vector<int>* pointIDs,
	std::vector<int>* polyIDs,
	typename mesh_adaptor<Mesh3D>::scalar minEdgeLength,
	detail::ClippingContext<typename mesh_adaptor<Mesh3D>::scalar>* ctxt_)
{
	typedef mesh_adaptor<Mesh3D> 			Adaptor;
	typedef typename Adaptor::scalar		T;
	typedef Imath::Vec3<T>					vec3_type;
	typedef simple_mesh<vec3_type>			simple_mesh_type;
	typedef detail::ClippingContext<T> 		ctxt_type;

	if(planesBegin == planesEnd)
		return INTERSECT_INSIDE;

	ctxt_type ctxt_local;
	ctxt_type& ctxt = (ctxt_)? *ctxt_ : ctxt_local;

	simple_mesh_type smesh[2];
	std::vector<int> polyIDs_[2];
	std::vector<int>* poly_ids[2] = { NULL, NULL };
	if(polyIDs)
	{
		poly_ids[0] = &polyIDs_[0];
		poly_ids[1] = &polyIDs_[1];
	}

	bool all_inside = true;
	unsigned int q = 0;

	// apply each plane clip
	for(ConstPlaneIterator it=planesBegin; it!=planesEnd; ++it)
	{
		if(all_inside)
		{
			IntersectType inters = detail::clipMesh3D_to_simple(mesh, NULL, *it, closed,
				break_holes, smesh[0], poly_ids[0], &ctxt);

			if(inters == INTERSECT_OUTSIDE)
				return INTERSECT_OUTSIDE;
			else if(inters == INTERSECT_INTERSECTS)
			{
				q = 1-q;
				all_inside = false;
			}
		}
		else
		{
			IntersectType inters = detail::clipMesh3D_to_simple(smesh[1-q], poly_ids[1-q],
				*it, closed, break_holes, smesh[q], poly_ids[q], &ctxt);

			if(inters == INTERSECT_OUTSIDE)
				return INTERSECT_OUTSIDE;
			else if(inters == INTERSECT_INTERSECTS)
				q = 1-q;
		}
	}

	if(all_inside)
		return INTERSECT_INSIDE;

	// write out final clipped mesh
	q = 1-q;
	simple_mesh_type &smOut = smesh[q];

	// remove degenerate features
	std::vector<int> clean_point_remapping;
	std::vector<int> clean_poly_remapping;
	std::vector<int>* ppt_remap = (pointIDs)? &clean_point_remapping : NULL;
	std::vector<int>* ppoly_remap = (polyIDs)? &clean_poly_remapping : NULL;

	T epsilon = (minEdgeLength < 0)? std::numeric_limits<T>::epsilon()*2 : minEdgeLength;
	cleanMesh(smOut, result, epsilon, ppt_remap, ppoly_remap);

	if(pointIDs)
	{
		// recall that each plane clip only adds points to the working mesh, for performance
		// reasons. So we know that any point at id < original num points is an original.
		unsigned int npoints_orig = Adaptor(mesh).numPoints();
		unsigned int npoints_ = ppt_remap->size();
		pointIDs->resize(npoints_);

		for(unsigned int i=0; i<npoints_; ++i)
		{
			unsigned int new_ptnum = (*ppt_remap)[i];
			(*pointIDs)[i] = (new_ptnum < npoints_orig)? new_ptnum : -1;
		}
	}

	if(polyIDs)
	{
		const std::vector<int>& poly_ids_ = polyIDs_[q];
		unsigned int npolys_ = ppoly_remap->size();
		polyIDs->resize(npolys_);

		for(unsigned int i=0; i<npolys_; ++i)
		{
			int index = (*ppoly_remap)[i];
			assert(index != 0); // cleanMesh should never return 0-index poly

			if(index < 0)
			{
				int new_poly_id = poly_ids_[-1-index];
				(*polyIDs)[i] = (new_poly_id > 0)?
					-new_poly_id : new_poly_id;
			}
			else
				(*polyIDs)[i] = poly_ids_[index-1];
		}
	}

	return INTERSECT_INTERSECTS;
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
