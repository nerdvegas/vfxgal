#ifndef _DGAL_ALGO_VORONOIFRACTUREMESH3D__H_
#define _DGAL_ALGO_VORONOIFRACTUREMESH3D__H_

#include <boost/scoped_ptr.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include "../adaptors/octree.hpp"
#include "../VoronoiCell.hpp"
#include "../exceptions.h"
#include "../detail/qhull_voronoi.hpp"
#include "../octree/polygonSet.hpp"
#include "clipMesh3D.hpp"

#ifdef VFXGAL_NO_TBB
#include "../util/no_tbb.hpp"
namespace _tbb = vfxgal::no_tbb;
#else
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/enumerable_thread_specific.h>
namespace _tbb = tbb;
#endif


namespace vfxgal {


	struct VoronoiSettings
	{
		VoronoiSettings(
			bool createPointRemapping = true,
			bool createPolyRemapping = true,
			bool includeUnboundedCells = true,
			bool includeIntersectingCells = true,
			bool includeInternalCells = true,
			bool includeExternalCells = false,
			bool clipBoundedCells = true,
			unsigned int octreeDepth = 8,
			float minEdgeLength = -1,
			bool qhull_verbose = false)
		:	m_createPointRemapping(createPointRemapping),
			m_createPolyRemapping(createPolyRemapping),
			m_includeUnboundedCells(includeUnboundedCells),
			m_includeIntersectingCells(includeIntersectingCells),
			m_includeInternalCells(includeInternalCells),
			m_includeExternalCells(includeExternalCells),
			m_clipBoundedCells(clipBoundedCells),
			m_octreeDepth(octreeDepth),
			m_minEdgeLength(minEdgeLength),
			m_qhull_verbose(qhull_verbose)
		{}

		bool m_createPointRemapping;
		bool m_createPolyRemapping;
		bool m_includeUnboundedCells;
		bool m_includeIntersectingCells;
		bool m_includeInternalCells;
		bool m_includeExternalCells;
		bool m_clipBoundedCells;
		unsigned int m_octreeDepth;
		float m_minEdgeLength;
		bool m_qhull_verbose;
	};


	/*
	 * @class VoronoiCellMesh
	 * @brief
	 * The mesh of a voronoi cell created by the voronoiFractureMesh3D function.
	 * @var m_cellSite The index of the cell site this mesh represents.
	 * @var m_mesh The cell's mesh representation.
	 * @var m_type Whether the cell is inside, outside, or intersects the mesh.
	 * @var m_pointRemapping See remapMesh.MeshRemapSettings.pointMapping.
	 * @var m_polyRemapping See remapMesh.MeshRemapSettings.polyMapping.
	 */
	template<typename Vec>
	struct VoronoiCellMesh
	{
		unsigned int m_cellSite;
		IntersectType m_type;
		bool m_clipped;
		simple_mesh<Vec> m_mesh;
		std::vector<int> m_pointRemapping;
		std::vector<int> m_polyRemapping;
	};


	/*
	 * @brief voronoiFractureMesh3D
	 * Fracture the given mesh into the voronoi cells resulting from the given voronoi
	 * cell sites.
	 * @param Mesh to fracture. If not supplied, voronoi cells are unclipped, and unbound
	 * cells are not included.
	 * @param cell_sites The positions of each cell site.
	 * @param[out] cells Resulting cell meshes (key matches cell site indices).
	 * @param settings Algorithm settings.
	 * @note The mesh, if provided, is assumed to be manifold, and each faces' points
	 * coplanar. If this requirement is not met then clipping errors can occur.
	 */
	template<typename Mesh3D, typename Points3D>
	void voronoiFractureMesh3D(const Mesh3D* mesh, const Points3D& cell_sites,
		std::map<unsigned int, boost::shared_ptr<
			VoronoiCellMesh<typename mesh_adaptor<Mesh3D>::point_type> > >& cells,
		const VoronoiSettings& settings = VoronoiSettings());


	namespace detail {

		template<typename Mesh3D>
		struct thread_voronoiFractureMesh3D
		{
			typedef mesh_adaptor<Mesh3D> 							MeshAdaptor;
			typedef typename MeshAdaptor::scalar					scalar;
			typedef typename MeshAdaptor::point_type				point_type;
			typedef Imath::Plane3<scalar>							plane3_type;
			typedef Imath::Box<point_type>							box_type;
			typedef LineSegment<point_type>							lineseg_type;
			typedef simple_mesh<point_type>							simple_mesh_type;
			typedef VoronoiCell3D<scalar>							voronoi_cell;
			typedef PolygonSet<Mesh3D>								polygonset_type;
			typedef Octree<polygonset_type>							octree_type;
			typedef detail::ClippingContext<scalar> 				ctxt_type;
			typedef VoronoiCellMesh<point_type>						voronoi_cell_mesh;
			typedef boost::shared_ptr<voronoi_cell_mesh>			voronoi_cell_mesh_ptr;
			typedef std::map<unsigned int, voronoi_cell_mesh_ptr>	voronoi_cell_mesh_map;

			struct local_data
			{
				voronoi_cell_mesh_map m_cells;
			};

			typedef _tbb::enumerable_thread_specific<local_data> thread_local_data;

			thread_voronoiFractureMesh3D(const Mesh3D* mesh,
				const std::vector<point_type>& vpoints,
				const std::vector<voronoi_cell>& vcells,
				const polygonset_type* polyset,
				const octree_type* octree,
				const VoronoiSettings& settings,
				thread_local_data& localData)
			:	m_mesh(mesh),
				m_vpoints(vpoints),
				m_vcells(vcells),
				m_polyset(polyset),
				m_octree(octree),
				m_settings(settings),
				m_localData(localData)
			{}

			void operator()(const _tbb::blocked_range<unsigned int>& r) const;

			void outputUnclippedCellMesh(simple_mesh_type& smesh, const voronoi_cell& vcell) const;

			const Mesh3D* m_mesh;
			const std::vector<point_type>& m_vpoints;
			const std::vector<voronoi_cell>& m_vcells;
			const polygonset_type* m_polyset;
			const octree_type* m_octree;
			const VoronoiSettings& m_settings;
			thread_local_data& m_localData;
		};


		template<typename T>
		struct vcell_cut_plane_compare
		{
			typedef Imath::Plane3<T> 	plane_type;
			typedef Imath::Vec3<T>		vec_type;

			vcell_cut_plane_compare(const vec_type& toMeshCentre)
			: m_toMeshCentre(toMeshCentre){}

			bool operator()(const plane_type& a, const plane_type& b) {
				return (a.normal.dot(m_toMeshCentre) < b.normal.dot(m_toMeshCentre));
			}

			vec_type m_toMeshCentre;
		};

	}


///////////////////////// impl

template<typename Mesh3D>
void detail::thread_voronoiFractureMesh3D<Mesh3D>::operator()(
	const _tbb::blocked_range<unsigned int>& r) const
{
	ctxt_type ctxt;
	voronoi_cell_mesh_map* pcells;

	{
		typename thread_local_data::reference ref = m_localData.local();
		pcells = &(ref.m_cells);
	}

	point_type mesh_centre;
	box_type mesh_box;
	if(m_mesh)
	{
		assert(m_polyset);
		assert(m_octree);
		m_polyset->getTotalBounds(mesh_box);
		mesh_centre = mesh_box.center();
	}

	for(unsigned int ri=r.begin(); ri!=r.end(); ++ri)
	{
		const voronoi_cell& vcell = m_vcells[ri];
		voronoi_cell_mesh_ptr p_vmesh = voronoi_cell_mesh_ptr(new voronoi_cell_mesh());
		voronoi_cell_mesh& vmesh = *p_vmesh;
		vmesh.m_cellSite = ri;
		vmesh.m_clipped = false;

		if(vcell.m_isBound)
		{
			if(!m_mesh && !m_settings.m_includeInternalCells)
				continue;

			bool intersects = (m_mesh && m_octree->intersect(vcell.m_bounds));
			if(intersects)
			{
				if(m_settings.m_includeIntersectingCells && !m_settings.m_clipBoundedCells)
				{
					// cell should be clipped but clipping is explicitly turned off
					vmesh.m_type = INTERSECT_INTERSECTS;
					outputUnclippedCellMesh(vmesh.m_mesh, vcell);
					pcells->insert(typename voronoi_cell_mesh_map::value_type(ri, p_vmesh));

					continue;
				}
			}
			else
			{
				if(m_settings.m_includeInternalCells || m_settings.m_includeExternalCells)
				{
					vmesh.m_type = INTERSECT_INSIDE;
					if(m_mesh && (m_settings.m_includeInternalCells != m_settings.m_includeExternalCells))
					{
						// do mesh inside/outside test on the cell. We only need to test one
						// point - we use the bounding box centre, a property of a convex hull
						// is that this point will always be inside the hull
						bool cellInside = false;

						{
							point_type ray_start = vcell.m_bounds.center();
							if(mesh_box.intersects(ray_start))
							{
								// intersect axis-aligned ray with octree, and count intersections
								point_type ray_end = Imath::closestPointOnBox(ray_start, mesh_box);
								lineseg_type ray_seg(ray_start, ray_end);
								unsigned int ray_axis = getLargestAxis(ray_seg.vector());
								bool fwd = (ray_seg.vector()[ray_axis] > 0);

								std::vector<boost::unordered_set<unsigned> > polys_(1);
								m_octree->query(ray_start, ray_axis, fwd, polys_);

								unsigned int numIntersect = 0;
								const boost::unordered_set<unsigned>& polys = polys_[0];
								MeshAdaptor a(*m_mesh);

								for(boost::unordered_set<unsigned>::const_iterator it=polys.begin();
									it!=polys.end(); ++it)
								{
									if(intersect(a.getPoly(*it), ray_seg))
										++numIntersect;
								}

								cellInside = (numIntersect%2 == 1);
							}
						}

						if(	(cellInside && !m_settings.m_includeInternalCells) ||
							(!cellInside && !m_settings.m_includeExternalCells) )
							continue;

						vmesh.m_type = (cellInside)? INTERSECT_INSIDE : INTERSECT_OUTSIDE;
					}

					// cell is bound and non-mesh-intersecting
					outputUnclippedCellMesh(vmesh.m_mesh, vcell);
					pcells->insert(typename voronoi_cell_mesh_map::value_type(ri, p_vmesh));
				}

				continue;
			}
		}
		else
		{
			// cell is unbound
			if(!m_mesh || !m_settings.m_includeUnboundedCells)
				continue;
		}

		if(m_mesh && m_settings.m_includeIntersectingCells)
		{
			// the cell probably intersects the mesh surface - perform clipping. Sort
			// cut planes to try and remove the most polys with earlier planes
			vcell_cut_plane_compare<scalar> cmp(mesh_centre - vcell.m_bounds.center());
			std::vector<plane3_type> cutplanes = vcell.m_cutPlanes;
			std::sort(cutplanes.begin(), cutplanes.end(), cmp);

			std::vector<int>* ppointIDs = (m_settings.m_createPointRemapping)?
				&(vmesh.m_pointRemapping) : NULL;

			std::vector<int>* ppolyIDs = (m_settings.m_createPolyRemapping)?
				&(vmesh.m_polyRemapping) : NULL;

			// do mesh/plane clip
			vmesh.m_type = clipMesh3D(*m_octree,
				cutplanes.begin(), cutplanes.end(),
				true, true, vmesh.m_mesh,
				ppointIDs, ppolyIDs,
				m_settings.m_minEdgeLength, &ctxt);

			// the cell is close to the mesh surface but is outside and doesn't touch
			if(vmesh.m_type == INTERSECT_OUTSIDE)
			{
				if(m_settings.m_includeExternalCells)
				{
					simple_mesh_type& smesh = vmesh.m_mesh;
					outputUnclippedCellMesh(smesh, vcell);
					pcells->insert(typename voronoi_cell_mesh_map::value_type(ri, p_vmesh));
				}

				continue;
			}

			// the whole mesh is entirely inside this one voronoi cell!
			if(vmesh.m_type == INTERSECT_INSIDE)
			{
				vmesh.m_type = INTERSECT_INSIDE;
				vmesh.m_mesh.setMesh(*m_mesh);

				if(ppointIDs)
				{
					unsigned int npoints = MeshAdaptor(*m_mesh).numPoints();
					ppointIDs->resize(npoints);

					for(unsigned int i=0; i<npoints; ++i)
						(*ppointIDs)[i] = i;
				}

				if(ppolyIDs)
				{
					unsigned int npolys = MeshAdaptor(*m_mesh).numPolys();
					ppolyIDs->resize(npolys);

					for(unsigned int i=0; i<npolys; ++i)
						(*ppolyIDs)[i] = i;
				}
			}
			else
			{
				// the cell is clipped by the mesh surface
				vmesh.m_clipped = true;
			}

			pcells->insert(typename voronoi_cell_mesh_map::value_type(ri, p_vmesh));
		}
	}
}


template<typename Mesh3D>
void detail::thread_voronoiFractureMesh3D<Mesh3D>::outputUnclippedCellMesh(
	simple_mesh_type& smesh, const voronoi_cell& vcell) const
{
	smesh.clear();
	smesh.m_polys.resize(vcell.m_faces.size());

	typedef boost::unordered_map<unsigned int, unsigned int> u_u_map;
	u_u_map pt_remap;

	for(unsigned int i=0; i<vcell.m_faces.size(); ++i)
	{
		const std::vector<unsigned int>& face = vcell.m_faces[i];
		std::vector<unsigned int>& poly = smesh.m_polys[i];
		poly.resize(face.size());

		for(unsigned int j=0; j<face.size(); ++j)
		{
			unsigned int vert = face[j];
			u_u_map::iterator it = pt_remap.find(vert);
			if(it == pt_remap.end())
			{
				unsigned int newvert = smesh.m_points.size();
				pt_remap.insert(u_u_map::value_type(vert, newvert));
				smesh.m_points.push_back(m_vpoints[vert]);
				poly[j] = newvert;
			}
			else
				poly[j] = it->second;
		}
	}
}


template<typename Mesh3D, typename Points3D>
void voronoiFractureMesh3D(const Mesh3D* mesh, const Points3D& cell_sites,
	std::map<unsigned int, boost::shared_ptr<
		VoronoiCellMesh<typename mesh_adaptor<Mesh3D>::point_type> > >& cells,
	const VoronoiSettings& s)
{
	typedef mesh_adaptor<Mesh3D>					Adaptor;
	typedef typename Adaptor::scalar				scalar;
	typedef typename Adaptor::point_type			point_type;
	typedef Imath::Box<point_type>					box_type;
	typedef Imath::Plane3<scalar>					plane3_type;
	typedef simple_mesh<point_type>					simple_mesh_type;
	typedef PolygonSet<Mesh3D>						polygonset_type;
	typedef Octree<polygonset_type>					octree_type;
	typedef VoronoiCell3D<scalar>					voronoi_cell;
	typedef VoronoiCellMesh<point_type>				voronoi_cell_mesh;
	typedef boost::shared_ptr<voronoi_cell_mesh>	voronoi_cell_mesh_ptr;

	// check for settings which would produce no cells
	if(!s.m_includeIntersectingCells && !s.m_includeInternalCells
		&& !s.m_includeExternalCells) 								return;
	if(!mesh && !s.m_includeInternalCells) 							return;

	// generate voronoi cells
	std::vector<voronoi_cell> vcells;
	std::vector<point_type> vpoints;
	detail::qhull_createVoronoiCells(cell_sites, vpoints, vcells, s.m_qhull_verbose);

	// convert mesh into octree
	boost::scoped_ptr<polygonset_type> polyset;
	boost::scoped_ptr<octree_type> octree;
	if(mesh)
	{
		unsigned int npolys = Adaptor(*mesh).numPolys();
		unsigned int n = std::max(1 << s.m_octreeDepth, 1);
		unsigned int polysPerTreeLeaf = std::max(npolys/n, 1u);

		polyset.reset(new polygonset_type(*mesh));
		octree.reset(new octree_type(*polyset, polysPerTreeLeaf));
	}

	// iterate over vcells
	{
		typedef typename detail::thread_voronoiFractureMesh3D<Mesh3D>::thread_local_data thread_data;

		thread_data tdata;
		detail::thread_voronoiFractureMesh3D<Mesh3D> body(mesh, vpoints,
			vcells, polyset.get(), octree.get(), s, tdata);

		// do mesh/cell clips in multiple threads
		_tbb::blocked_range<unsigned int> br(0, vcells.size());
		_tbb::parallel_for(br, body);

		// gather thread outputs into single result
		cells.clear();
		for(typename thread_data::iterator it=tdata.begin(); it!=tdata.end(); ++it)
			cells.insert(it->m_cells.begin(), it->m_cells.end());
	}
}

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
