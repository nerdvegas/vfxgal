#ifndef _DGAL_DETAIL_QHULL_VORONOI__H_
#define _DGAL_DETAIL_QHULL_VORONOI__H_

#include "../VoronoiCell.hpp"
#include "../exceptions.h"
#include <sstream>
#include <limits>
#include <cstdio>
#include <pystring.h>
#include <boost/process.hpp>


namespace vfxgal { namespace detail {


	/*
	 * @brief qhull_createVoronoiCells
	 * Creates the set of voronoi cells resulting from the given site points, using qhull
	 * as a subprocess.
	 * @param points The voronoi cell sites.
	 * @param[out] voronoi_points The voronoi cell points. VoronoiCell objects refer to
	 * points in this vector. This list is appended to.
	 * @param[out] cells The resulting voronoi cells. This list is appended to.
	 * @note This function may throw a DgalSubprocessError if Qhull fails (this can happen
	 * for eg if the site count is too low).
	 */
	template<typename Points3D>
	void qhull_createVoronoiCells(const Points3D& points,
		std::vector<typename points_adaptor<Points3D>::elem_type>& voronoi_points,
		std::vector<VoronoiCell3D<typename points_adaptor<Points3D>::scalar> >& cells,
		bool verbose=false);


	template<typename T>
	struct qhull_voronoi_data
	{
		typedef Imath::Vec3<T>		vec3_type;
		typedef VoronoiCell3D<T>	voronoi_cell;

		enum Mode
		{
			FIRST_LINE = 0,
			VERTICES_REGIONS_COUNT,
			INF_VERTEX,
			VERTICES,
			REGIONS,
			FACE_COUNT,
			FACES,
			BOUNDED_HYPERPLANE_COUNT,
			BOUNDED_HYPERPLANES,
			UNBOUNDED_HYPERPLANE_COUNT,
			UNBOUNDED_HYPERPLANES,
			DONE
		};

		qhull_voronoi_data(std::vector<vec3_type>& vertices, std::vector<voronoi_cell>& cells,
				bool verbose=false)
		:	m_mode(FIRST_LINE),
			m_nVoronoiVerts(0),
			m_nVoronoiRegions(0),
			m_nVoronoiFaces(0),
			m_nHyperplanes(0),
			m_ncurrVoronoiVerts(0),
			m_ncurrVoronoiRegions(0),
			m_ncurrVoronoiFaces(0),
			m_ncurrHyperplanes(0),
			m_first_vert(vertices.size()),
			m_first_cell(cells.size()),
			m_voronoiVertices(vertices),
			m_cells(cells),
			m_verbose(verbose)
			{}

		template<typename Points3D>
		bool consume_line(const Points3D& sites, const std::string& ln);

		inline bool done() const { return (m_mode == DONE); }

		Mode m_mode;
		unsigned int m_nVoronoiVerts;
		unsigned int m_nVoronoiRegions;
		unsigned int m_nVoronoiFaces;
		unsigned int m_nHyperplanes;

		unsigned int m_ncurrVoronoiVerts;
		unsigned int m_ncurrVoronoiRegions;
		unsigned int m_ncurrVoronoiFaces;
		unsigned int m_ncurrHyperplanes;

		unsigned int m_first_vert;
		unsigned int m_first_cell;

		std::vector<vec3_type>& m_voronoiVertices;
		std::vector<voronoi_cell>& m_cells;
		bool m_verbose;
	};


///////////////////////// impl

template<typename T>
template<typename Points3D>
bool qhull_voronoi_data<T>::consume_line(const Points3D& sites, const std::string& ln)
{
	typedef Imath::Vec3<T>								vec3_type;
	typedef Imath::Plane3<T>							plane3_type;
	typedef points_adaptor<Points3D> 					Adaptor;
	typedef typename Adaptor::const_elem_ref			const_point_ref;
	typedef std::pair<unsigned int, unsigned int>		site_pair;
	Adaptor a(sites);

	if(m_verbose)
		std::cout << "qhull output line: " << ln << std::endl;

	if(m_mode == DONE)
		return true;

	std::string s = pystring::strip(ln);
	if(s.empty() || pystring::startswith(s, "#"))
		return true;

	switch(m_mode)
	{
	case FIRST_LINE:
		{
			m_mode = VERTICES_REGIONS_COUNT;
		}
		break;
	case VERTICES_REGIONS_COUNT:
		{
			sscanf(ln.c_str(), "%d %d", &m_nVoronoiVerts, &m_nVoronoiRegions);
			if(m_nVoronoiRegions != a.size())
				return false;

			m_nVoronoiVerts--; // skip inf vertex
			m_voronoiVertices.resize(m_first_vert + m_nVoronoiVerts);
			m_cells.resize(m_first_cell + m_nVoronoiRegions);

			m_mode = INF_VERTEX;
		}
		break;
	case INF_VERTEX:
		{
			m_mode = VERTICES;
		}
		break;
	case VERTICES:
		{
			Imath::V3f v;
			if(sscanf(ln.c_str(), "%f %f %f", &v.x, &v.y, &v.z) != 3)
				return false;

			m_voronoiVertices[m_first_vert + m_ncurrVoronoiVerts] = v;
			if(++m_ncurrVoronoiVerts == m_nVoronoiVerts)
				m_mode = REGIONS;
		}
		break;
	case REGIONS:
		{
			voronoi_cell& cell = m_cells[m_first_cell + m_ncurrVoronoiRegions];
			cell.m_isBound = true;

			std::istringstream ss(ln);
			unsigned int nverts = 0;
			ss >> nverts;
			for(unsigned int i=0; i<nverts; ++i)
			{
				unsigned int vert;
				ss >> vert;
				if(vert == 0)
					cell.m_isBound = false;
				else
				{
					const vec3_type& pt = m_voronoiVertices[m_first_vert + vert - 1];
					cell.m_bounds.extendBy(pt);
				}
			}

			if(ss.fail())
				return false;

			if(++m_ncurrVoronoiRegions == m_nVoronoiRegions)
				m_mode = FACE_COUNT;
		}
		break;
	case FACE_COUNT:
		{
			int nfaces = -1;
			sscanf(ln.c_str(), "%d", &nfaces);
			if(nfaces == -1)
				return false;

			m_nVoronoiFaces = static_cast<unsigned int>(nfaces);
			m_mode = FACES;
		}
		break;
	case FACES:
		{
			site_pair p;
			unsigned int nverts;
			std::istringstream ss(ln);
			ss >> nverts >> p.first >> p.second;
			nverts -= 2;
			if(ss.fail() || (nverts < 3) || (p.first >= m_nVoronoiRegions)
				|| (p.second >= m_nVoronoiRegions))
			{
				return false;
			}

			std::vector<unsigned int> verts(nverts);
			bool bounded_face = true;

			for(unsigned int i=0; i<nverts; ++i)
			{
				unsigned int vert;
				ss >> vert;
				if(vert == 0)
				{
					bounded_face = false;
					break;
				}
				else
					verts[i] = m_first_vert + vert - 1;
			}

			if(bounded_face)
			{
				// construct an equivalent but opposite-facing face for each cell in the
				// neighbouring pair. Irritatingly, qhull's output does not show consistent
				// winding order, so we have to detect it.
				std::vector<unsigned int> r_verts(verts);
				std::reverse(r_verts.begin(), r_verts.end());

				plane3_type plane;
				{
					points_indexer<std::vector<vec3_type> > facePts(
						&verts[0], &verts[0]+nverts, m_voronoiVertices);
					get_poly_plane(facePts, plane);
				}

				if(isInside(plane, a[p.first]))
				{
					m_cells[m_first_cell + p.first].m_faces.push_back(r_verts);
					m_cells[m_first_cell + p.second].m_faces.push_back(verts);
				}
				else
				{
					m_cells[m_first_cell + p.first].m_faces.push_back(verts);
					m_cells[m_first_cell + p.second].m_faces.push_back(r_verts);
				}
			}

			if(ss.fail())
				return false;

			if(++m_ncurrVoronoiFaces == m_nVoronoiFaces)
				m_mode = BOUNDED_HYPERPLANE_COUNT;
		}
		break;
	case BOUNDED_HYPERPLANE_COUNT:
	case UNBOUNDED_HYPERPLANE_COUNT:
		{
			int nplanes = -1;
			sscanf(ln.c_str(), "%d", &nplanes);
			if(nplanes == -1)
				return false;

			if(nplanes == 0)
			{
				m_mode = (m_mode == BOUNDED_HYPERPLANE_COUNT)?
					UNBOUNDED_HYPERPLANE_COUNT : DONE;
			}
			else
			{
				m_nHyperplanes = static_cast<unsigned int>(nplanes);
				m_ncurrHyperplanes = 0;

				m_mode = (m_mode == BOUNDED_HYPERPLANE_COUNT)?
					BOUNDED_HYPERPLANES : UNBOUNDED_HYPERPLANES;
			}
		}
		break;
	case BOUNDED_HYPERPLANES:
	case UNBOUNDED_HYPERPLANES:
		{
			plane3_type plane;
			site_pair p;
			int six = 0;

			std::istringstream ss(ln);
			ss >> six >> p.first >> p.second >>
				plane.normal.x >> plane.normal.y >> plane.normal.z >> plane.distance;

			// qhull seems to have slightly different repr of plane...
			plane.distance = -plane.distance;

			if((six != 6) || ss.fail() || (p.first >= m_nVoronoiRegions)
				|| (p.second >= m_nVoronoiRegions))
			{
				return false;
			}

			// construct an equivalent but opposite-facing plane for each cell in the
			// neighbouring pair. Irritatingly, qhull's output does not show consistent
			// plane directionality, so we have to detect it.
			plane3_type r_plane = reverse(plane);
			if(isInside(plane, a[p.first]))
			{
				m_cells[m_first_cell + p.first].m_cutPlanes.push_back(plane);
				m_cells[m_first_cell + p.second].m_cutPlanes.push_back(r_plane);
			}
			else
			{
				m_cells[m_first_cell + p.first].m_cutPlanes.push_back(r_plane);
				m_cells[m_first_cell + p.second].m_cutPlanes.push_back(plane);
			}

			if(++m_ncurrHyperplanes == m_nHyperplanes)
			{
				m_mode = (m_mode == BOUNDED_HYPERPLANES)?
					UNBOUNDED_HYPERPLANE_COUNT : DONE;
			}
		}
		break;
	default: break;
	}

	return true;
}


template<typename Points3D>
void qhull_createVoronoiCells(const Points3D& points,
	std::vector<typename points_adaptor<Points3D>::elem_type>& voronoi_points,
	std::vector<VoronoiCell3D<typename points_adaptor<Points3D>::scalar> >& cells, bool verbose)
{
	typedef points_adaptor<Points3D> 			Adaptor;
	typedef typename Adaptor::scalar			scalar;
	typedef qhull_voronoi_data<scalar>			qhull_data;
	typedef typename Adaptor::const_elem_ref	const_point_ref;

	// print points in OFF format
	std::string ptsOFF;
	{
		Adaptor a(points);
		unsigned int npoints = a.size();

		std::ostringstream ss;
		ss << "3\n";
		ss << npoints << '\n';

		ss.setf(std::ios::scientific|std::ios::fixed, std::ios::floatfield);
		ss.precision(16);
		for(unsigned int i=0; i<npoints; ++i)
		{
			const_point_ref p = a[i];
			ss << p.x << ' ' << p.y << ' ' << p.z << '\n';
		}

		ptsOFF = ss.str();
		if(verbose)
			std::cout << "\nqhull input points in OFF format:\n" << ptsOFF << std::endl;
	}

	// run qhull and create cells
	qhull_data data(voronoi_points, cells, verbose);
	{
		// spawn qhull subprocess
		namespace bp = ::boost::process;
		bp::context ctx;
		ctx.environment = bp::self::get_environment();
		ctx.stdin_behavior = bp::capture_stream();
		ctx.stdout_behavior = bp::capture_stream();
		ctx.stderr_behavior = bp::capture_stream();

		std::vector<std::string> args;
		args.push_back(""); // fixes an apparent bug in boost.process
		args.push_back("o");
		args.push_back("Fv");
		args.push_back("Fi");
		args.push_back("Fo");
		std::string cmd = boost::process::find_executable_in_path("qvoronoi");
		bp::child cproc = bp::launch(cmd, args, ctx);

		// pipe voronoi cell sites into qhull stdin
		cproc.get_stdin() << ptsOFF;
		cproc.get_stdin().close();

		// process qhull output
		bp::pistream &is = cproc.get_stdout();
		std::string ln;
		bool done = false;

		while(!data.done() && std::getline(is, ln))
		{
			if(!data.consume_line(points, ln)) {
				DGAL_THROW(DgalSubprocessError, "unexpected qhull output: \'" << ln << "\'");
			}
		}

		// finish up qhull subprocess
		bp::status s = cproc.wait();
		if(!s.exited()) {
			DGAL_THROW(DgalSubprocessError, "qhull process failed to exit.");
		}
		else if(s.exit_status() != 0)
		{
			std::string strerr;
			bp::pistream &iserr = cproc.get_stderr();
			while (std::getline(iserr, ln))
				strerr += ln + "\n";

			DGAL_THROW(DgalSubprocessError, "qhull process failed:\n" << strerr);
		}
	}
}

} }

#endif











