#include <iostream>
#include <stdlib.h>
#include "vfxgal/plane.hpp"
#include "vfxgal/line3.hpp"
#include "vfxgal/simple_mesh.hpp"
#include "vfxgal/algorithm/voronoiFractureMesh3D.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>

using namespace Imath;

typedef float T;
typedef Vec3<T>									vec3_type;
typedef Line3<T>								line3_type;
typedef vfxgal::simple_mesh<vec3_type>			simple_mesh_type;
typedef vfxgal::VoronoiCellMesh<vec3_type>		vcell_type;
typedef boost::shared_ptr<vcell_type>			vcell_type_ptr;
typedef std::map<unsigned int, vcell_type_ptr>	vcell_map;


void create_unit_cube(simple_mesh_type& m);


int main(int argc, char** argv)
{
	std::cout << "testing plane intersection... ";
	{
		Plane3f p1(V3f(0,1,0), 2);
		Plane3f p2(V3f(1,0,1), 0);
		line3_type l;
		vfxgal::intersect(p1, p2, l);
		assert(vfxgal::equivalent_line(l, line3_type(vec3_type(0,2,0), vec3_type(-1,2,1)), true));
		std::cout << "Done." << std::endl;
	}

	std::cout << "testing voronoi fracture... ";
	{
		simple_mesh_type m;
		create_unit_cube(m);

		std::vector<vec3_type> scatter_pts;
		scatter_pts.push_back(vec3_type(-0.515961, 0.430299, 0.163856));
		scatter_pts.push_back(vec3_type(0.00293567, -0.0618937, -0.333178));
		scatter_pts.push_back(vec3_type(-0.371831, -0.436775, 0.291822));
		scatter_pts.push_back(vec3_type(0.127858, 0.0630668, -0.208178));
		scatter_pts.push_back(vec3_type(-0.121987, -0.186854, -0.458178));

		vcell_map cells;
		vfxgal::voronoiFractureMesh3D(&m, scatter_pts, cells);

		assert(cells.size() == 5);

		assert(cells[0]->m_mesh.numPoints() == 12);
		assert(cells[0]->m_mesh.numPolys() == 8);
		assert(cells[0]->m_mesh.numVertices() == 36);

		assert(cells[1]->m_mesh.numPoints() == 12);
		assert(cells[1]->m_mesh.numPolys() == 8);
		assert(cells[1]->m_mesh.numVertices() == 36);

		assert(cells[2]->m_mesh.numPoints() == 8);
		assert(cells[2]->m_mesh.numPolys() == 6);
		assert(cells[2]->m_mesh.numVertices() == 24);

		assert(cells[3]->m_mesh.numPoints() == 10);
		assert(cells[3]->m_mesh.numPolys() == 7);
		assert(cells[3]->m_mesh.numVertices() == 30);

		assert(cells[4]->m_mesh.numPoints() == 6);
		assert(cells[4]->m_mesh.numPolys() == 5);
		assert(cells[4]->m_mesh.numVertices() == 18);
		std::cout << "Done." << std::endl;
	}

	const unsigned int npoints = 10000;
	std::cout << "random points voronoi fracture: " << npoints << " cells..." << std::endl;
	{
		simple_mesh_type m;
		create_unit_cube(m);

		std::vector<vec3_type> scatter_pts(npoints);
		for(unsigned int i=0; i<npoints; ++i)
		{
			for(unsigned int j=0; j<3; ++j)
				scatter_pts[i][j] = ((float)rand() / RAND_MAX) - 0.5f;
		}

		vcell_map cells;
		vfxgal::voronoiFractureMesh3D(&m, scatter_pts, cells);
		std::cout << "Done." << std::endl;
	}

	std::cout << "\nAll tests passed." << std::endl;
	return 0;
}


void create_unit_cube(simple_mesh_type& m)
{
	m.clear();

	std::vector<vec3_type> box_pts;
	box_pts.push_back(vec3_type(-0.5,	-0.5,	-0.5));
	box_pts.push_back(vec3_type(0.5,	-0.5,	-0.5));
	box_pts.push_back(vec3_type(0.5,	-0.5,	0.5));
	box_pts.push_back(vec3_type(-0.5,	-0.5,	0.5));
	box_pts.push_back(vec3_type(-0.5,	0.5,	-0.5));
	box_pts.push_back(vec3_type(0.5,	0.5,	-0.5));
	box_pts.push_back(vec3_type(0.5,	0.5,	0.5));
	box_pts.push_back(vec3_type(-0.5,	0.5,	0.5));
	m.setPoints(box_pts);

	std::vector<unsigned int> poly;
	poly.push_back(1);
	poly.push_back(5);
	poly.push_back(4);
	poly.push_back(0);
	m.appendPoly(poly);
	poly.clear();
	poly.push_back(2);
	poly.push_back(6);
	poly.push_back(5);
	poly.push_back(1);
	m.appendPoly(poly);
	poly.clear();
	poly.push_back(3);
	poly.push_back(7);
	poly.push_back(6);
	poly.push_back(2);
	m.appendPoly(poly);
	poly.clear();
	poly.push_back(0);
	poly.push_back(4);
	poly.push_back(7);
	poly.push_back(3);
	m.appendPoly(poly);
	poly.clear();
	poly.push_back(2);
	poly.push_back(1);
	poly.push_back(0);
	poly.push_back(3);
	m.appendPoly(poly);
	poly.clear();
	poly.push_back(5);
	poly.push_back(6);
	poly.push_back(7);
	poly.push_back(4);
	m.appendPoly(poly);
}


















