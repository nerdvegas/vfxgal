#include <hdk_utils/ScopedCook.h>
#include <hdk_utils/util.h>
#include <UT/UT_DSOVersion.h>
#include <OP/OP_OperatorTable.h>
#include <GU/GU_PrimPoly.h>
#include <houdini/PRM/PRM_Include.h>
#include <dgal/adaptors/houdini.hpp>
#include <dgal/algorithm/voronoiFractureMesh3D.hpp>
#include <dgal/algorithm/remapMesh.hpp>
#include <hdk_utils/GeoAttributeCopier.h>
#include "SOP_VoronoiFracture.h"
#include "util/simple_mesh.h"


using namespace clip_sops;
using namespace hdk_utils;


void newSopOperator(OP_OperatorTable *table)
{
    table->addOperator(
	    new OP_Operator(SOP_NAME,					// Internal name
			 SOP_LABEL,								// UI name
			 SOP_VoronoiFracture::myConstructor,	// How to build the SOP
			 SOP_VoronoiFracture::myTemplateList,	// My parameters
			 0,										// Min # of sources
			 2,										// Max # of sources
			 SOP_VoronoiFracture::myVariables,		// Local variables
			 0)
	    );
}


// switcher
static PRM_Name switcherName("switcher");
static PRM_Default switcherList[] = {
    PRM_Default(4, "General"),
    PRM_Default(5, "Culling"),
    PRM_Default(2, "Advanced"),
};

// general params
static PRM_Name VF_ParmNamePointIDAttrib( "point_id_attrib", "Point ID" );
static PRM_Default VF_ParmDefaultPointIDAttrib(0, "");

static PRM_Name VF_ParmNamePolyIDAttrib( "poly_id_attrib", "Face ID" );
static PRM_Default VF_ParmDefaultPolyIDAttrib(0, "");

static PRM_Name VF_ParmNameCellIDAttrib( "cell_id_attrib", "Cell ID" );
static PRM_Default VF_ParmDefaultCellIDAttrib(0, "");

static PRM_Name VF_ParmNameCellTypeAttrib( "cell_type_attrib", "Cell Type" );
static PRM_Default VF_ParmDefaultCellTypeAttrib(0, "");

// culling params
static PRM_Name VF_ParmNameIncludeUnboundedCells( "inc_unbounded_cells", "Keep Unbounded Cells" );
static PRM_Name VF_ParmNameIncludeIntersectingCells( "inc_int_cells", "Keep Mesh-Intersecting Cells" );
static PRM_Name VF_ParmNameIncludeInternalCells( "inc_internal_cells", "Keep Internal Cells" );
static PRM_Name VF_ParmNameIncludeExternalCells( "inc_external_cells", "Keep External Cells" );
static PRM_Name VF_ParmNameClipBoundedCells( "clip_bounded_cells", "Clip Bounded Cells" );

static PRM_Default VF_ParmDefaultIncludeUnboundedCells(1);
static PRM_Default VF_ParmDefaultIncludeIntersectingCells(1);
static PRM_Default VF_ParmDefaultIncludeInternalCells(1);
static PRM_Default VF_ParmDefaultIncludeExternalCells(0);
static PRM_Default VF_ParmDefaultBoundedCells(1);

// advanced params
static PRM_Name VF_ParmNameOctreeDepth( "octree_depth", "Octree Depth" );
static PRM_Name VF_ParmNameMinEdgeLength( "min_edge_len", "Edge Fuse Length" );

static PRM_Default VF_ParmDefaultOctreeDepth(8);
static PRM_Default VF_ParmDefaultMinEdgeLength(0.00001f);

static PRM_Range VF_ParmRangeOctreeDepth(PRM_RANGE_RESTRICTED, 0, PRM_RANGE_RESTRICTED, 20);


PRM_Template SOP_VoronoiFracture::myTemplateList[] =
{
	PRM_Template(PRM_SWITCHER, 3, &switcherName, switcherList),

	PRM_Template(PRM_STRING, 1, &VF_ParmNamePointIDAttrib, &VF_ParmDefaultPointIDAttrib, 0, 0, 0, 0, 1,
		"Point ID - -1:new point; >=0:orig point"),
	PRM_Template(PRM_STRING, 1, &VF_ParmNamePolyIDAttrib, &VF_ParmDefaultPolyIDAttrib, 0, 0, 0, 0, 1,
		"Face ID - 0:new face; >0:unclipped face,orig=ID-1; <0:clipped face,orig=-1-ID"),
	PRM_Template(PRM_STRING, 1, &VF_ParmNameCellIDAttrib, &VF_ParmDefaultCellIDAttrib, 0, 0, 0, 0, 1,
		"Voronoi cell site ID"),
	PRM_Template(PRM_STRING, 1, &VF_ParmNameCellTypeAttrib, &VF_ParmDefaultCellTypeAttrib, 0, 0, 0, 0, 1,
		"Cell type - 0:internal; 1:external; 2:clipped"),

	PRM_Template(PRM_TOGGLE, 1, &VF_ParmNameIncludeUnboundedCells, &VF_ParmDefaultIncludeUnboundedCells),
	PRM_Template(PRM_TOGGLE, 1, &VF_ParmNameIncludeIntersectingCells, &VF_ParmDefaultIncludeIntersectingCells),
	PRM_Template(PRM_TOGGLE, 1, &VF_ParmNameIncludeInternalCells, &VF_ParmDefaultIncludeInternalCells),
	PRM_Template(PRM_TOGGLE, 1, &VF_ParmNameIncludeExternalCells, &VF_ParmDefaultIncludeExternalCells),
	PRM_Template(PRM_TOGGLE, 1, &VF_ParmNameClipBoundedCells, &VF_ParmDefaultBoundedCells),
	PRM_Template(PRM_INT, 	 1, &VF_ParmNameOctreeDepth, &VF_ParmDefaultOctreeDepth, 0, &VF_ParmRangeOctreeDepth),
	PRM_Template(PRM_FLT, 	 1, &VF_ParmNameMinEdgeLength, &VF_ParmDefaultMinEdgeLength)
};

CH_LocalVariable SOP_VoronoiFracture::myVariables[] = {
	{ 0 }		// End the table with a null entry
};


const char* SOP_VoronoiFracture::inputLabel(unsigned int idx) const
{
	return (idx == 0)? "Mesh to Fracture" : "Voronoi Cell Sites";
}


OP_Node* SOP_VoronoiFracture::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
    return new SOP_VoronoiFracture(net, name, op);
}


SOP_VoronoiFracture::SOP_VoronoiFracture(OP_Network *net, const char *name, OP_Operator *op)
:	SOP_Node(net, name, op)
{
}


float SOP_VoronoiFracture::getVariableValue(int index, int thread)
{
    return SOP_Node::getVariableValue(index, thread);
}


OP_ERROR SOP_VoronoiFracture::cookMySop(OP_Context &context)
{
	typedef dgal::simple_mesh<Imath::V3f> 						simple_mesh;
	typedef dgal::VoronoiCellMesh<Imath::V3f>					voronoi_cell_mesh;
	typedef boost::shared_ptr<voronoi_cell_mesh>				voronoi_cell_mesh_ptr;
	typedef std::map<unsigned int, voronoi_cell_mesh_ptr>		voronoi_cell_mesh_map;

	hdk_utils::ScopedCook scc(*this, context, "Performing voronoi fracture");
	if(!scc.good())
		return error();

	double now = context.getTime();

	// get inputs
	const GU_Detail* gdp0 = inputGeo(0); //This is the geo to fracture
	const GU_Detail* gdp1 = inputGeo(1); //This is the voronoi cell sites pointlist
	if (!gdp1 ) { SOP_ADD_FATAL(SOP_MESSAGE, "Not enough sources specified."); }

	bool inc_unbounded_cells = 	(evalInt("inc_unbounded_cells", 0, now) != 0);
	bool inc_int_cells = 		(evalInt("inc_int_cells", 0, now) != 0);
	bool inc_internal_cells = 	(evalInt("inc_internal_cells", 0, now) != 0);
	bool inc_external_cells = 	(evalInt("inc_external_cells", 0, now) != 0);
	bool clip_bounded_cells = 	(evalInt("clip_bounded_cells", 0, now) != 0);
	int octree_depth = 			(evalInt("octree_depth", 0, now) != 0);
	float min_edge_len = 		evalFloat("min_edge_len", 0, now);

	UT_String pointIDAttrib;
	evalString(pointIDAttrib, "point_id_attrib", 0, now);
	pointIDAttrib.trimSpace();
	std::string pointIDAttribStr = pointIDAttrib.toStdString();

	UT_String polyIDAttrib;
	evalString(polyIDAttrib, "poly_id_attrib", 0, now);
	polyIDAttrib.trimSpace();
	std::string polyIDAttribStr = polyIDAttrib.toStdString();

	UT_String cellIDAttrib;
	evalString(cellIDAttrib, "cell_id_attrib", 0, now);
	cellIDAttrib.trimSpace();
	std::string cellIDAttribStr = cellIDAttrib.toStdString();

	UT_String cellTypeAttrib;
	evalString(cellTypeAttrib, "cell_type_attrib", 0, now);
	cellTypeAttrib.trimSpace();
	std::string cellTypeAttribStr = cellTypeAttrib.toStdString();

	gdp->clearAndDestroy();

	// generate clipped voronoi cells
	dgal::VoronoiSettings vs(
		true,
		true,
		inc_unbounded_cells,
		inc_int_cells,
		inc_internal_cells,
		inc_external_cells,
		clip_bounded_cells,
		octree_depth,
		min_edge_len);

	voronoi_cell_mesh_map vcells;

	try {
		dgal::voronoiFractureMesh3D<GEO_Detail, GEO_PointList>(
			gdp0, gdp1->points(), vcells, vs);
	}
	catch(const dgal::DgalSubprocessError& e)
	{
		SOP_ADD_FATAL(SOP_MESSAGE, e.what());
	}

	if(gdp0)
	{
		dgal::MeshRemapSettings<int> remap_settings;
		dgal::MeshRemapResult<unsigned int, float> remap_result;
		GeoAttributeCopier::remapping_vector cell_pt_remap;

		// construct the clipped cells as houdini geometry, and build up remapping
		// data at the same time
		for(voronoi_cell_mesh_map::const_iterator it=vcells.begin(); it!=vcells.end(); ++it)
		{
			const voronoi_cell_mesh& vmesh = *(it->second);
			unsigned int cellId = it->first;

			if(vmesh.m_type == dgal::INTERSECT_INTERSECTS)
			{
				util::add_simple_mesh(*gdp, vmesh.m_mesh,
					pointIDAttribStr, &(vmesh.m_pointRemapping),
					polyIDAttribStr, &(vmesh.m_polyRemapping),
					cellTypeAttribStr, static_cast<unsigned int>(vmesh.m_type),
					cellIDAttribStr, cellId);

				remap_settings.m_pointMapping = &(vmesh.m_pointRemapping);
				remap_settings.m_polyMapping = &(vmesh.m_polyRemapping);

				dgal::remapMesh<GEO_Detail, simple_mesh, int>(*gdp0, vmesh.m_mesh,
					remap_settings, remap_result);

				remap_settings.m_firstPoint += vmesh.m_mesh.numPoints();
				remap_settings.m_firstPoly += vmesh.m_mesh.numPolys();

				for(unsigned int i=0; i<vmesh.m_mesh.m_polys.size(); ++i)
					cell_pt_remap.push_back(cellId);
			}
		}

		// construct the internal cells. This is done after the clipped cells so that
		// all the complicated attribute remapping starts at poly/point zero.
		for(voronoi_cell_mesh_map::const_iterator it=vcells.begin(); it!=vcells.end(); ++it)
		{
			const voronoi_cell_mesh& vmesh = *(it->second);
			unsigned int cellId = it->first;

			if(vmesh.m_type != dgal::INTERSECT_INTERSECTS)
			{
				util::add_simple_mesh(*gdp, vmesh.m_mesh,
					pointIDAttribStr, NULL,
					polyIDAttribStr, NULL,
					cellTypeAttribStr, static_cast<unsigned int>(vmesh.m_type),
					cellIDAttribStr, cellId);

				for(unsigned int i=0; i<vmesh.m_mesh.m_polys.size(); ++i)
					cell_pt_remap.push_back(cellId);
			}
		}

		// remap attributes from source geometry onto target geo
		{
			GeoAttributeCopier gc(*gdp);
			gc.add(*gdp0, GEO_DETAIL_DICT, GEO_DETAIL_DICT);
			gc.add(*gdp0, GEO_POINT_DICT, GEO_POINT_DICT, &(remap_result.m_pointMapping));
			gc.add(*gdp0, GEO_POINT_DICT, GEO_POINT_DICT, &(remap_result.m_pointIMapping),
				GeoAttributeCopier::GAC_MERGE);
			gc.add(*gdp0, GEO_PRIMITIVE_DICT, GEO_PRIMITIVE_DICT, &(remap_result.m_polyMapping));
			gc.add(*gdp0, GEO_VERTEX_DICT, GEO_VERTEX_DICT, &(remap_result.m_vertexMapping));
			gc.add(*gdp0, GEO_VERTEX_DICT, GEO_VERTEX_DICT, &(remap_result.m_vertexIMapping),
				GeoAttributeCopier::GAC_MERGE);
			gc.apply();
		}

		// remap cell point attribs to prim attribs on target geo
		{
			GeoAttributeCopier gc(*gdp);
			gc.add(*gdp1, GEO_POINT_DICT, GEO_PRIMITIVE_DICT, &cell_pt_remap);
			gc.apply();
		}
	}

	return error();
}


















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
