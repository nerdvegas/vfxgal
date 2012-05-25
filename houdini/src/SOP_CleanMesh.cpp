#include <UT/UT_DSOVersion.h>
#include <OP/OP_OperatorTable.h>
#include <GU/GU_PrimPoly.h>
#include <PRM/PRM_Include.h>
#include <vfxgal/adaptors/houdini.hpp>
#include <vfxgal/algorithm/cleanMesh.hpp>
#include <vfxgal/algorithm/remapMesh.hpp>
#include "SOP_CleanMesh.h"
#include "util/simple_mesh.h"
#include "util/ScopedCook.h"
#include "util/util.h"
#include "util/GeoAttributeCopier.h"


using namespace vfxgal_hou;

void newSopOperator(OP_OperatorTable *table)
{
    table->addOperator(
	    new OP_Operator(SOP_NAME,			// Internal name
			 SOP_LABEL,						// UI name
			 SOP_CleanMesh::myConstructor,		// How to build the SOP
			 SOP_CleanMesh::myTemplateList,		// My parameters
			 0,								// Min # of sources
			 1,								// Max # of sources
			 SOP_CleanMesh::myVariables,		// Local variables
			 0)
	    );
}


static PRM_Name CleanMesh_ParmNameMinEdgeLen( "min_edge_len", "Min edge length" );
static PRM_Default CleanMesh_ParmDefaultMinEdgeLen(0.00001f);

PRM_Template SOP_CleanMesh::myTemplateList[] = {
	PRM_Template(PRM_FLT, 1, &CleanMesh_ParmNameMinEdgeLen, &CleanMesh_ParmDefaultMinEdgeLen),
	PRM_Template()
};

CH_LocalVariable SOP_CleanMesh::myVariables[] = {
	{ 0 }		// End the table with a null entry
};


const char* SOP_CleanMesh::inputLabel(unsigned int idx) const
{
	return "Mesh to Clean";
}


OP_Node* SOP_CleanMesh::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
    return new SOP_CleanMesh(net, name, op);
}


SOP_CleanMesh::SOP_CleanMesh(OP_Network *net, const char *name, OP_Operator *op)
:	SOP_Node(net, name, op)
{
}


float SOP_CleanMesh::getVariableValue(int index, int thread)
{
    return SOP_Node::getVariableValue(index, thread);
}


OP_ERROR SOP_CleanMesh::cookMySop(OP_Context &context)
{
	typedef Imath::V3f 						vec3_type;
	typedef vfxgal::simple_mesh<vec3_type>	simple_mesh;

	ScopedCook scc(*this, context, "Performing drd mesh clean");
	if(!scc.good())
		return error();

	double now = context.getTime();
	float minEdgeLen = evalFloat("min_edge_len", 0, now);

	const GU_Detail* gdp0 = inputGeo(0); //This is the geo to clean
	if (!gdp0 ) {
		SOP_ADD_FATAL(SOP_MESSAGE, "Not enough sources specified.");
	}

	gdp->clearAndDestroy();

	// do the mesh clean
	simple_mesh m;
	std::vector<int> point_remap;
	std::vector<int> poly_remap;
	vfxgal::cleanMesh<GEO_Detail>(*gdp0, m, minEdgeLen, &point_remap, &poly_remap);

	// calculate the remapping. This is only necessary because polys which lose degenerate
	// vertices will need to be remapped.
	vfxgal::MeshRemapResult<unsigned int, float> remap_result;
	vfxgal::MeshRemapSettings<int> remap_settings(true, true, true, 0, 0, &point_remap, &poly_remap);
	vfxgal::remapMesh<GEO_Detail, simple_mesh, int>(*gdp0, m, remap_settings, remap_result);

	// construct the cleaned geometry
	add_simple_mesh(*gdp, m);

	// transfer attributes
	GeoAttributeCopier gc(*gdp);
	gc.add(*gdp0, GEO_DETAIL_DICT, GEO_DETAIL_DICT);
	gc.add(*gdp0, GEO_POINT_DICT, GEO_POINT_DICT, &(remap_result.m_pointMapping));
	gc.add(*gdp0, GEO_PRIMITIVE_DICT, GEO_PRIMITIVE_DICT, &(remap_result.m_polyMapping));
	gc.add(*gdp0, GEO_VERTEX_DICT, GEO_VERTEX_DICT, &(remap_result.m_vertexMapping));
	gc.add(*gdp0, GEO_VERTEX_DICT, GEO_VERTEX_DICT, &(remap_result.m_vertexIMapping),
		GeoAttributeCopier::GAC_MERGE);

	gc.apply();

	return error();
}


















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
