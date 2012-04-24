#include <UT/UT_DSOVersion.h>
#include <OP/OP_OperatorTable.h>
#include <GU/GU_PrimPoly.h>
#include <houdini/PRM/PRM_Include.h>
#include <vfxgal/core/adaptors/houdini.hpp>
#include <vfxgal/core/algorithm/clipMesh3D.hpp>
#include <vfxgal/core/algorithm/remapMesh.hpp>
#include "SOP_HullClip.h"
#include "util/simple_mesh.h"
#include "util/ScopedCook.h"
#include "util/util.h"
#include "util/GeoAttributeCopier.h"

using namespace vfxgal_hou;


void newSopOperator(OP_OperatorTable *table)
{
    table->addOperator(
	    new OP_Operator(SOP_NAME,				// Internal name
			 SOP_LABEL,							// UI name
			 SOP_HullClip::myConstructor,		// How to build the SOP
			 SOP_HullClip::myTemplateList,		// My parameters
			 0,									// Min # of sources
			 2,									// Max # of sources
			 SOP_HullClip::myVariables,			// Local variables
			 0)
	    );
}


// switcher
static PRM_Name switcherName("switcher");
static PRM_Default switcherList[] = {
    PRM_Default(2, "General"),
    PRM_Default(1, "Advanced")
};

static PRM_Name HullClip_ParmNameClosed( "closed", "Closed" );
static PRM_Default HullClip_ParmDefaultClosed(1);

static PRM_Name HullClip_ParmNamePolyIDAttrib( "poly_id_attrib", "Poly ID Attribute Name" );
static PRM_Default HullClip_ParmDefaultPolyIDAttrib(0, "");

static PRM_Name HullClip_ParmNameMinEdgeLength( "min_edge_len", "Fuse Length" );
static PRM_Default HullClip_ParmDefaultMinEdgeLength(0.00001f);


PRM_Template SOP_HullClip::myTemplateList[] =
{
	PRM_Template(PRM_SWITCHER, 3, &switcherName, switcherList),
	PRM_Template(PRM_STRING, 1, &HullClip_ParmNamePolyIDAttrib, &HullClip_ParmDefaultPolyIDAttrib),
	PRM_Template(PRM_TOGGLE, 1, &HullClip_ParmNameClosed, &HullClip_ParmDefaultClosed),
	PRM_Template(PRM_FLT, 1, &HullClip_ParmNameMinEdgeLength, &HullClip_ParmDefaultMinEdgeLength),
	PRM_Template()
};

CH_LocalVariable SOP_HullClip::myVariables[] = {
	{ 0 }		// End the table with a null entry
};


const char* SOP_HullClip::inputLabel(unsigned int idx) const
{
	return (idx == 0)? "Mesh to Clip" : "Clipping hull";
}


OP_Node* SOP_HullClip::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
    return new SOP_HullClip(net, name, op);
}


SOP_HullClip::SOP_HullClip(OP_Network *net, const char *name, OP_Operator *op)
:	SOP_Node(net, name, op)
{
}


float SOP_HullClip::getVariableValue(int index, int thread)
{
    return SOP_Node::getVariableValue(index, thread);
}


OP_ERROR SOP_HullClip::cookMySop(OP_Context &context)
{
	typedef Imath::Plane3<float> 			plane3_type;
	typedef Imath::V3f 						vec3_type;
	typedef vfxgal::simple_mesh<vec3_type>	simple_mesh;

	vfxgal_hou::ScopedCook scc(*this, context, "Performing drd convex-hull clip");
	if(!scc.good())
		return error();

	double now = context.getTime();

	// get inputs
	const GU_Detail* gdp0 = inputGeo(0); //This is the geo to split
	if (!gdp0 ) { SOP_ADD_FATAL(SOP_MESSAGE, "Not enough sources specified."); }
	const GU_Detail* gdp1 = inputGeo(1); //This is the splitting hull
	if (!gdp1 ) { SOP_ADD_FATAL(SOP_MESSAGE, "Not enough sources specified."); }

	gdp->clearAndDestroy();

	// convert hull into collection of planes
	std::list<plane3_type> cutPlanes;
	{
		const GEO_PrimList& prims = gdp1->primitives();
		for(unsigned int i=0; i<prims.entries(); ++i)
		{
			const GEO_PrimPoly* poly = dynamic_cast<const GEO_PrimPoly*>(prims[i]);
			if(poly)
			{
				plane3_type plane;
				vfxgal::get_poly_plane(*poly, plane);
				plane.normal.negate();
				plane.distance = -plane.distance;
				cutPlanes.push_back(plane);
			}
		}
	}

	// do the clip
	simple_mesh cmesh;
	std::vector<int> pointIDs;
	std::vector<int> polyIDs;

	UT_String polyIDAttrib;
	bool closed = (evalInt("closed", 0, now) != 0);
	float minEdgeLen = evalFloat("min_edge_len", 0, now);
	evalString(polyIDAttrib, "poly_id_attrib", 0, now);
	polyIDAttrib.trimSpace();

	vfxgal::IntersectType inters = vfxgal::clipMesh3D<GEO_Detail>(
		*gdp0, cutPlanes.begin(), cutPlanes.end(), closed, true,
		cmesh, &pointIDs, &polyIDs, minEdgeLen);

	if(inters == vfxgal::INTERSECT_OUTSIDE)
		return error();
	else if(inters == vfxgal::INTERSECT_INSIDE)
	{
		duplicateSource(0, context);
		return error();
	}

	// construct the clipped geometry
	util::add_simple_mesh(*gdp, cmesh, polyIDAttrib.toStdString(), &polyIDs);

	// calculate the remapping.
	vfxgal::MeshRemapResult<unsigned int, float> remap_result;
	vfxgal::MeshRemapSettings<int> remap_settings(true, true, true, 0, 0, &pointIDs, &polyIDs);
	vfxgal::remapMesh<GEO_Detail, simple_mesh, int>(*gdp0, cmesh, remap_settings, remap_result);

	// apply the remapping
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
