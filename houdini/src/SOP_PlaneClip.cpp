#include <hdk_utils/ScopedCook.h>
#include <hdk_utils/util.h>
#include <hdk_utils/GeoAttributeCopier.h>
#include <UT/UT_DSOVersion.h>
#include <OP/OP_OperatorTable.h>
#include <GU/GU_PrimPoly.h>
#include <houdini/PRM/PRM_Include.h>
#include <dgal/adaptors/houdini.hpp>
#include <dgal/algorithm/clipMesh3D.hpp>
#include <dgal/algorithm/remapMesh.hpp>
#include "SOP_PlaneClip.h"
#include "util/simple_mesh.h"


using namespace clip_sops;
using namespace hdk_utils;


void newSopOperator(OP_OperatorTable *table)
{
    table->addOperator(
	    new OP_Operator(SOP_NAME,			// Internal name
			 SOP_LABEL,						// UI name
			 SOP_PlaneClip::myConstructor,		// How to build the SOP
			 SOP_PlaneClip::myTemplateList,		// My parameters
			 0,								// Min # of sources
			 1,								// Max # of sources
			 SOP_PlaneClip::myVariables,		// Local variables
			 0)
	    );
}


// switcher
static PRM_Name switcherName("switcher");
static PRM_Default switcherList[] = {
    PRM_Default(5, "General"),
    PRM_Default(1, "Advanced")
};

static PRM_Name names[] = {
	PRM_Name(PARM_ORIGIN, "Origin"),
	PRM_Name(PARM_DIRECTION, "Direction"),
};

PRM_Default originDefaults[3] = {
	PRM_Default(0.0f, 0, CH_OLD_EXPRESSION),
	PRM_Default(0.0f, 0, CH_OLD_EXPRESSION),
	PRM_Default(0.0f, 0, CH_OLD_EXPRESSION)
};

PRM_Default directionDefaults[3] = {
	PRM_Default(0.0f, 0, CH_OLD_EXPRESSION),
	PRM_Default(0.0f, 0, CH_OLD_EXPRESSION),
	PRM_Default(1.0f, 0, CH_OLD_EXPRESSION)
};

static PRM_Name PlaneClip_ParmNameClosed( "closed", "Closed" );
static PRM_Default PlaneClip_ParmDefaultClosed(1);

static PRM_Name PlaneClip_ParmNameBreakHoles( "break_holes", "Break Holes" );
static PRM_Default PlaneClip_ParmDefaultBreakHoles(1);

static PRM_Name PlaneClip_ParmNamePolyIDAttrib( "poly_id_attrib", "Poly ID Attribute Name" );
static PRM_Default PlaneClip_ParmDefaultPolyIDAttrib(0, "");

static PRM_Name PlaneClip_ParmNameMinEdgeLength( "min_edge_len", "Fuse Length" );
static PRM_Default PlaneClip_ParmDefaultMinEdgeLength(0.00001f);


PRM_Template SOP_PlaneClip::myTemplateList[] = {
	PRM_Template(PRM_SWITCHER, 3, &switcherName, switcherList),
	PRM_Template(PRM_FLT, 3, &names[0], originDefaults),
	PRM_Template(PRM_FLT, 3, &names[1], directionDefaults),
	PRM_Template(PRM_TOGGLE, 1, &PlaneClip_ParmNameClosed, &PlaneClip_ParmDefaultClosed),
	PRM_Template(PRM_TOGGLE, 1, &PlaneClip_ParmNameBreakHoles, &PlaneClip_ParmDefaultBreakHoles),
	PRM_Template(PRM_STRING, 1, &PlaneClip_ParmNamePolyIDAttrib, &PlaneClip_ParmDefaultPolyIDAttrib),
	PRM_Template(PRM_FLT, 1, &PlaneClip_ParmNameMinEdgeLength, &PlaneClip_ParmDefaultMinEdgeLength),
	PRM_Template()
};

CH_LocalVariable SOP_PlaneClip::myVariables[] = {
	{ 0 }		// End the table with a null entry
};


const char* SOP_PlaneClip::inputLabel(unsigned int idx) const
{
	return "Mesh to Clip";
}


OP_Node* SOP_PlaneClip::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
    return new SOP_PlaneClip(net, name, op);
}


SOP_PlaneClip::SOP_PlaneClip(OP_Network *net, const char *name, OP_Operator *op)
:	SOP_Node(net, name, op)
{
}


float SOP_PlaneClip::getVariableValue(int index, int thread)
{
    return SOP_Node::getVariableValue(index, thread);
}


Imath::V3f SOP_PlaneClip::getOrigin(double time)
{
	Imath::V3f v;
	v.x = evalFloat(PARM_ORIGIN, 0, time);
	v.y = evalFloat(PARM_ORIGIN, 1, time);
	v.z = evalFloat(PARM_ORIGIN, 2, time);
	return v;
}


Imath::V3f SOP_PlaneClip::getDirection(double time)
{
	Imath::V3f v;
	v.x = evalFloat(PARM_DIRECTION, 0, time);
	v.y = evalFloat(PARM_DIRECTION, 1, time);
	v.z = evalFloat(PARM_DIRECTION, 2, time);
	return v;
}


OP_ERROR SOP_PlaneClip::cookMySop(OP_Context &context)
{
	typedef Imath::V3f 						vec3_type;
	typedef dgal::simple_mesh<vec3_type>	simple_mesh;

	hdk_utils::ScopedCook scc(*this, context, "Performing drd plane clip");
	if(!scc.good())
		return error();

	double now = context.getTime();


	const GU_Detail* gdp0 = inputGeo(0); //This is the geo to split
	if (!gdp0 ) {
		SOP_ADD_FATAL(SOP_MESSAGE, "Not enough sources specified.");
	}

	const GEO_PrimList& prims0 = gdp0->primitives();
	const GEO_PointList& points0 = gdp0->points();

	Imath::V3f planeOrigin = getOrigin(now);
	Imath::V3f planeN = getDirection(now);
	Imath::Plane3f cutPlane(planeOrigin, planeN);

	gdp->clearAndDestroy();

	// do the clip
	simple_mesh cmesh;
	std::vector<int> pointIDs;
	std::vector<int> polyIDs;

	std::list<Imath::Plane3f> cutPlanes;
	cutPlanes.push_back(cutPlane);

	UT_String polyIDAttrib;
	bool closed = (evalInt("closed", 0, now) != 0);
	bool break_holes = (evalInt("break_holes", 0, now) != 0);
	float minEdgeLen = evalFloat("min_edge_len", 0, now);
	evalString(polyIDAttrib, "poly_id_attrib", 0, now);
	polyIDAttrib.trimSpace();

	dgal::IntersectType inters = dgal::clipMesh3D<GEO_Detail>(
		*gdp0, cutPlanes.begin(), cutPlanes.end(), closed, break_holes,
		cmesh, &pointIDs, &polyIDs, minEdgeLen);

	if(inters == dgal::INTERSECT_OUTSIDE)
		return error();
	else if(inters == dgal::INTERSECT_INSIDE)
	{
		duplicateSource(0, context);
		return error();
	}

	// construct the clipped geometry
	util::add_simple_mesh(*gdp, cmesh, polyIDAttrib.toStdString(), &polyIDs);

	// calculate the remapping.
	dgal::MeshRemapResult<unsigned int, float> remap_result;
	dgal::MeshRemapSettings<int> remap_settings(true, true, true, 0, 0, &pointIDs, &polyIDs);
	dgal::remapMesh<GEO_Detail, simple_mesh, int>(*gdp0, cmesh, remap_settings, remap_result);

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
