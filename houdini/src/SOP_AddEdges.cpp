#include <UT/UT_DSOVersion.h>
#include <OP/OP_OperatorTable.h>
#include <GU/GU_PrimPoly.h>
#include <PRM/PRM_Include.h>
#include <vfxgal/adaptors/houdini.hpp>
#include <vfxgal/algorithm/addMeshEdges.hpp>
#include <vfxgal/algorithm/remapMesh.hpp>
#include <pystring.h>
#include <sstream>
#include "SOP_AddEdges.h"
#include "util/simple_mesh.h"
#include "util/ScopedCook.h"
#include "util/util.h"
#include "util/GeoAttributeCopier.h"


using namespace vfxgal_hou;
namespace pystring = vfxgal::pystring;


void newSopOperator(OP_OperatorTable *table)
{
    table->addOperator(
	    new OP_Operator(SOP_NAME,			// Internal name
			 SOP_LABEL,						// UI name
			 SOP_AddEdges::myConstructor,		// How to build the SOP
			 SOP_AddEdges::myTemplateList,		// My parameters
			 0,								// Min # of sources
			 1,								// Max # of sources
			 SOP_AddEdges::myVariables,		// Local variables
			 0)
	    );
}


static PRM_Name AE_ParmNameEdgesStrAttrib( "edges_string", "Edges" );
static PRM_Default AE_ParmDefaultEdgesStrAttrib(0, "");

PRM_Template SOP_AddEdges::myTemplateList[] = {
	PRM_Template(PRM_STRING, 1, &AE_ParmNameEdgesStrAttrib, &AE_ParmDefaultEdgesStrAttrib, 0, 0, 0, 0, 1,
		"Format: ptA,ptB | ptA-ptB:frac | ptA-ptB:frac,ptS | ptA-ptB:frac,ptC-ptD:frac"),
	PRM_Template()
};

CH_LocalVariable SOP_AddEdges::myVariables[] = {
	{ 0 }		// End the table with a null entry
};


const char* SOP_AddEdges::inputLabel(unsigned int idx) const
{
	return "Target Mesh";
}


OP_Node* SOP_AddEdges::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
    return new SOP_AddEdges(net, name, op);
}


SOP_AddEdges::SOP_AddEdges(OP_Network *net, const char *name, OP_Operator *op)
:	SOP_Node(net, name, op)
{
}


float SOP_AddEdges::getVariableValue(int index, int thread)
{
    return SOP_Node::getVariableValue(index, thread);
}


OP_ERROR SOP_AddEdges::cookMySop(OP_Context &context)
{
	typedef vfxgal::simple_mesh<Imath::V3f>					simple_mesh;
	typedef vfxgal::EdgeIntersection<unsigned int, float> 	edge_intersection;
	typedef std::pair<unsigned int, unsigned int>			edge_type;

	vfxgal_hou::ScopedCook scc(*this, context, "Performing edges add");
	if(!scc.good())
		return error();

	double now = context.getTime();

	// inputs
	const GU_Detail* gdp0 = inputGeo(0);
	if (!gdp0 ) {
		SOP_ADD_FATAL(SOP_MESSAGE, "Not enough sources specified.");
	}

	unsigned int npoints0 = gdp0->points().entries();

	UT_String edgesAttrib;
	evalString(edgesAttrib, "edges_string", 0, now);
	std::string edgesAttribStr = edgesAttrib.toStdString();

	std::vector<std::string> toks;
	pystring::split(edgesAttribStr, toks);
	if(toks.empty())
	{
		duplicateSource(0, context);
		return error();
	}

	simple_mesh m, m2;
	simple_mesh* pm = NULL;
	std::vector<int> points_remap;
	std::vector<int> polys_remap;
	vfxgal::MeshRemapSettings<int> remap_settings(true, true, true, 0, 0, &points_remap, &polys_remap);
	std::map<std::string, unsigned int> new_points;

	// create new points
	{
		std::vector<edge_intersection> edgeints;

		std::string s = pystring::replace(edgesAttribStr, ",", " ");
		std::vector<std::string> toks;
		pystring::split(s, toks);

		for(unsigned int i=0; i<toks.size(); ++i)
		{
			// new point will be in form 'X-Y:f'
			if(toks[i].find('-') != std::string::npos)
			{
				s = pystring::replace(toks[i], "-", " ");
				s = pystring::replace(s, ":", " ");

				std::istringstream strm(s);
				edge_intersection ei;
				try {
					strm >> ei.m_point1 >> ei.m_point2 >> ei.m_u;
				}
				catch(const std::exception&) {
					continue;
				}

				new_points[toks[i]] = npoints0 + edgeints.size();
				edgeints.push_back(ei);
			}
		}

		if(new_points.empty())
		{
			// no new points means no point remapping
			remap_settings.m_genPointRemapping = false;
			remap_settings.m_identity_point_mapping = true;
		}
		else
		{
			vfxgal::addMeshPoints<GEO_Detail, std::vector<edge_intersection>::const_iterator>(
				*gdp0, m, edgeints.begin(), edgeints.end(), &points_remap, &polys_remap);
			pm = &m;
		}
	}

	// add new edges
	std::vector<edge_type> new_edges;
	{
		std::vector<std::string> toks;
		pystring::split(edgesAttribStr, toks);
		for(unsigned int i=0; i<toks.size(); ++i)
		{
			if(toks[i].find(',') != std::string::npos)
			{
				std::string s = pystring::replace(toks[i], ",", " ");
				std::vector<std::string> toks2;
				pystring::split(s, toks2);
				if(toks2.size() != 2)
					continue;

				int ptnum[2] = {-1,-1};
				for(unsigned int j=0; j<2; ++j)
				{
					std::map<std::string, unsigned int>::const_iterator it = new_points.find(toks2[j]);
					if(it == new_points.end())
						sscanf(toks2[j].c_str(), "%d", &ptnum[j]);
					else
						ptnum[j] = static_cast<int>(it->second);
				}

				if((ptnum[0]!=-1) && (ptnum[1]!=-1))
				{
					new_edges.push_back(edge_type(
						static_cast<unsigned int>(ptnum[0]),
						static_cast<unsigned int>(ptnum[1])));
				}
			}
		}
	}

	if(!new_edges.empty())
	{
		if(pm)
		{
			std::vector<int> polys_remap2;
			vfxgal::addMeshEdges<simple_mesh, std::vector<edge_type>::const_iterator>(
				*pm, m2, new_edges.begin(), new_edges.end(), &polys_remap2);

			vfxgal::combinePolyRemapping(polys_remap, polys_remap2, polys_remap);
		}
		else
		{
			vfxgal::addMeshEdges<GEO_Detail, std::vector<edge_type>::const_iterator>(
				*gdp0, m2, new_edges.begin(), new_edges.end(), &polys_remap);
		}

		pm = &m2;
	}

	if(pm)
	{
		// create resulting geometry
		gdp->clearAndDestroy();
		add_simple_mesh(*gdp, *pm);

		// calc attrib remapping
		vfxgal::MeshRemapResult<unsigned int, float> remap_result;
		vfxgal::remapMesh<GEO_Detail, simple_mesh, int>(*gdp0, *pm, remap_settings, remap_result);

		// remap attributes
		GeoAttributeCopier gc(*gdp);
		gc.add(*gdp0, GEO_DETAIL_DICT, GEO_DETAIL_DICT);

		gc.add(*gdp0, GEO_PRIMITIVE_DICT, GEO_PRIMITIVE_DICT, &(remap_result.m_polyMapping));
		gc.add(*gdp0, GEO_VERTEX_DICT, GEO_VERTEX_DICT, &(remap_result.m_vertexMapping));
		gc.add(*gdp0, GEO_VERTEX_DICT, GEO_VERTEX_DICT, &(remap_result.m_vertexIMapping),
			GeoAttributeCopier::GAC_MERGE);

		if(remap_settings.m_genPointRemapping)
		{
			gc.add(*gdp0, GEO_POINT_DICT, GEO_POINT_DICT, &(remap_result.m_pointMapping));
			gc.add(*gdp0, GEO_POINT_DICT, GEO_POINT_DICT, &(remap_result.m_pointIMapping),
				GeoAttributeCopier::GAC_MERGE);
		}
		else
			gc.add(*gdp0, GEO_POINT_DICT, GEO_POINT_DICT);

		gc.apply();
	}
	else
	{
		duplicateSource(0, context);
		return error();
	}

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
