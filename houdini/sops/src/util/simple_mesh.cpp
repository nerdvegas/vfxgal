#include "simple_mesh.h"


namespace clip_sops { namespace util {

void add_simple_mesh(GU_Detail& gdp,
	const dgal::simple_mesh<Imath::V3f>& smesh,
	const std::string& pointIDAttrib, const std::vector<int>* pointIDs,
	const std::string& polyIDAttrib, const std::vector<int>* polyIDs,
	const std::string& cellTypeAttrib, unsigned int cellType,
	const std::string& cellIDAttrib, unsigned int cellID)
{
	unsigned int first_point = gdp.points().entries();
	unsigned int first_prim = gdp.primitives().entries();

	// create dest points
	for(std::vector<Imath::V3f>::const_iterator it=smesh.m_points.begin();
		it!=smesh.m_points.end(); ++it)
	{
		GEO_Point* pt = gdp.appendPoint();
		pt->setPos(it->x, it->y, it->z);
	}

	GEO_PointList& points = gdp.points();
	GEO_PrimList& prims = gdp.primitives();

	// create dest polys
	for(unsigned int i=0; i<smesh.m_polys.size(); ++i)
	{
		const std::vector<unsigned int>& cpoly = smesh.m_polys[i];

		GEO_PrimPoly* poly = GU_PrimPoly::build(&gdp, cpoly.size(), GU_POLY_CLOSED, 0);
		for(unsigned int j=0; j<cpoly.size(); ++j)
			poly->getVertex(j).setPt(points[cpoly[j] + first_point]);
	}

	// create cell-type attribute
	if(!cellTypeAttrib.empty())
	{
		int defaultv = -1;
		GB_AttributeRef aref = gdp.addAttribute(cellTypeAttrib.c_str(), sizeof(int),
			GB_ATTRIB_INT, &defaultv, GEO_PRIMITIVE_DICT);

		if(aref.isValid())
		{
			unsigned int npolys = prims.entries();
			for(unsigned int i=first_prim; i<npolys; ++i)
				prims[i]->setValue<int32>(aref, static_cast<int>(cellType));
		}
	}

	// create cell-id attribute
	if(!cellIDAttrib.empty())
	{
		int defaultv = -1;
		GB_AttributeRef aref = gdp.addAttribute(cellIDAttrib.c_str(), sizeof(int),
			GB_ATTRIB_INT, &defaultv, GEO_PRIMITIVE_DICT);

		if(aref.isValid())
		{
			unsigned int npolys = prims.entries();
			for(unsigned int i=first_prim; i<npolys; ++i)
				prims[i]->setValue<int32>(aref, static_cast<int>(cellID));
		}
	}

	// create point-id attribute
	if(!pointIDAttrib.empty())
	{
		int defaultv = -1;
		GB_AttributeRef aref = gdp.addAttribute(pointIDAttrib.c_str(), sizeof(int),
			GB_ATTRIB_INT, &defaultv, GEO_POINT_DICT);

		if(aref.isValid() && pointIDs)
		{
			unsigned int sz = std::min(points.entries()-first_point,
				static_cast<unsigned int>(pointIDs->size()));

			for(unsigned int i=0; i<sz; ++i)
				points[i + first_point]->setValue<int32>(aref, (*pointIDs)[i]);
		}
	}

	// create poly-id attribute
	if(!polyIDAttrib.empty())
	{
		int defaultv = 0;
		GB_AttributeRef aref = gdp.addAttribute(polyIDAttrib.c_str(), sizeof(int),
			GB_ATTRIB_INT, &defaultv, GEO_PRIMITIVE_DICT);

		if(aref.isValid() && polyIDs)
		{
			unsigned int sz = std::min(prims.entries()-first_prim,
				static_cast<unsigned int>(polyIDs->size()));

			for(unsigned int i=0; i<sz; ++i)
				prims[i + first_prim]->setValue<int32>(aref, (*polyIDs)[i]);
		}
	}
}

} }









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
