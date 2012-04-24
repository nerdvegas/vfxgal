#ifndef _CLIP_SOPS_UTIL_SIMPLE_MESH__H_
#define _CLIP_SOPS_UTIL_SIMPLE_MESH__H_

#include <vfxgal/core/adaptors/mesh.hpp>
#include <vfxgal/core/simple_mesh.hpp>
#include <GU/GU_PrimPoly.h>


namespace clip_sops { namespace util {

	void add_simple_mesh(GU_Detail& gdp, const vfxgal::simple_mesh<Imath::V3f>& smesh,
		const std::string& pointIDAttrib = "", const std::vector<int>* pointIDs = NULL,
		const std::string& polyIDAttrib = "", const std::vector<int>* polyIDs = NULL,
		const std::string& cellTypeAttrib = "", unsigned int cellType = 0,
		const std::string& cellIDAttrib = "", unsigned int cellID = 0);

} }

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
