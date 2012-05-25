#ifndef _HDKUTILS_OPPARAMS__H_
#define _HDKUTILS_OPPARAMS__H_

#include <string>
#include <OP/OP_Context.h>
#include <OP/OP_Parameters.h>
#include <UT/UT_String.h>

namespace vfxgal_hou {

	std::string getStringParam(OP_Parameters& node, OP_Context &context,
		const std::string& label, bool trimspace = true);

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
