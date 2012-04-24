#ifndef _HDKUTILS_GEOUTILS__H_
#define _HDKUTILS_GEOUTILS__H_

#include <GU/GU_Detail.h>


namespace hdk_utils {


	/*
	 * @brief deleteAttribute
	 * Delete all attributes with the given name and given class, from geo.
	 * @param geo Geometry to delete attribute(s) from.
	 * @attribClass Attribute class.
	 * @returns The number of attributes that were deleted from geo.
	 */
	unsigned int deleteAttribute(GU_Detail& geo, const UT_String& attribName,
		GEO_AttributeOwner attribClass);


}


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
