#ifndef _SOP_PLANESPLIT__H_
#define _SOP_PLANESPLIT__H_

#include <OpenEXR/ImathVec.h>
#include <SOP/SOP_Node.h>

#define SOP_NAME "vfxgal_plane_clip"
#define SOP_LABEL "vfxgal Plane Clip"

#define PARM_ORIGIN "origin"
#define PARM_DIRECTION "direction"

namespace vfxgal_hou
{

	class SOP_PlaneClip : public SOP_Node
	{
	public:
		static OP_Node* myConstructor(OP_Network*, const char*, OP_Operator*);
		virtual const char* inputLabel(unsigned idx) const;
		static PRM_Template myTemplateList[];
		static CH_LocalVariable	myVariables[];

	protected:
		SOP_PlaneClip(OP_Network *net, const char *name, OP_Operator *op);
		virtual ~SOP_PlaneClip(){}

		virtual OP_ERROR cookMySop(OP_Context &context);
		virtual float getVariableValue(int index, int thread);

	protected:

		Imath::V3f getOrigin(double time);
		Imath::V3f getDirection(double time);
	};

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
